/******************************************************************************************************************
 ** This sample demonstrates how to use two zeds_ with the ZED SDK, each grab are in a separate thread             **
 ** This sample has been tested with 3 zeds_ in HD720@30fps resolution. Linux only.                                **
 *******************************************************************************************************************/

#include <sl/Camera.hpp>
#include "zed_components/zed_camera_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class ZedPublisher : public rclcpp::Node
{
  public:
    ZedPublisher();
    ~ZedPublisher();

  private:
    int main_loop();
    void get_and_publish(sl::Camera& zed, bool& run);
    void fillCamInfo(sl::Camera& zed, sl::Resolution& resolution, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg, 
              std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg, std::string leftFrameId,
              std::string rightFrameId, bool rawParam = false);
    void publish_odometry(sl::Camera& zed, bool& run);

    bool run_;
    int res_w_;
    int res_h_;
    float downsampling_;
    int fps_;
    std::uint32_t sn_l_;
    std::uint32_t sn_r_;
    std::unordered_map<std::uint32_t, std::string> camera_frames_;
    std::unordered_map<std::uint32_t, std::array<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>, 2>> image_publishers_;
    std::unordered_map<std::uint32_t, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>> info_publishers_;
    size_t nb_detected_zed_;
    std::vector<std::shared_ptr<sl::Camera>> zeds_;
    std::vector<std::thread> thread_pool_;
    std::mutex zed_mutex_;
};


class ZedOdom
{
  public:
    ZedOdom();
    ~ZedOdom();

  private:
    void getOdom();

    bool mSensor2BaseTransfValid;
    bool mSensor2CameraTransfValid;
    bool mCamera2BaseTransfValid;
    bool mPosTrackingStarted;

    bool mInitOdomWithPose;
    bool mFloorAlignment;
    bool mTwoDMode;

    // ----> initialization Transform listener
    std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
    // <---- initialization Transform listener

     // ----> TF Transforms
    tf2::Transform mMap2OdomTransf;         // Coordinates of the odometry frame in map frame
    tf2::Transform mOdom2BaseTransf;        // Coordinates of the base in odometry frame
    tf2::Transform mMap2BaseTransf;         // Coordinates of the base in map frame
    tf2::Transform mSensor2BaseTransf;      // Coordinates of the base frame in sensor frame
    tf2::Transform mSensor2CameraTransf;    // Coordinates of the camera frame in sensor frame
    tf2::Transform mCamera2BaseTransf;      // Coordinates of the base frame in camera frame
    // <---- TF Transforms

    std::string mCameraFrameId;
    std::string mBaseFrameId;
    std::string mDepthFrameId;

    rclcpp::Node& mNode;
    sl::Camera& mZed;

}

ZedOdom::ZedOdom(rclcpp::Node& node, sl::Camera& zed) : mNode{node}, mZed{zed}, mSensor2BaseTransfValid{false}, mSensor2CameraTransfValid{false}, mCamera2BaseTransfValid{false},
                    mInitOdomWithPose{true}, mFloorAlignment{false}, mTwoDMode{false}
{
  mTfBuffer = std::make_unique<tf2_ros::Buffer>(node->get_clock())
  // Initialize tracking parameters
}

void ZedOdom::getOdom()
{
  if (!mPosTrackingStarted){
    startPosTracking();
  }
  processOdometry();
  publish_odometry();
}

ZedPublisher::ZedPublisher() : Node{"zed_rgbd_publisher"}, run_{true}, nb_detected_zed_{0}
{
  rclcpp::QoS qos_profile(10);

  sn_l_ = this->declare_parameter<int>("left_sn", 28846348);
  sn_r_ = this->declare_parameter<int>("right_sn", 21128703);
  res_w_ = this->declare_parameter<int>("width", 1280);
  res_h_ = this->declare_parameter<int>("height", 720);
  downsampling_ = this->declare_parameter<float>("downsampling", 1.0);
  fps_ = this->declare_parameter<int>("fps", 15);

  auto rgb_publisher_l_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed2_l/rgb/info", qos_profile);
  auto rgb_publisher_r_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("zed2_r/rgb/info", qos_profile);
  auto rgb_publisher_l_ = this->create_publisher<sensor_msgs::msg::Image>("zed2_l/rgb", qos_profile);
  auto rgb_publisher_r_ = this->create_publisher<sensor_msgs::msg::Image>("zed2_r/rgb", qos_profile);
  auto depth_publisher_l_ = this->create_publisher<sensor_msgs::msg::Image>("zed2_l/depth", qos_profile);
  auto depth_publisher_r_ = this->create_publisher<sensor_msgs::msg::Image>("zed2_r/depth", qos_profile);
  auto odom_publisher = create_publisher<nav_msgs::msg::Odometry>("zed2_l/odom", qos_profile);

  image_publishers_ = {{sn_l_, {rgb_publisher_l_, depth_publisher_l_}}, {sn_r_, {rgb_publisher_r_, depth_publisher_r_}}};
  info_publishers_ = {{sn_l_, rgb_publisher_l_info_}, {sn_r_, rgb_publisher_r_info_}};
  camera_frames_ = {{sn_l_, "zed2_l_left_camera_optical_frame"},  {sn_r_, "zed2_r_left_camera_optical_frame"}};

  main_loop();
};


ZedPublisher::~ZedPublisher(){
  // stop all running threads
  run_ = false;

  // Wait for every thread to be stopped
  for (size_t z = 0; z < nb_detected_zed_; z++)
        if (zeds_.at(z)->isOpened())
            thread_pool_[z].join();
};

int ZedPublisher::main_loop() {
  sl::InitParameters init_parameters;
  init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  init_parameters.camera_resolution = sl::RESOLUTION::HD720;
  init_parameters.camera_fps = fps_;
  init_parameters.depth_minimum_distance = 0.5f;
  init_parameters.depth_maximum_distance = 5.0f;
  init_parameters.coordinate_units = sl::UNIT::METER;
  init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    
  std::vector< sl::DeviceProperties> devList = sl::Camera::getDeviceList();
    nb_detected_zed_ = devList.size();

  for (size_t z = 0; z < nb_detected_zed_; z++) {
    zeds_.emplace_back(std::make_shared<sl::Camera>());
    std::cout << "ID : " << devList[z].id << " ,model : " << devList[z].camera_model << " , S/N : " << devList[z].serial_number << " , state : "<<devList[z].camera_state<<std::endl;
  }

  if (nb_detected_zed_ == 0) {
    std::cout << "Did not detect ZED cameras, exit program" << std::endl;
      return EXIT_FAILURE;
  }
    
  std::cout << nb_detected_zed_ << " ZED Detected" << std::endl;
  // try to open every detected cameras
  for (size_t z = 0; z < nb_detected_zed_; z++) {
      init_parameters.input.setFromCameraID(z);
      sl::ERROR_CODE err = zeds_.at(z)->open(init_parameters);
      if (err == sl::ERROR_CODE::SUCCESS) {
          auto cam_info = zeds_.at(z)->getCameraInformation();
          std::cout << cam_info.camera_model << ", ID: " << z << ", SN: " << cam_info.serial_number << " Opened" << std::endl;
      } else {
        std::cout << "ZED ID:" << z << " Error: " << err << std::endl;
          zeds_.at(z)->close();
      }
  }
    
  // Create a grab thread for each opened camera

  for (size_t z = 0; z < nb_detected_zed_; z++){
      if (zeds_.at(z)->isOpened()) {
          // camera acquisition thread
        std::cout << "Camera thread " << z+1 << " spawned" << std::endl;
        thread_pool_.emplace_back(&ZedPublisher::get_and_publish, this, std::ref(*zeds_.at(z)), std::ref(run_));
        if (zeds_.at(z)->getCameraInformation().serial_number == sn_l_){
          thread_pool_.emplace_back(&ZedPublisher::publish_odometry, this, std::ref(*zeds_.at(z)), std::ref(run_));
        }
      }
	}
};


void ZedPublisher::get_and_publish(sl::Camera& zed, bool& run) {
  const std::lock_guard<std::mutex> lock(zed_mutex_);
  sl::Mat mat_depth;
  sl::Mat mat_image;
  auto left_cam_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  auto right_cam_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  auto sn = zed.getCameraInformation().serial_number;
  sl::Resolution res(int(res_w_ * downsampling_), int(res_h_ * downsampling_));
  fillCamInfo(zed, res, left_cam_info_msg, right_cam_info_msg, camera_frames_.at(sn), std::string{"no_right_frame_id"});
  while (run) {
    // grab current images and compute depth
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveImage(mat_image, sl::VIEW::LEFT, sl::MEM::CPU, res);
      zed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, res);
      auto ts = sl_tools::slTime2Ros(zed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
      auto image = sl_tools::imageToROSmsg(mat_image, camera_frames_.at(sn), ts);
      left_cam_info_msg->header.stamp = ts;
      auto depth_img = sl_tools::imageToROSmsg(mat_depth, camera_frames_.at(sn), ts);
      image_publishers_.at(sn).at(0)->publish(*image);
      image_publishers_.at(sn).at(1)->publish(*depth_img);
      info_publishers_.at(sn)->publish(*left_cam_info_msg);
    }
    sl::sleep_ms(2);
  }
  zed.close();
};

void publish_odometry(sl::Camera& zed, bool& run){
  const std::lock_guard<std::mutex> lock(zed_mutex_);
}


bool ZedCamera::startPosTracking()
{
  if (mDepthDisabled)
  {
    RCLCPP_WARN(mNode->get_logger(), "Cannot start Positional Tracking if `depth.quality` is set to `0` [NONE]");
    return false;
  }

  RCLCPP_INFO_STREAM(mNode->get_logger(), "*** Starting Positional Tracking ***");

  RCLCPP_INFO(mNode->get_logger(), " * Waiting for valid static transformations...");

  bool transformOk = false;
  double elapsed = 0.0;
  mPosTrackingReady = false;

  auto start = std::chrono::high_resolution_clock::now();

  do
  {
    transformOk = setPose(mInitialBasePose[0], mInitialBasePose[1], mInitialBasePose[2], mInitialBasePose[3],
                          mInitialBasePose[4], mInitialBasePose[5]);

    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start)
                  .count();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (elapsed > 10000)
    {
      RCLCPP_WARN(mNode->get_logger(), " !!! Failed to get static transforms. Is the 'ROBOT STATE PUBLISHER' node correctly "
                                "working? ");
      break;
    }

  } while (transformOk == false);

  if (transformOk)
  {
    RCLCPP_DEBUG(mNode->get_logger(), "Time required to get valid static transforms: %g sec", elapsed / 1000.);
  }

  RCLCPP_INFO(mNode->get_logger(), "Initial ZED left camera pose (ZED pos. tracking): ");
  RCLCPP_INFO(mNode->get_logger(), " * T: [%g,%g,%g]", mInitialPoseSl.getTranslation().x, mInitialPoseSl.getTranslation().y,
              mInitialPoseSl.getTranslation().z);
  RCLCPP_INFO(mNode->get_logger(), " * Q: [%g,%g,%g,%g]", mInitialPoseSl.getOrientation().ox,
              mInitialPoseSl.getOrientation().oy, mInitialPoseSl.getOrientation().oz,
              mInitialPoseSl.getOrientation().ow);

  if (mAreaMemoryDbPath != "" && !sl_tools::file_exist(mAreaMemoryDbPath))
  {
    mAreaMemoryDbPath = "";
    RCLCPP_WARN_STREAM(mNode->get_logger(),
                       "'area_memory_db_path' path doesn't exist or is unreachable: " << mAreaMemoryDbPath);
  }

  // Tracking parameters
  sl::PositionalTrackingParameters trackParams;

  trackParams.area_file_path = mAreaMemoryDbPath.c_str();

  mPoseSmoothing = false;  // Always false. Pose Smoothing is to be enabled only for VR/AR applications
  trackParams.enable_pose_smoothing = mPoseSmoothing;

  trackParams.enable_area_memory = mAreaMemory;
  trackParams.enable_imu_fusion = mImuFusion;
  trackParams.initial_world_transform = mInitialPoseSl;

  trackParams.set_floor_as_origin = mFloorAlignment;

  sl::ERROR_CODE err = mZed.enablePositionalTracking(trackParams);

  if (err == sl::ERROR_CODE::SUCCESS)
  {
    mPosTrackingStarted = true;
  }
  else
  {
    mPosTrackingStarted = false;

    RCLCPP_WARN(mNode->get_logger(), "Tracking not started: %s", sl::toString(err).c_str());
  }

  if (mPosTrackingStarted)
  {
    startPathPubTimer(mPathPubRate);
  }

  return mPosTrackingStarted;
}



bool ZedCamera::getCamera2BaseTransform()
{
  RCLCPP_DEBUG(mNode->get_logger(), "Getting static TF from '%s' to '%s'", mCameraFrameId.c_str(), mBaseFrameId.c_str());

  mCamera2BaseTransfValid = false;
  static bool first_error = true;

  // ----> Static transforms
  // Sensor to Base link
  try
  {
    // Save the transformation
    geometry_msgs::msg::TransformStamped c2b =
        mTfBuffer->lookupTransform(mCameraFrameId, mBaseFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));

    // Get the TF2 transformation
    // tf2::fromMsg(c2b.transform, mCamera2BaseTransf);
    geometry_msgs::msg::Transform in = c2b.transform;
    mCamera2BaseTransf.setOrigin(tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
    // w at the end in the constructor
    mCamera2BaseTransf.setRotation(tf2::Quaternion(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

    double roll, pitch, yaw;
    tf2::Matrix3x3(mCamera2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(mNode->get_logger(), "Static transform Camera Center to Base [%s -> %s]", mCameraFrameId.c_str(),
                mBaseFrameId.c_str());
    RCLCPP_INFO(mNode->get_logger(), " * Translation: {%.3f,%.3f,%.3f}", mCamera2BaseTransf.getOrigin().x(),
                mCamera2BaseTransf.getOrigin().y(), mCamera2BaseTransf.getOrigin().z());
    RCLCPP_INFO(mNode->get_logger(), " * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
  }
  catch (tf2::TransformException& ex)
  {
    if (!first_error)
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_DEBUG_THROTTLE(mNode->get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
      RCLCPP_WARN_THROTTLE(mNode->get_logger(), steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
                           mCameraFrameId.c_str(), mBaseFrameId.c_str());
      RCLCPP_WARN_THROTTLE(mNode->get_logger(), steady_clock, 1.0,
                           "Note: one of the possible cause of the problem is the absense of an instance "
                           "of the `robot_state_publisher` node publishing the correct static TF transformations "
                           "or a modified URDF not correctly reproducing the ZED "
                           "TF chain '%s' -> '%s' -> '%s'",
                           mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      first_error = false;
    }

    mCamera2BaseTransf.setIdentity();
    return false;
  }

  // <---- Static transforms
  mCamera2BaseTransfValid = true;
  return true;
}

bool ZedCamera::getSens2CameraTransform()
{
  RCLCPP_DEBUG(mNode->get_logger(), "Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mCameraFrameId.c_str());

  mSensor2CameraTransfValid = false;

  static bool first_error = true;

  // ----> Static transforms
  // Sensor to Camera Center
  try
  {
    // Save the transformation
    geometry_msgs::msg::TransformStamped s2c =
        mTfBuffer->lookupTransform(mDepthFrameId, mCameraFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));

    // Get the TF2 transformation
    // tf2::fromMsg(s2c.transform, mSensor2CameraTransf);
    geometry_msgs::msg::Transform in = s2c.transform;
    mSensor2CameraTransf.setOrigin(tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
    // w at the end in the constructor
    mSensor2CameraTransf.setRotation(tf2::Quaternion(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

    double roll, pitch, yaw;
    tf2::Matrix3x3(mSensor2CameraTransf.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(mNode->get_logger(), "Static transform Sensor to Camera Center [%s -> %s]", mDepthFrameId.c_str(),
                mCameraFrameId.c_str());
    RCLCPP_INFO(mNode->get_logger(), " * Translation: {%.3f,%.3f,%.3f}", mSensor2CameraTransf.getOrigin().x(),
                mSensor2CameraTransf.getOrigin().y(), mSensor2CameraTransf.getOrigin().z());
    RCLCPP_INFO(mNode->get_logger(), " * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
  }
  catch (tf2::TransformException& ex)
  {
    if (!first_error)
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_DEBUG_THROTTLE(mNode->get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
      RCLCPP_WARN_THROTTLE(mNode->get_logger(), steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
                           mDepthFrameId.c_str(), mCameraFrameId.c_str());
      RCLCPP_WARN_THROTTLE(mNode->get_logger(), steady_clock, 1.0,
                           "Note: one of the possible cause of the problem is the absense of an instance "
                           "of the `robot_state_publisher` node publishing the correct static TF transformations "
                           "or a modified URDF not correctly reproducing the ZED "
                           "TF chain '%s' -> '%s' -> '%s'",
                           mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      first_error = false;
    }

    mSensor2CameraTransf.setIdentity();
    return false;
  }

  // <---- Static transforms

  mSensor2CameraTransfValid = true;
  return true;
}

bool ZedCamera::getSens2BaseTransform()
{
  RCLCPP_DEBUG(mNode->get_logger(), "Getting static TF from '%s' to '%s'", mDepthFrameId.c_str(), mBaseFrameId.c_str());

  mSensor2BaseTransfValid = false;
  static bool first_error = true;

  // ----> Static transforms
  // Sensor to Base link
  try
  {
    // Save the transformation
    geometry_msgs::msg::TransformStamped s2b =
        mTfBuffer->lookupTransform(mDepthFrameId, mBaseFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));

    // Get the TF2 transformation
    // tf2::fromMsg(s2b.transform, mSensor2BaseTransf);
    geometry_msgs::msg::Transform in = s2b.transform;
    mSensor2BaseTransf.setOrigin(tf2::Vector3(in.translation.x, in.translation.y, in.translation.z));
    // w at the end in the constructor
    mSensor2BaseTransf.setRotation(tf2::Quaternion(in.rotation.x, in.rotation.y, in.rotation.z, in.rotation.w));

    double roll, pitch, yaw;
    tf2::Matrix3x3(mSensor2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(mNode->get_logger(), "Static transform Sensor to Base [%s -> %s]", mDepthFrameId.c_str(),
                mBaseFrameId.c_str());
    RCLCPP_INFO(mNode->get_logger(), " * Translation: {%.3f,%.3f,%.3f}", mSensor2BaseTransf.getOrigin().x(),
                mSensor2BaseTransf.getOrigin().y(), mSensor2BaseTransf.getOrigin().z());
    RCLCPP_INFO(mNode->get_logger(), " * Rotation: {%.3f,%.3f,%.3f}", roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
  }
  catch (tf2::TransformException& ex)
  {
    if (!first_error)
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_DEBUG_THROTTLE(mNode->get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
      RCLCPP_WARN_THROTTLE(mNode->get_logger(), steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
                           mDepthFrameId.c_str(), mBaseFrameId.c_str());
      RCLCPP_WARN_THROTTLE(mNode->get_logger(), steady_clock, 1.0,
                           "Note: one of the possible cause of the problem is the absense of an instance "
                           "of the `robot_state_publisher` node publishing the correct static TF transformations "
                           "or a modified URDF not correctly reproducing the ZED "
                           "TF chain '%s' -> '%s' -> '%s'",
                           mBaseFrameId.c_str(), mCameraFrameId.c_str(), mDepthFrameId.c_str());
      first_error = false;
    }

    mSensor2BaseTransf.setIdentity();
    return false;
  }

  // <---- Static transforms

  mSensor2BaseTransfValid = true;
  return true;
}


void ZedOdom::processOdometry()
{
  //adapted from https://github.com/stereolabs/zed-ros2-wrapper/blob/de1dfc4cde9ada3173074a05e14631d4d56e442c/zed_components/src/zed_camera/src/zed_camera_component.cpp
  
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);

  if (!mSensor2BaseTransfValid)
  {
    getSens2BaseTransform();
  }

  if (!mSensor2CameraTransfValid)
  {
    getSens2CameraTransform();
  }

  if (!mCamera2BaseTransfValid)
  {
    getCamera2BaseTransform();
  }

  if (!mInitOdomWithPose)
  {
    sl::Pose deltaOdom;

    mPosTrackingStatus = mZed.getPosition(deltaOdom, sl::REFERENCE_FRAME::CAMERA);

    sl::Translation translation = deltaOdom.getTranslation();
    sl::Orientation quat = deltaOdom.getOrientation();

#if 0
        RCLCPP_DEBUG(mNode->get_logger(), "delta ODOM [%s] - %.2f,%.2f,%.2f %.2f,%.2f,%.2f,%.2f",
                     sl::toString(mPosTrackingStatus).c_str(),
                     translation(0), translation(1), translation(2),
                     quat(0), quat(1), quat(2), quat(3));
#endif

    if (mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::OK ||
        mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::SEARCHING ||
        mPosTrackingStatus == sl::POSITIONAL_TRACKING_STATE::FPS_TOO_LOW)
    {
      // Transform ZED delta odom pose in TF2 Transformation
      tf2::Transform deltaOdomTf;
      deltaOdomTf.setOrigin(tf2::Vector3(translation(0), translation(1), translation(2)));
      // w at the end in the constructor
      deltaOdomTf.setRotation(tf2::Quaternion(quat(0), quat(1), quat(2), quat(3)));

      // delta odom from sensor to base frame
      tf2::Transform deltaOdomTf_base = mSensor2BaseTransf.inverse() * deltaOdomTf * mSensor2BaseTransf;

      // Propagate Odom transform in time
      mOdom2BaseTransf = mOdom2BaseTransf * deltaOdomTf_base;

      if (mTwoDMode)
      {
        tf2::Vector3 tr_2d = mOdom2BaseTransf.getOrigin();
        tr_2d.setZ(mFixedZValue);
        mOdom2BaseTransf.setOrigin(tr_2d);

        double roll, pitch, yaw;
        tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

        tf2::Quaternion quat_2d;
        quat_2d.setRPY(0.0, 0.0, yaw);

        mOdom2BaseTransf.setRotation(quat_2d);
      }

#if 0
            double roll, pitch, yaw;
            tf2::Matrix3x3(mOdom2BaseTransf.getRotation()).getRPY(roll, pitch, yaw);

            RCLCPP_DEBUG(mNode->get_logger(), "+++ Odometry [%s -> %s] - {%.3f,%.3f,%.3f} {%.3f,%.3f,%.3f}",
                         mOdomFrameId.c_str(), mBaseFrameId.c_str(),
                         mOdom2BaseTransf.getOrigin().x(),
                         mOdom2BaseTransf.getOrigin().y(),
                         mOdom2BaseTransf.getOrigin().z(),
                         roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
#endif

      // Publish odometry message
      auto mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
      publishOdom(mOdom2BaseTransf, deltaOdom, mFrameTimestamp);
      mPosTrackingReady = true;
    }
  }
  else if (mFloorAlignment)
  {
    RCLCPP_DEBUG_THROTTLE(mNode->get_logger(), steady_clock, 5.0,
                          "Odometry will be published as soon as the floor as been detected for the first time");
  }
}


void ZedOdom::publishOdom(tf2::Transform& odom2baseTransf, sl::Pose& slPose, rclcpp::Time t)
{
  //adapted from https://github.com/stereolabs/zed-ros2-wrapper/blob/de1dfc4cde9ada3173074a05e14631d4d56e442c/zed_components/src/zed_camera/src/zed_camera_component.cpp
  
  odomMsgPtr odomMsg = std::make_unique<nav_msgs::msg::Odometry>();

  odomMsg->header.stamp = t;
  odomMsg->header.frame_id = mOdomFrameId;  // frame
  odomMsg->child_frame_id = mBaseFrameId;   // camera_frame

  // Add all value in odometry message
  odomMsg->pose.pose.position.x = odom2baseTransf.getOrigin().x();
  odomMsg->pose.pose.position.y = odom2baseTransf.getOrigin().y();
  odomMsg->pose.pose.position.z = odom2baseTransf.getOrigin().z();
  odomMsg->pose.pose.orientation.x = odom2baseTransf.getRotation().x();
  odomMsg->pose.pose.orientation.y = odom2baseTransf.getRotation().y();
  odomMsg->pose.pose.orientation.z = odom2baseTransf.getRotation().z();
  odomMsg->pose.pose.orientation.w = odom2baseTransf.getRotation().w();

  // Odometry pose covariance
  for (size_t i = 0; i < odomMsg->pose.covariance.size(); i++)
  {
    odomMsg->pose.covariance[i] = static_cast<double>(slPose.pose_covariance[i]);

    if (mTwoDMode)
    {
      if (i == 14 || i == 21 || i == 28)
      {
        odomMsg->pose.covariance[i] = 1e-9;  // Very low covariance if 2D mode
      }
      else if ((i >= 2 && i <= 4) || (i >= 8 && i <= 10) || (i >= 12 && i <= 13) || (i >= 15 && i <= 16) ||
               (i >= 18 && i <= 20) || (i == 22) || (i >= 24 && i <= 27))
      {
        odomMsg->pose.covariance[i] = 0.0;
      }
    }
  }

  // Publish odometry message
  publish_odometry->publish(std::move(odomMsg));
}

void ZedPublisher::fillCamInfo(sl::Camera& zed, sl::Resolution& resolution, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
                            std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg, std::string leftFrameId,
                            std::string rightFrameId, bool rawParam)
{
	//adapted from https://github.com/stereolabs/zed-ros2-wrapper/blob/de1dfc4cde9ada3173074a05e14631d4d56e442c/zed_components/src/zed_camera/src/zed_camera_component.cpp
	bool extrinsic_in_camera_frame = false;
  sl::CalibrationParameters zedParam;

  #if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    if (rawParam)
    {
      zedParam = zed.getCameraInformation(resolution).calibration_parameters_raw;  // ok
    }
    else
    {
      zedParam = zed.getCameraInformation(resolution).calibration_parameters;  // ok
    }
  #else
    if (rawParam)
    {
      zedParam = zed.getCameraInformation(resolution).camera_configuration.calibration_parameters_raw;
    }
    else
    {
      zedParam = zed.getCameraInformation(resolution).camera_configuration.calibration_parameters;
    }
  #endif

  float baseline = zedParam.getCameraBaseline();
  leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  leftCamInfoMsg->d.resize(5);
  rightCamInfoMsg->d.resize(5);
  leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
  leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
  leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];    // k3
  leftCamInfoMsg->d[3] = zedParam.left_cam.disto[2];    // p1
  leftCamInfoMsg->d[4] = zedParam.left_cam.disto[3];    // p2
  rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
  rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
  rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4];  // k3
  rightCamInfoMsg->d[3] = zedParam.right_cam.disto[2];  // p1
  rightCamInfoMsg->d[4] = zedParam.right_cam.disto[3];  // p2
  leftCamInfoMsg->k.fill(0.0);
  rightCamInfoMsg->k.fill(0.0);
  leftCamInfoMsg->k[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->k[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->k[4] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->k[5] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->k[8] = 1.0;
  rightCamInfoMsg->k[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->k[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->k[4] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->k[5] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->k[8] = 1.0;
  leftCamInfoMsg->r.fill(0.0);
  rightCamInfoMsg->r.fill(0.0);

  for (size_t i = 0; i < 3; i++)
  {
    // identity
    rightCamInfoMsg->r[i + i * 3] = 1;
    leftCamInfoMsg->r[i + i * 3] = 1;
  }

  #if ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION < 1
    if (rawParam)
    {
      std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
      float* p = R_.data();

      for (int i = 0; i < 9; i++)
      {
        rightCamInfoMsg->r[i] = p[i];
      }
    }
  #else
  if (rawParam)
  {
    if (extrinsic_in_camera_frame)
    {  // Camera frame (Z forward, Y down, X right)

      std::vector<float> R_ = sl_tools::convertRodrigues(zedParam.R);
      float* p = R_.data();

      for (int i = 0; i < 9; i++)
      {
        rightCamInfoMsg->r[i] = p[i];
      }
    }
    else
    {  // ROS frame (X forward, Z up, Y left)
      for (int i = 0; i < 9; i++)
      {
        rightCamInfoMsg->r[i] = zedParam.stereo_transform.getRotationMatrix().r[i];
      }
    }
  }
  #endif

  leftCamInfoMsg->p.fill(0.0);
  rightCamInfoMsg->p.fill(0.0);
  leftCamInfoMsg->p[0] = static_cast<double>(zedParam.left_cam.fx);
  leftCamInfoMsg->p[2] = static_cast<double>(zedParam.left_cam.cx);
  leftCamInfoMsg->p[5] = static_cast<double>(zedParam.left_cam.fy);
  leftCamInfoMsg->p[6] = static_cast<double>(zedParam.left_cam.cy);
  leftCamInfoMsg->p[10] = 1.0;
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  rightCamInfoMsg->p[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
  rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
  rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
  rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
  rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
  rightCamInfoMsg->p[10] = 1.0;
  leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(resolution.width);
  leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(resolution.height);
  leftCamInfoMsg->header.frame_id = leftFrameId;
  rightCamInfoMsg->header.frame_id = rightFrameId;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedPublisher>());
  rclcpp::shutdown();
  return 0;
}
