// adapted from https://github.com/stereolabs/zed-ros2-wrapper/blob/de1dfc4cde9ada3173074a05e14631d4d56e442c/zed_components/src/zed_camera/src/zed_camera_component.cpp


#include <sl/Camera.hpp>
#include "zed_components/zed_camera_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

class ZedOdom
{
  public:
    ZedOdom(rclcpp::Node* node, std::shared_ptr<sl::Camera> zed);
    ~ZedOdom(){};
    void getOdom(std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPublisher, bool& run);

  private:
    template<typename T> void getParam(std::string paramName, T defValue, T& outVal, std::string log_info=std::string());
    void getPosTrackingParams();
    void initTransforms();
    bool setPose(float xt, float yt, float zt, float rr, float pr, float yr);
    bool startPosTracking();
    bool getCamera2BaseTransform();
    bool getSens2CameraTransform();
    bool getSens2BaseTransform();
    void processOdometry(std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPublisher);
    void publishOdom(tf2::Transform& odom2baseTransf, sl::Pose& slPose, rclcpp::Time t, std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPublisher);

    bool mSensor2BaseTransfValid = false;
    bool mSensor2CameraTransfValid = false;
    bool mCamera2BaseTransfValid = false;
    bool mPosTrackingStarted = false;

    bool mInitOdomWithPose = false;
    bool mFloorAlignment = false;
    bool mTwoDMode = false;
    bool mDepthDisabled = false;
    bool mPosTrackingEnabled = true;
    bool mPublishTF = false;
    bool mPublishMapTF = false;
    bool mPublishImuTF = false;
    bool mAreaMemory = true;
    bool mImuFusion = true;


    // ----> initialization Transform listener
    std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;
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
    std::string mMapFrameId;
    std::string mOdomFrameId;
    std::string mDepthFrameId;
    std::string mAreaMemoryDbPath;

    rclcpp::Node * mNode;
    std::shared_ptr<sl::Camera>  mZed;

    std::vector<double> mInitialBasePose;
    sl::Transform mInitialPoseSl;

    float mPathPubRate = 2.0;
    int mPathMaxCount = -1;
    float mFixedZValue = 0.0;

    rclcpp::QoS mPoseQos;
    sl::POSITIONAL_TRACKING_STATE mPosTrackingStatus;
};

ZedOdom::ZedOdom(rclcpp::Node * node, std::shared_ptr<sl::Camera> zed) : mNode{node}, mZed{zed}, mPoseQos(1)
{
  mTfBuffer = std::make_unique<tf2_ros::Buffer>(mNode->get_clock());
  mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
  // Initialize tracking parameters
  mInitialBasePose = std::vector<double>(6, 0.0);
  getPosTrackingParams();
}

void ZedOdom::getOdom(std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPublisher, bool& run)
{
  while(run && mZed->isOpened()){
      if (!mPosTrackingStarted){
        startPosTracking();
      }
      processOdometry(odomPublisher);
      sl::sleep_ms(1);
  }
}

template <typename T>
void ZedOdom::getParam(std::string paramName, T defValue, T& outVal, std::string log_info)
{
  mNode->declare_parameter(paramName, rclcpp::ParameterValue(defValue));

  if (!mNode->get_parameter(paramName, outVal))
  {
    RCLCPP_WARN_STREAM(mNode->get_logger(), "The parameter '"
                                         << paramName << "' is not available or is not valid, using the default value: "
                                         << defValue);
  }

  if (!log_info.empty())
  {
    RCLCPP_INFO_STREAM(mNode->get_logger(), log_info << outVal);
  }
}


void ZedOdom::getPosTrackingParams()
{
  rclcpp::Parameter paramVal;
  std::string paramName;

  rmw_qos_history_policy_t qos_hist = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  int qos_depth = 1;
  rmw_qos_reliability_policy_t qos_reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_durability_policy_t qos_durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  RCLCPP_INFO(mNode->get_logger(), "*** POSITIONAL TRACKING parameters ***");

  getParam("pos_tracking.pos_tracking_enabled", mPosTrackingEnabled, mPosTrackingEnabled);
  RCLCPP_INFO_STREAM(mNode->get_logger(), " * Positional tracking enabled: " << (mPosTrackingEnabled ? "TRUE" : "FALSE"));

  getParam("pos_tracking.base_frame", mBaseFrameId, mBaseFrameId, " * Base frame id: ");
  getParam("pos_tracking.map_frame", mMapFrameId, mMapFrameId, " * Map frame id: ");
  getParam("pos_tracking.odometry_frame", mOdomFrameId, mOdomFrameId, " * Odometry frame id: ");

  getParam("pos_tracking.publish_tf", mPublishTF, mPublishTF);
  RCLCPP_INFO_STREAM(mNode->get_logger(), " * Broadcast Odometry TF: " << (mPublishTF ? "TRUE" : "FALSE"));
  if (mPublishTF)
  {
    getParam("pos_tracking.publish_map_tf", mPublishMapTF, mPublishMapTF);
    RCLCPP_INFO_STREAM(mNode->get_logger(), " * Broadcast Pose TF: " << (mPublishMapTF ? "TRUE" : "FALSE"));
    getParam("pos_tracking.publish_imu_tf", mPublishImuTF, mPublishImuTF);
    RCLCPP_INFO_STREAM(mNode->get_logger(),
                       " * Broadcast Static IMU TF [not for ZED]: " << (mPublishImuTF ? "TRUE" : "FALSE"));
  }

  getParam("pos_tracking.path_pub_rate", mPathPubRate, mPathPubRate, " * [DYN] Path publishing rate: ");
  getParam("pos_tracking.path_max_count", mPathMaxCount, mPathMaxCount);
  if (mPathMaxCount < 2 && mPathMaxCount != -1)
  {
    mPathMaxCount = 2;
  }
  RCLCPP_INFO_STREAM(mNode->get_logger(), " * Path history lenght: " << mPathMaxCount);

  paramName = "pos_tracking.initial_base_pose";
  mNode->declare_parameter(paramName, rclcpp::ParameterValue(mInitialBasePose));
  if (!mNode->get_parameter(paramName, mInitialBasePose))
  {
    RCLCPP_WARN_STREAM(mNode->get_logger(),
                       "The parameter '" << paramName << "' is not available or is not valid, using the default value");
    mInitialBasePose = std::vector<double>(6, 0.0);
  }
  if (mInitialBasePose.size() < 6)
  {
    RCLCPP_WARN_STREAM(mNode->get_logger(), "The parameter '" << paramName << "' must be a vector of 6 values of double type");
    mInitialBasePose = std::vector<double>(6, 0.0);
  }
  RCLCPP_INFO(mNode->get_logger(), " * Initial pose: [%g,%g,%g,%g,%g,%g,]", mInitialBasePose[0], mInitialBasePose[1],
              mInitialBasePose[2], mInitialBasePose[3], mInitialBasePose[4], mInitialBasePose[5]);

  getParam("pos_tracking.area_memory", mAreaMemory, mAreaMemory);
  RCLCPP_INFO_STREAM(mNode->get_logger(), " * Area Memory: " << (mAreaMemory ? "TRUE" : "FALSE"));
  getParam("pos_tracking.area_memory_db_path", mAreaMemoryDbPath, mAreaMemoryDbPath, " * Area Memory DB: ");
  getParam("pos_tracking.imu_fusion", mImuFusion, mImuFusion);
  RCLCPP_INFO_STREAM(mNode->get_logger(), " * IMU Fusion [not for ZED]: " << (mImuFusion ? "TRUE" : "FALSE"));
  getParam("pos_tracking.floor_alignment", mFloorAlignment, mFloorAlignment);
  RCLCPP_INFO_STREAM(mNode->get_logger(), " * Floor Alignment: " << (mFloorAlignment ? "TRUE" : "FALSE"));
  getParam("pos_tracking.init_odom_with_first_valid_pose", mInitOdomWithPose, mInitOdomWithPose);
  RCLCPP_INFO_STREAM(mNode->get_logger(),
                     " * Init Odometry with first valid pose data: " << (mInitOdomWithPose ? "TRUE" : "FALSE"));
  getParam("pos_tracking.two_d_mode", mTwoDMode, mTwoDMode);
  RCLCPP_INFO_STREAM(mNode->get_logger(), " * 2D mode: " << (mTwoDMode ? "TRUE" : "FALSE"));
  if (mTwoDMode)
  {
    getParam("pos_tracking.fixed_z_value", mFixedZValue, mFixedZValue, " * Fixed Z value: ");
  }

  // ------------------------------------------

  paramName = "pos_tracking.qos_history";
  mNode->declare_parameter(paramName, rclcpp::ParameterValue(qos_hist));

  if (mNode->get_parameter(paramName, paramVal))
  {
    qos_hist = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
    mPoseQos.history(qos_hist);
  }
  else
  {
    RCLCPP_WARN(mNode->get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
  }

  RCLCPP_INFO(mNode->get_logger(), " * Pose/Odometry QoS History: %s", sl_tools::qos2str(qos_hist).c_str());

  // ------------------------------------------

  paramName = "pos_tracking.qos_depth";
  mNode->declare_parameter(paramName, rclcpp::ParameterValue(qos_depth));

  if (mNode->get_parameter(paramName, paramVal))
  {
    qos_depth = paramVal.as_int();
    mPoseQos.keep_last(qos_depth);
  }
  else
  {
    RCLCPP_WARN(mNode->get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
  }

  RCLCPP_INFO(mNode->get_logger(), " * Pose/Odometry QoS History depth: %d", qos_depth);

  // ------------------------------------------

  paramName = "pos_tracking.qos_reliability";
  mNode->declare_parameter(paramName, rclcpp::ParameterValue(qos_reliability));

  if (mNode->get_parameter(paramName, paramVal))
  {
    qos_reliability =
        paramVal.as_int() == 1 ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    mPoseQos.reliability(qos_reliability);
  }
  else
  {
    RCLCPP_WARN(mNode->get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
  }

  RCLCPP_INFO(mNode->get_logger(), " * Pose/Odometry QoS Reliability: %s", sl_tools::qos2str(qos_reliability).c_str());

  // ------------------------------------------

  paramName = "pos_tracking.qos_durability";
  mNode->declare_parameter(paramName, rclcpp::ParameterValue(qos_durability));

  if (mNode->get_parameter(paramName, paramVal))
  {
    qos_durability =
        paramVal.as_int() == 1 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL : RMW_QOS_POLICY_DURABILITY_VOLATILE;
    mPoseQos.durability(qos_durability);
  }
  else
  {
    RCLCPP_WARN(mNode->get_logger(), "The parameter '%s' is not available, using the default value", paramName.c_str());
  }

  RCLCPP_INFO(mNode->get_logger(), " * Pose/Odometry QoS Durability: %s", sl_tools::qos2str(qos_durability).c_str());
}

void ZedOdom::initTransforms()
{
  // According to REP 105 -> http://www.ros.org/reps/rep-0105.html

  // base_link <- odom <- map
  //     ^                 |
  //     |                 |
  //     -------------------

  // ----> Dynamic transforms
  mOdom2BaseTransf.setIdentity();  // broadcasted if `publish_tf` is true
  mMap2OdomTransf.setIdentity();   // broadcasted if `publish_map_tf` is true
  mMap2BaseTransf.setIdentity();   // used internally, but not broadcasted
                                   // <---- Dynamic transforms
}

bool ZedOdom::setPose(float xt, float yt, float zt, float rr, float pr, float yr)
{
  initTransforms();

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

  // Apply Base to sensor transform
  tf2::Transform initPose;
  tf2::Vector3 origin(xt, yt, zt);
  initPose.setOrigin(origin);
  tf2::Quaternion quat;
  quat.setRPY(rr, pr, yr);
  initPose.setRotation(quat);

  initPose = initPose * mSensor2BaseTransf.inverse();  // TODO Check this transformation. Rotation seems wrong

  // SL pose
  sl::float3 t_vec;
  t_vec[0] = initPose.getOrigin().x();
  t_vec[1] = initPose.getOrigin().y();
  t_vec[2] = initPose.getOrigin().z();

  sl::float4 q_vec;
  q_vec[0] = initPose.getRotation().x();
  q_vec[1] = initPose.getRotation().y();
  q_vec[2] = initPose.getRotation().z();
  q_vec[3] = initPose.getRotation().w();

  sl::Translation trasl(t_vec);
  sl::Orientation orient(q_vec);
  mInitialPoseSl.setTranslation(trasl);
  mInitialPoseSl.setOrientation(orient);

  return (mSensor2BaseTransfValid & mSensor2CameraTransfValid & mCamera2BaseTransfValid);
}

bool ZedOdom::startPosTracking()
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
  //mPosTrackingReady = false;

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

  bool mPoseSmoothing = false;  // Always false. Pose Smoothing is to be enabled only for VR/AR applications
  trackParams.enable_pose_smoothing = mPoseSmoothing;

  trackParams.enable_area_memory = mAreaMemory;
  trackParams.enable_imu_fusion = mImuFusion;
  trackParams.initial_world_transform = mInitialPoseSl;

  trackParams.set_floor_as_origin = mFloorAlignment;

  sl::ERROR_CODE err = mZed->enablePositionalTracking(trackParams);

  if (err == sl::ERROR_CODE::SUCCESS)
  {
    mPosTrackingStarted = true;
  }
  else
  {
    mPosTrackingStarted = false;

    RCLCPP_WARN(mNode->get_logger(), "Tracking not started: %s", sl::toString(err).c_str());
  }

  return mPosTrackingStarted;
}

bool ZedOdom::getCamera2BaseTransform()
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

bool ZedOdom::getSens2CameraTransform()
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

bool ZedOdom::getSens2BaseTransform()
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

void ZedOdom::processOdometry(std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPublisher)
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

    mPosTrackingStatus = mZed->getPosition(deltaOdom, sl::REFERENCE_FRAME::CAMERA);

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
      auto mFrameTimestamp = sl_tools::slTime2Ros(mZed->getTimestamp(sl::TIME_REFERENCE::CURRENT));
      publishOdom(mOdom2BaseTransf, deltaOdom, mFrameTimestamp, odomPublisher);
      //mPosTrackingReady = true;
    }
  }
  else if (mFloorAlignment)
  {
    RCLCPP_DEBUG_THROTTLE(mNode->get_logger(), steady_clock, 5.0,
                          "Odometry will be published as soon as the floor as been detected for the first time");
  }
}

void ZedOdom::publishOdom(tf2::Transform& odom2baseTransf, sl::Pose& slPose, rclcpp::Time t, std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPublisher)
{
  //adapted from https://github.com/stereolabs/zed-ros2-wrapper/blob/de1dfc4cde9ada3173074a05e14631d4d56e442c/zed_components/src/zed_camera/src/zed_camera_component.cpp
  
  auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();

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
  odomPublisher->publish(std::move(odomMsg));
}
