/******************************************************************************************************************
 ** This sample demonstrates how to use two zeds_ with the ZED SDK, each grab are in a separate thread             **
 ** This sample has been tested with 3 zeds_ in HD720@30fps resolution. Linux only.                                **
 *******************************************************************************************************************/

#include <sl/Camera.hpp>
#include "zed_components/zed_camera_component.hpp"
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/publisher.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/distortion_models.hpp>


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

    bool run_;
    const int res_w_;
    const int res_h_;
    const float downsampling_;
    const int fps_;
    std::uint32_t sn_l_;
    std::uint32_t sn_r_;
    std::unordered_map<std::uint32_t, std::string> camera_frames_;
    std::unordered_map<std::uint32_t, std::array<std::shared_ptr<image_transport::CameraPublisher>, 2>> camera_publishers_;
	size_t nb_detected_zed_;
	std::vector<std::shared_ptr<sl::Camera>> zeds_;
	std::vector<std::thread> thread_pool_;
};


ZedPublisher::ZedPublisher() : Node{"zed_rgbd_publisher"}, run_{true}, res_w_{1280}, res_h_{720}, downsampling_{1.0}, fps_{15},
	  	sn_l_{28846348}, sn_r_{21128703}, nb_detected_zed_{0}
{
  camera_frames_ = {{sn_l_, "zed2_l_left_camera_optical_frame"},  {sn_r_, "zed2_r_left_camera_optical_frame"}};
  rclcpp::QoS qos_profile(10);
  auto rgb_publisher_l_ = std::make_shared<image_transport::CameraPublisher>(this, "zed2_l/rgb", qos_profile.get_rmw_qos_profile());
  auto depth_publisher_l_ = std::make_shared<image_transport::CameraPublisher>(this, "zed2_l/depth", qos_profile.get_rmw_qos_profile());
  auto rgb_publisher_r_ = std::make_shared<image_transport::CameraPublisher>(this, "zed2_r/rgb", qos_profile.get_rmw_qos_profile());
  auto depth_publisher_r_ = std::make_shared<image_transport::CameraPublisher>(this, "zed2_r/depth", qos_profile.get_rmw_qos_profile());
  camera_publishers_ = {{sn_l_, {rgb_publisher_l_, depth_publisher_l_}}, {sn_r_, {rgb_publisher_r_, depth_publisher_r_}}};
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
      }
	}
};


void ZedPublisher::get_and_publish(sl::Camera& zed, bool& run) {
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
      camera_publishers_.at(sn).at(0)->publish(image, left_cam_info_msg);
      camera_publishers_.at(sn).at(1)->publish(depth_img, left_cam_info_msg);
    }
    sl::sleep_ms(2);
  }
  zed.close();
};


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
