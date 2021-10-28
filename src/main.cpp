///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


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

 // Using std and sl namespaces
using namespace std;
using namespace sl;

class ZedPublisher : public rclcpp::Node
{
  public:
	ZedPublisher();
	~ZedPublisher();

  private:
    int main_loop();
    void get_and_publish(Camera& zed, bool& run);
    bool run_;
    const int res_w_;
    const int res_h_;
    const float downsampling_;
    const int fps_;
    std::uint8_t sn_l_;
    std::uint8_t sn_r_;
    std::unordered_map<std::uint8_t, std::string> camera_frames_;
    std::unordered_map<std::uint8_t, std::array<image_transport::CameraPublisher *, 2>> camera_publishers_;
	int nb_detected_zed_;
	std::vector<Camera> zeds_;
	std::vector<thread> thread_pool_;
};


ZedPublisher::ZedPublisher() : Node{"zed_rgbd_publisher"}, run_{true}, res_w_{1280}, res_h_{720}, downsampling_{1.0}, fps_{30},
	  	sn_l_{1}, sn_r_{2}, nb_detected_zed_{0}
    {
		camera_frames_ = {{sn_l_, "zed2_l_left_camera_optical_frame"},  {sn_r_, "zed2_r_left_camera_optical_frame"}};
		rclcpp::QoS qos_profile(10);
		auto rgb_publisher_l_ = image_transport::create_camera_publisher(this, "zed2_l/rgb", qos_profile.get_rmw_qos_profile());
		auto depth_publisher_l_ = image_transport::create_camera_publisher(this, "zed2_l/depth", qos_profile.get_rmw_qos_profile());
		auto rgb_publisher_r_ = image_transport::create_camera_publisher(this, "zed2_r/rgb", qos_profile.get_rmw_qos_profile());
		auto depth_publisher_r_ = image_transport::create_camera_publisher(this, "zed2_r/depth", qos_profile.get_rmw_qos_profile());
		camera_publishers_ = {{sn_l_, {&rgb_publisher_l_, &depth_publisher_l_}}, {sn_r_, {&rgb_publisher_r_, &depth_publisher_r_}}};
		main_loop();
    };


ZedPublisher::~ZedPublisher(){
		// stop all running threads
		run_ = false;

		// Wait for every thread to be stopped
		for (int z = 0; z < nb_detected_zed_; z++)
		     if (zeds_[z].isOpened())
		         thread_pool_[z].join();
};

int ZedPublisher::main_loop() {
    
    InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.camera_fps = fps_;
    
	std::vector< sl::DeviceProperties> devList = sl::Camera::getDeviceList();
    nb_detected_zed_ = devList.size();

	for (int z = 0; z < nb_detected_zed_; z++) {
		std::cout << "ID : " << devList[z].id << " ,model : " << devList[z].camera_model << " , S/N : " << devList[z].serial_number << " , state : "<<devList[z].camera_state<<std::endl;
	}
	
    if (nb_detected_zed_ != 2) {
    	std::cout << "Did not detect two ZED cameras, exit program" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::cout << nb_detected_zed_ << " ZED Detected" << std::endl;
	std::vector<Camera> vctr(nb_detected_zed_);
    zeds_ = vctr;
    // try to open every detected cameras
    for (int z = 0; z < nb_detected_zed_; z++) {
        init_parameters.input.setFromCameraID(z);
        ERROR_CODE err = zeds_[z].open(init_parameters);
        if (err == ERROR_CODE::SUCCESS) {
            auto cam_info = zeds_[z].getCameraInformation();
            std::cout << cam_info.camera_model << ", ID: " << z << ", SN: " << cam_info.serial_number << " Opened" << std::endl;
        } else {
        	std::cout << "ZED ID:" << z << " Error: " << err << std::endl;
            zeds_[z].close();
        }
    }
    
    // Create a grab thread for each opened camera
	std::vector<thread> vctr1(nb_detected_zed_);
    thread_pool_ = vctr1; // compute threads

    for (int z = 0; z < nb_detected_zed_; z++)
        if (zeds_[z].isOpened()) {
            // camera acquisition thread
            thread_pool_[z] = std::thread(&ZedPublisher::get_and_publish, this, ref(zeds_[z]), ref(run_));
        }
};


void ZedPublisher::get_and_publish(Camera& zed, bool& run) {
    sl::Mat mat_depth;
    sl::Mat mat_image;
    auto left_cam_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    auto right_cam_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    auto sn = zed.getCameraInformation().serial_number;
    Resolution res(res_w_ * downsampling_, res_h_ * downsampling_);
    stereolabs::ZedCamera::fillCamInfo(zed, left_cam_info_msg, right_cam_info_msg, camera_frames_[sn], "no_right_frame_id");
    while (run) {
        // grab current images and compute depth
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(mat_image, VIEW::LEFT, MEM::CPU, res);
            zed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, res);
            auto ts = sl_tools::slTime2Ros(zed.getTimestamp(TIME_REFERENCE::IMAGE));
            auto image = sl_tools::imageToROSmsg(mat_image, camera_frames_[sn], ts);
            left_cam_info_msg->header.stamp = ts;
            auto depth_img = sl_tools::imageToROSmsg(mat_depth, camera_frames_[sn], ts);
            (camera_publishers_.find(sn))[0]->publish(image, left_cam_info_msg);
            (camera_publishers_.find(sn))[1]->publish(depth_img, left_cam_info_msg);
        }
        sleep_ms(2);
    }
    zed.close();
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedPublisher>());
  rclcpp::shutdown();
  return 0;
}
