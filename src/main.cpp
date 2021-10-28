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
 ** This sample demonstrates how to use two ZEDs with the ZED SDK, each grab are in a separate thread             **
 ** This sample has been tested with 3 ZEDs in HD720@30fps resolution. Linux only.                                **
 *******************************************************************************************************************/

#include <sl/Camera.hpp>
#include <zed_components/sl_tools.h>
#include <zed_components/zed_camera/zed_camera_component.hpp>
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
	ZedPublisher()
    : Node("zed_rgbd_publisher"), count_(0), run_{true}, res_w_{1280}, res_h_{720}, downsampling_{1.0}, fps_{30},
	  sn_l{"some"}, sn_r{"some_other"}, camera_frames_{{sn_l_, "zed2_l_left_camera_optical_frame"},
		  {sn_r_, "zed2_r_left_camera_optical_frame"}},
	  }
    {
		rgb_publisher_l_ = std::make_shared<image_transport::create_camera_publisher>(this, "zed2_l/rgb", 10);
		depth_publisher_l_ = std::make_shared<image_transport::create_camera_publisher>(this, "zed2_l/depth", 10);
		rgb_publisher_r_ = std::make_shared<image_transport::create_camera_publisher>(this, "zed2_r/rgb", 10);
		depth_publisher_r_ = std::make_shared<image_transport::create_camera_publisher>(this, "zed2_r/depth", 10);
		camera_publishers_{{sn_l_, {rgb_publisher_l_, depth_publisher_l_}}, {sn_r_, {rgb_publisher_r_, depth_publisher_r_}};
		main_loop();
    };

	~ZedPublisher(){
		// stop all running threads
		run_ = false;

		// Wait for every thread to be stopped
		for (int z = 0; z < nb_detected_zed; z++)
		     if (zeds[z].isOpened())
		         thread_pool[z].join();
	};

  private:
    int main_loop();
    void zed_acquisition(Camera& zed, cv::Mat& image_low_res, bool& run, Timestamp& ts);
    bool run_
	const int res_w_;
    const int res_h_;
    const float downsampling_;
    const int fps_;
    std::string sn_l_;
    std::string sn_r_;
    std::shared_ptr<image_transport::CameraPublisher> rgb_publisher1_;
    std::shared_ptr<image_transport::CameraPublisher> depth_publisher1_;
    std::shared_ptr<image_transport::CameraPublisher> rgb_publisher2_;
    std::shared_ptr<image_transport::CameraPublisher> depth_publisher2_;
    size_t count_;
    std::unordered_map<std::string, std::string> camera_frames_;
    std::unordered_map<std::string, std::array<std::shared_ptr<image_transport::CameraPublisher>>, 2> camera_publishers_;
};


int ZedPublisher::main_loop() {
    
	InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_paramters.camera_fps = fps_;
    
	std::vector< sl::DeviceProperties> devList = sl::Camera::getDeviceList();
    int nb_detected_zed = devList.size();

	for (int z = 0; z < nb_detected_zed; z++) {
		std::cout << "ID : " << devList[z].id << " ,model : " << devList[z].camera_model << " , S/N : " << devList[z].serial_number << " , state : "<<devList[z].camera_state<<std::endl;
	}
	
    if (nb_detected_zed != 2) {
    	std::cout << "Did not detect two ZED cameras, exit program" << std::endl;
        return sl::EXIT_FAILURE;
    }
    
    std::cout << nb_detected_zed << " ZED Detected" << std::endl;

    std::vector<Camera> zeds(nb_detected_zed);
    // try to open every detected cameras
    for (int z = 0; z < nb_detected_zed; z++) {
        init_parameters.input.setFromCameraID(z);
        sl::ERROR_CODE err = zeds[z].open(init_parameters);
        if (err == ERROR_CODE::SUCCESS) {
            auto cam_info = zeds[z].getCameraInformation();
            std::cout << cam_info.camera_model << ", ID: " << z << ", SN: " << cam_info.serial_number << " Opened" << std::endl;
        } else {
        	std::cout << "ZED ID:" << z << " Error: " << err << std::endl;
            zeds[z].close();
        }
    }
    
    // Create a grab thread for each opened camera
    std::vector<thread> thread_pool(nb_detected_zed); // compute threads
    std::vector<cv::Mat> images_lr(nb_detected_zed); // display images
    std::vector<string> wnd_names(nb_detected_zed); // display windows names
    std::vector<Timestamp> images_ts(nb_detected_zed); // images timestamps

    for (int z = 0; z < nb_detected_zed; z++)
        if (zeds[z].isOpened()) {
            // camera acquisition thread
            thread_pool[z] = std::thread(ZedPublisher::get_and_publish, ref(zeds[z]), ref(run_));
        }
}


void ZedPublisher::get_and_publish(Camera& zed, bool& run) {
    sl::Mat mat_depth;
    sl::Mat mat_image;
    left_cam_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    // used just for the method call
    right_cam_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    auto sn = zeds.getCameraInformation().serial_number;
    Resolution res(res_w_ * downsampling_, res_h_ * downsampling_);
    ZedCamera::fillCamInfo(zed, left_cam_info_msg, right_cam_info_msg, camera_frames_[sn], "no_right_frame_id");
    while (run) {
        // grab current images and compute depth
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(mat_image, VIEW::LEFT, MEM::CPU, res);
            zed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, res)
            ts = zed.getTimestamp(TIME_REFERENCE::IMAGE);
            auto image = sl_tools::imageToROSmsg(mat_image, camera_frames_[sn], ts);
            left_cam_info_msg->header.stamp = ts;
            auto depth_img = sl_tools::imageToROSmsg(depth, camera_frames_[sn], ts);
            camera_publishers_[sn][0]->publish(image, left_cam_info_msg);
            camera_publishers_[sn][1]->publish(depth_img, left_cam_info_msg);
        }
        sleep_ms(2);
    }
    zed.close();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedPublisher>());
  rclcpp::shutdown();
  return 0;
}
