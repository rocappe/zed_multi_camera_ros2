> Project: "ZED2 multi-camera node"

> Owner: "Roberto Cappellaro" 

> Date: "2021:11" 

---

# ZED2 multi-camera node

## Description of the project
It is a Ros2 foxy node based on https://github.com/stereolabs/zed-multi-camera and snippets of the zed ros2 wrapper https://github.com/stereolabs/zed-ros2-wrapper. It is written to be used with two ZED2 cameras and it uses the config files present in the zed ros2 wrapper.  
There is also a ros2 launchfile *zed2_multi.launch.py* to launch two cameras using the zed ros2 node made by the manufacturer. The configuration parameters are in zed_l.launch.py and zed_r.launch.py.

## Installation procedure
Clone it in the colcon workspace and build it. 

## Limitations

- USB bandwidth: The ZED  in 1080p30 mode generates around 250MB/s of image data. USB 3.0 maximum bandwidth is around 400MB/s, so the number of cameras, resolutions and framerates you can use on a single machine will be limited by the USB 3.0 controller on the motherboard. When bandwidth limit is exceeded, corrupted frames (green or purple frames, tearing) can appear.
- Using a single USB 3.0 controller, here are the tested configurations: 2 ZEDs in HD1080 @ 15fps and HD720 @ 30fps.
- You can also use multiple GPUs to load-balance computations (use `param.device` to select a GPU for a ZED) and improve performance.
