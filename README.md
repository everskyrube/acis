# ACIS
## A Real-time Autonomous Crack Inspection System on UAV
#Add abstract here

<p align = "center">


</p>

### Video

[[ACIS Demo]]([https://www.youtube.com/watch?v=4HIoySNRzHI])

### Publications
Pending to submit

### Requirements
* **Ubuntu 16.04 or 18.04**
* **ROS Kinetic or Melodic:** [ROS Install](http://wiki.ros.org/ROS/Installation)
* **OpenCV >= 4.4:** [OpenCV Linux Install](https://docs.opencv.org/4.4.0/d7/d9f/tutorial_linux_install.html)
* **Python 3.8** 
* **CUDA >= 10.0:** [CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) 
* **CUDNN >= 7.0:** [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive)

### Build instructions on Linux
1. clone repository into working space

```
cd ~/catkin_ws/src
git clone https://github.com/everskyrube/ACIS.git
```

2. Install 3rd Party library
``` 
cd ~/catkin_ws/src/ACIS/3rdPartLib/
./install3rdPartLib.sh
```
3. Compile 
```
cd ~/catkin_ws
catkin_make
```

### How to use (to detect cracks in our experiment)
1. Download [Yolov4.cfg](https://drive.google.com/file/), [Yolov4.weight](https://drive.google.com/file/) and [obj.names](https://drive.google.com/file/)
2. Change these lines :   
- [cfg_path]() 
- [weight_path]()
- [classid_path]()  
```c++
static string cfg_path
static string weight_path
static string classid_path
```  
3. Compile and launch `camera` node for crack detection:
```
cd ~/catkin_ws
rosrun ACIS camera
```
4. To visualize 2-D bounding boxes, uncomment the line [drawBoundingBox]()
```c++
yolo.drawBoundingBox(image_rgb);
```
5. To visualize the estimate of object position in inertial frame, launch `rviz` node:
```
cd ~/catkin_ws
roslaunch ACIS rviz.launch
```
6. To improve the detection speed or accuracy, change the default input size [yoloNet]() 
```c++
static yoloNet yolo = yoloNet(cfg_path, weight_path, classid_path, 608, 608, 0.5);
```

### How to use (to inspect objects in our experiment)
1. Connect the quadrotor with flight controller and launch mavros  
```
roslaunch mavros px4.launch 
```
2. Launch the D455i camera by [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
3. Launch the `camera` node and `fj005` node:
```
cd ~/catkin_ws
roslaunch ACIS fj005.launch
```
4. To visualize the detected object and flight trajectory in inertial frame, launch `rviz` node:
```
cd ~/catkin_ws
roslaunch ACIS rviz.launch
```
## Maintainer 
Kwai-wa Tse (Dept.AAE,PolyU): [kwai-wa.tse@connect.polyu.hk](kwai-wa.tse@connect.polyu.hk)

