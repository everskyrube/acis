# ACIS
## A Real-time Autonomous Crack Inspection System on UAV

A crack inspection system on UAV accurately computes the cracks' positions only relied on a RGB-D camera. 

<p align = "center">


</p>

### [Video](https://www.youtube.com/watch?v=4HIoySNRzHI)
[![ACIS_DEMO](https://img.youtube.com/vi/4HIoySNRzHI/0.jpg)](https://www.youtube.com/watch?v=4HIoySNRzHI)

### Publications
Pending to submit

### Requirements
* **Ubuntu 16.04 or 18.04**
* **ROS Kinetic or Melodic:** [ROS Install](http://wiki.ros.org/ROS/Installation)
* **OpenCV >= 4.4:** [OpenCV 4.4 Linux Install](https://docs.opencv.org/4.4.0/d7/d9f/tutorial_linux_install.html)
* **To use OpenCV dnn module supports YOLOv4-SE, it requires OpenCV >= 4.7:** [OpenCV 4.7 Linux Install](https://docs.opencv.org/4.7.0/d7/d9f/tutorial_linux_install.html)
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
1. Download the config, weight and obj.names files from [Here](https://connectpolyu-my.sharepoint.com/:f:/g/personal/19111075r_connect_polyu_hk/Ekw5lvWiybhGts8BI-H3C7oB7zRT8viEhOOmU7kE7Qd5zA?e=fm0TES)
2. Change the path of these lines:   
- [cfg_path](https://github.com/everskyrube/acis/blob/ad52c4416c2625b7a6ea87a48a59527f930e4a04/ACIS/src/camera.cpp#L48) 
- [weight_path](https://github.com/everskyrube/acis/blob/ad52c4416c2625b7a6ea87a48a59527f930e4a04/ACIS/src/camera.cpp#L49)
- [classid_path](https://github.com/everskyrube/acis/blob/ad52c4416c2625b7a6ea87a48a59527f930e4a04/ACIS/src/camera.cpp#L50)  
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
4. To visualize 2-D bounding boxes, uncomment the line [drawBoundingBox](https://github.com/everskyrube/acis/blob/ad52c4416c2625b7a6ea87a48a59527f930e4a04/ACIS/src/camera.cpp#L266)
```c++
yolo.drawBoundingBox(image_rgb);
```
5. To visualize the estimate of object position in inertial frame, launch `rviz` node:
```
cd ~/catkin_ws
roslaunch ACIS rviz.launch
```
6. To improve the detection speed or accuracy, change the default input size [yoloNet](https://github.com/everskyrube/acis/blob/ad52c4416c2625b7a6ea87a48a59527f930e4a04/ACIS/src/camera.cpp#L56) 
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

