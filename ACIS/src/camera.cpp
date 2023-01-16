#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "ACIS/identify_command.h"
#include "ACIS/object.h"
#include "ACIS/Objects.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "utils/yoloNet.h"
#include "utils/kinetic_math.h"
#include "utils/yamlRead.h"
#include <numeric>
#include "visualization/rviz_object.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

static double x, y, z; // camrea_coord
static double fx, fy, cx, cy;
static double uav_x, uav_y, uav_z; // uav local parameter
static double uav_qx, uav_qy, uav_qz, uav_qw;
static double uav_size = 0.5;

static geometry_msgs::PoseStamped obj_pose;
static ACIS::Objects Object_array;

#define radius 1
// YOLO file path

static string cfg_path = "/home/sky/Yolo_crack_files/yolov4_crack.cfg";
static string weight_path = "/home/sky/Yolo_crack_files/yolov4_best_1005.weights";
static string classid_path = "/home/sky/Yolo_crack_files/obj.names";
static string object_cfg_path = "/home/sky/catkin_ws/src/ACIS/config/crack_config.yaml";
static string inspection_result_path = "/home/sky/catkin_ws/src/ACIS/result/result.yaml";
static std::vector<string> object_class = {"crack"};

// Creat the YOLO network
static yoloNet yolo = yoloNet(cfg_path, weight_path, classid_path, 608, 608, 0.5);
static bool detected_flag;
static bool obstacle = 0;
static bool identify_flag = true;

// IIR - infinite impulse response filter parameter
static Mat3x3 IIR_worldframe;
static visualization_msgs::Marker point_list;

void camera_info_cb(const sensor_msgs::CameraInfoPtr &msg)
{
  fx = msg->K[0];
  fy = msg->K[4];
  cx = msg->K[2];
  cy = msg->K[5];
}

void identify_cb(const ACIS::identify_commandConstPtr &msg)
{
  identify_flag = msg->identify;
}

void uav_lp_cb(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
  uav_x = pose->pose.position.x;
  uav_y = pose->pose.position.y;
  uav_z = pose->pose.position.z;
  uav_qx = pose->pose.orientation.x;
  uav_qy = pose->pose.orientation.y;
  uav_qz = pose->pose.orientation.z;
  uav_qw = pose->pose.orientation.w;
}

bool CheckOccupancy(YAML::Node config, string &name)
{

  bool occupied = false;
  float dist_thres = 0.15;
  auto ins_x = config[name]["X_w"].as<float>();
  auto ins_y = config[name]["Y_w"].as<float>();
  auto ins_z = config[name]["Z_w"].as<float>();
  YAML::Node result = YAML::LoadFile(inspection_result_path);
  int dist_cnt = 0;
  for (auto it = result.begin(); it != result.end(); ++it)
  {
    auto element = *it;
    auto key = it->first;
    auto value = it->second;
    auto X_w = value["X_w"].as<float>();
    auto Y_w = value["Y_w"].as<float>();
    auto Z_w = value["Z_w"].as<float>();
    dist_cnt++;
    auto dist = sqrt(pow(ins_x - X_w, 2) + pow(ins_y - Y_w, 2) + pow(ins_z - Z_w, 2));

    if (dist < dist_thres)
    {
      occupied = true;
      break;
    }
  }
  return occupied;
}

inline void depth_calculate(const yoloObject_t &object, int object_order, cv::Mat frame, string &name)
{

  Quaterniond q(uav_qw, uav_qx, uav_qy, uav_qz);
  Mat3x3 mat = Q2R(q);

  YAML::Node config = YAML::LoadFile(object_cfg_path);
  auto obj_size = config[name]["size"].as<double>();
  vector<double> sum_env_dep, sum_obj_dep; // vector to store environment depth and object depth
  vector<double> sum_u, sum_v, sum_w;      // vector to store world frame coordinate
  vector<double> sum_i, sum_j, sum_k;

  for (auto j : object.env_point)
  {
    double env_dep = 0.001 * frame.at<ushort>(j); // ushort holds unsigned 16-bit data
    if (env_dep > 0.5 && env_dep < 10.0)
    {
      sum_env_dep.push_back(env_dep); // get each depth of env_point
    }
  }

  for (auto j : object.depth_point)
  {

    double sur_dep = 0.001 * frame.at<ushort>(j);
    auto obj_dep = sur_dep;

    if (sur_dep > 0.5 && sur_dep < 8.0)
    {
      sum_obj_dep.push_back(obj_dep);
    }

    z = obj_dep;
    x = z * (j.x - cx) / fx; // pixel coordinate u,v -> camera coordinate x,y
    y = z * (j.y - cy) / fy;

    Vec4 camera_coord(x, y, z, 1);
    Vec4 body_coord, world_coord;
    Vec4 offset(0, 0, 0, 0);

    Mat4x4 matrix_camera_to_body;
    Mat4x4 matrix_body_to_world;

    matrix_camera_to_body << 0, 0, 1, 0.13,
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 0, 1;

    matrix_body_to_world << mat(0, 0), mat(0, 1), mat(0, 2), uav_x,
        mat(1, 0), mat(1, 1), mat(1, 2), uav_y,
        mat(2, 0), mat(2, 1), mat(2, 2), uav_z,
        0, 0, 0, 1;

    body_coord = matrix_camera_to_body * camera_coord + offset;
    world_coord = matrix_body_to_world * body_coord;

    // world_coord = camera_coord;

    sum_i.push_back(body_coord(0));
    sum_j.push_back(body_coord(1));
    sum_k.push_back(body_coord(2));
    sum_u.push_back(world_coord(0));
    sum_v.push_back(world_coord(1));
    sum_w.push_back(world_coord(2));

    if (obj_dep > 0.4 && obj_dep < 1.8 && abs(x) < 0.6) // condition to record crack position
    {
      config[name]["X_w"] = world_coord(0);
      config[name]["Y_w"] = world_coord(1);
      config[name]["Z_w"] = world_coord(2);
      config[name]["depth"] = obj_dep;

      cout << "###############################" << endl;
      cout << "object_order: " << object_order << endl;
      cout << "world_coord_x: " << config[name]["X_w"] << endl;
      cout << "world_coord_y: " << config[name]["Y_w"] << endl;
      cout << "world_coord_z: " << config[name]["Z_w"] << endl;
      cout << "###############################" << endl;
      cout << "                                " << endl;

      sort(sum_env_dep.begin(), sum_env_dep.end());
      double avg_env_dep = average(sum_env_dep);
      double traversable_dep = avg_env_dep - obj_dep;

      obstacle = (traversable_dep > radius) ? false : true;
      config[name]["obstacle"] = obstacle;

      bool occupied = CheckOccupancy(config, name);
      if (!occupied)
      {
        ros::Time stamp = ros::Time::now();
        stringstream ss;
        ss << stamp.sec << "." << stamp.nsec;
        config[name]["detected_time"] = ss.str();
        ofstream file_out;
        file_out.open(inspection_result_path, std::ios_base::app);
        file_out << config << endl;
        file_out.close();
      }
    }
  }

  IIRfilter(IIR_worldframe(0, getIndex(object_class, name)), average(sum_u));
  IIRfilter(IIR_worldframe(1, getIndex(object_class, name)), average(sum_v));
  IIRfilter(IIR_worldframe(2, getIndex(object_class, name)), average(sum_w));

  obj_pose.header.frame_id = "map";
  obj_pose.pose.position.x = IIR_worldframe(0, getIndex(object_class, name));
  obj_pose.pose.position.y = IIR_worldframe(1, getIndex(object_class, name));
  obj_pose.pose.position.z = IIR_worldframe(2, getIndex(object_class, name));
  obj_pose.pose.orientation.w = 1.0;
  obj_pose.pose.orientation.x = 0.0;
  obj_pose.pose.orientation.y = 0.0;
  obj_pose.pose.orientation.z = 0.0;

  Object_array.object_array.at(getIndex(object_class, name)).class_name = name;
  Object_array.object_array.at(getIndex(object_class, name)).X_w = IIR_worldframe(0, getIndex(object_class, name));
  Object_array.object_array.at(getIndex(object_class, name)).Y_w = IIR_worldframe(1, getIndex(object_class, name));
  Object_array.object_array.at(getIndex(object_class, name)).Z_w = IIR_worldframe(2, getIndex(object_class, name));
  Object_array.object_array.at(getIndex(object_class, name)).has_obstacle = obstacle;
}

void callback(const sensor_msgs::CompressedImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
{

  cv::Mat image_rgb;

  try
  {
    image_rgb = cv::imdecode(cv::Mat(rgb->data), 1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
  cv::Mat image_dep = depth_ptr->image; // Convert a sensor_msgs to an OpenCV-compatible CvImage

  if (identify_flag == true)
  {
    yolo.runOnFrame(image_rgb); // run the network
    detected_flag = yolo.check_detected();
    yolo.init_box();
    yolo.init_points();
    // yolo.display_points();
    yolo.drawBoudingBox(image_rgb);

    if (detected_flag)
    {
      int obj_cnt = 0;
      cout << "objects.size : " << yolo.objects.size() << endl;
      for (auto &object_i : yolo.objects)
      {
        obj_cnt++;
        if (object_i.confidence > 0.8)
        {
          depth_calculate(object_i, obj_cnt, image_dep, object_class[0]);
        }
      }
    }
    else
    {
      cout << "No object detected" << endl;
    }
  }
  else
  {
    cout << "identify not begin" << endl;
  }
}

void vicon_pose_recall(const geometry_msgs::PoseStamped::ConstPtr &pose1, const geometry_msgs::PoseStamped::ConstPtr &pose2)
{

  point_list.points.clear();
  // point_list.header.frame_id = "world";
  point_list.header.frame_id = "map";
  point_list.header.stamp = ros::Time::now();
  point_list.ns = "GT_points";
  point_list.id = 0;
  point_list.action = visualization_msgs::Marker::ADD;
  point_list.pose.orientation.w = 1.0;
  point_list.type = visualization_msgs::Marker::SPHERE_LIST;
  point_list.scale.x = point_list.scale.y = point_list.scale.z = 0.2;
  //  std_msgs::ColorRGBA color;
  //  color.r= 0.8;
  //  color.g= 0.8;
  //  color.b= 0.8;
  //  color.a= 1;
  point_list.color.a = 1;
  point_list.color.r = 1;
  point_list.color.g = 1;
  point_list.color.b = 1;

  geometry_msgs::Point obj1, obj2, obj3;

  obj1.x = pose1->pose.position.x;
  obj1.y = pose1->pose.position.y;
  obj1.z = pose1->pose.position.z;
  obj2.x = pose2->pose.position.x;
  obj2.y = pose2->pose.position.y;
  obj2.z = pose2->pose.position.z;
  point_list.points.push_back(obj1);
  point_list.points.push_back(obj2);

  // point_list.colors.push_back(color);
}

int main(int argc, char **argv)
{

  cout << ">>>> camera node on" << endl;
  ros::init(argc, argv, "camera");
  ros::NodeHandle nh;

  ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, camera_info_cb);
  ros::Subscriber uavposlp_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, uav_lp_cb); // mavros/vision_pose/pose
  ros::Publisher objects_pub = nh.advertise<ACIS::Objects>("/Objects_array", 1);
  ros::Subscriber identify_sub = nh.subscribe<ACIS::identify_command>("/identify", 1, identify_cb);
  ros::Publisher object_pub = nh.advertise<geometry_msgs::PoseStamped>("objectpose", 1);
  ros::Publisher groundtruth_pub = nh.advertise<visualization_msgs::Marker>("gt_points", 10);

  message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
  message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);

  message_filters::Subscriber<geometry_msgs::PoseStamped> gt1_sub(nh, "/vrpn_client_node/traffic_light/pose", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> gt2_sub(nh, "/vrpn_client_node/bulb/pose", 1);

  typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
  typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy2;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
  Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(10), gt1_sub, gt2_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));
  sync2.registerCallback(boost::bind(&vicon_pose_recall, _1, _2));
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    objects_pub.publish(Object_array);
    object_pub.publish(obj_pose);
    groundtruth_pub.publish(point_list);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
