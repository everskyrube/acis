/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "movement/generalmove.h"
#include "utils/yamlRead.h"
#include "utils/kinetic_math.h"
#include <algorithm>

using namespace std;

// subscribe FCU state through State msg to check arm and offboard
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

enum stage
{
    IDLE,
    TAKEOFF,
    HOVER,
    INSPECTION,
    RETURN,
    LANDING,
    END
} static mission_stage = IDLE;

// static bool force_start;
static string config_path;
static double x_max, y_max, x_min, y_min; // geofence

int main(int argc, char **argv)
{

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    // ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Publisher approach_pub = nh.advertise<visualization_msgs::Marker>("/approach_marker", 10);
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    string config_path = "/home/sky/catkin_ws/src/ACIS/config/basic_config.yaml";

    // ros::NodeHandle nh("~"); to use nh.getParam

    if (nh.getParam("basic_config", config_path))
    {
        // cout<<"config path: "<< config_file_path<<endl;
        ROS_INFO_STREAM("Config Folder: " << config_path);
    }
    else
    {
        ROS_ERROR("Cannot load config file !");
        // cout<<"cannot load basic_config "<<endl;
    }

    /*
    if(nh.getParam("force_start", force_start)){
        ROS_INFO_STREAM("force_start: " << force_start);
    } else {
        ROS_ERROR("Cannot get force_start");
    }
    */

    x_max = getDoubleVariableFromYamlhierarchy(config_path, "FJ005_geo_mode", "x_max");
    x_min = getDoubleVariableFromYamlhierarchy(config_path, "FJ005_geo_mode", "x_min");
    y_max = getDoubleVariableFromYamlhierarchy(config_path, "FJ005_geo_mode", "y_max");
    y_min = getDoubleVariableFromYamlhierarchy(config_path, "FJ005_geo_mode", "y_min");
    cout << "FJ005_geofence: " << endl;
    cout << "x_max: " << x_max << endl;
    cout << "x_min: " << x_min << endl;
    cout << "y_max: " << y_max << endl;
    cout << "y_min: " << y_min << endl;

    // wait for FCU connection
    // ros::NodeHandle nh; //To connect current_state

    while (ros::ok() && !current_state.connected)
    {
        ROS_INFO("waiting for connection");
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    pose.pose.orientation.w = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;

    // send a few setpoints before entering offboard mode, otherwise, will fail to switch mode
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0.5;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;

    ROS_INFO("Change time to now");
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {

        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    mission_stage = TAKEOFF;
                    cout << "mission stage : TAKEOFF 1---------------------- " << endl;
                }
                last_request = ros::Time::now();
            }
        }

        cout << mission_stage << endl;

        if (mission_stage == TAKEOFF)
        {
            mission_stage = HOVER;
            ROS_INFO("Next Stage: LANDING");
            last_request = ros::Time::now();
        }

        if (mission_stage == HOVER)
        {
            if (ros::Time::now() - last_request > ros::Duration(6.0))
            {
                mission_stage = LANDING;
                cout << "LANDING --------------------------" << endl;
                last_request = ros::Time::now();
            }
        }

        // local_pos_pub.publish(pose);

        if (mission_stage == LANDING)
        {
            // identify_flag.identify = true;
            double secs = (ros::Time::now() - last_request).toSec();
            cout << "secs: " << secs << endl;
            pose.pose.position.z = 1 - (secs)*0.1;
            cout << "z: " << pose.pose.position.z << endl;
            if (pose.pose.position.z < 0.5)
            {
                cout << "landing **************************** " << endl;
                // pose.pose.position.z = -0.3;
                // arm_cmd.request.value = false;
                if (land_client.call(land_cmd) &&
                    land_cmd.response.success)
                {
                    ROS_INFO("Vehicle landed****************************");
                    mission_stage = END;
                    return 0;
                }
            }
        }

        if (mission_stage == END)
        {
            cout << "enter END ****************************" << endl;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = -0.3;
            return 0;
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}