#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"
#include <ros/ros.h>
#include <tf/tf.h>

class MyService {
private:
  ros::Subscriber sub_odom;
  ros::Publisher pub_cmd_vel;

  ros::ServiceServer rot_service;

  // Odom message
  nav_msgs::Odometry odom_msg;
  geometry_msgs::Twist cmd_msg;

public:
  MyService(ros::NodeHandle nh) {

    ros::NodeHandle nh;

    this->pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    this->rot_service =
        nh.advertiseService("/rotate_robot", &MyService::service_clb, this);
    // this->sub_odom = nh.subscribe("/odom", 10, odom_clb);
  }

  bool service_clb(my_rb1_ros::Rotate::Request &req,
                   my_rb1_ros::Rotate::Response &res) {

    ROS_INFO("Service was called");
    this->sub_odom = nh.subscribe("/odom", 10, odom_clb);

    int actual_pose;
    int setpoint_pose;

    setpoint_pose = req.request;

    /* Get current orientation about z- axis */

    // ROS_INFO("Current orientation: %d", actual_pose_z);
    ROS_INFO("Setpoint orientation: %d", req.request);

    res.response = "Success";
    return true;
  }
};

int main(int argc, char **argv) {

  ROS_INFO("Hello from this node");
  ros::init(argc, argv, "rotate_service_server_node");

  ros::NodeHandle nh;

  MyService my_serv(nh);

  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}