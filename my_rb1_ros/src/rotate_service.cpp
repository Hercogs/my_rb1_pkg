#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/duration.h"
#include "ros/service_server.h"
#include <ros/ros.h>
#include <tf/tf.h>

/* Global variables */

ros::Subscriber sub_odom;
ros::Publisher pub_cmd_vel;

// Odom message
nav_msgs::Odometry odom_msg;
geometry_msgs::Twist cmd_msg;

// Global z pose
int actual_pose_z;


int rad_to_degree(double rad) {
  /* TODO*/
  return (int)(rad * 180 / 3.14159);
}


/*Odometry callback */
void odom_clb(const nav_msgs::Odometry::ConstPtr &msg) {
  // ROS_INFO("Odometry recieved");
  odom_msg = *msg;

  /* Get current orientation about z- axis */
  float z;
  tf::Quaternion q(
      odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
      odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  /* Convert to Euler */
  actual_pose_z = rad_to_degree(yaw);
}


/* Service callback */
bool service_clb(my_rb1_ros::Rotate::Request &req,
                 my_rb1_ros::Rotate::Response &res) {

  ROS_INFO("Service was called");

  int setpoint_pose;

  /* Get current orientation about z- axis */

  ROS_INFO("Current orientation: %d", actual_pose_z);
  ROS_INFO("Turn robot by %d degrees", req.degrees);

  setpoint_pose = actual_pose_z + req.degrees;

  if (setpoint_pose > 360 || setpoint_pose < -360) {
    res.result = "Fail";
    return true;
  }

  if (setpoint_pose > actual_pose_z) {
    ROS_INFO("Setpint is larger than current pos");

    while (actual_pose_z < setpoint_pose) {

      cmd_msg.angular.z = 0.1;
      pub_cmd_vel.publish(cmd_msg);
      ros::Duration(0.2).sleep();

      // ROS_INFO("Current orientation: %d", actual_pose);
    }

  } else if (setpoint_pose < actual_pose_z) {
    ROS_INFO("Setpint is smaller than current pos");

    while (actual_pose_z > setpoint_pose) {

      cmd_msg.angular.z = -0.1;
      pub_cmd_vel.publish(cmd_msg);
      ros::Duration(0.2).sleep();
    }

  } else {
    // Setpoint == actula
    ROS_INFO("Setpint is equal to current pos");
  }

  ROS_INFO("Service call finished, current pose: %d", actual_pose_z);

  cmd_msg.angular.z = 0.0;
  pub_cmd_vel.publish(cmd_msg);

  res.result = "Success";
  return true;
}

int main(int argc, char **argv) {

  ROS_INFO("Hello from this node");

  ros::init(argc, argv, "rotate_service_server_node");

  ros::NodeHandle nh;

  sub_odom = nh.subscribe("/odom", 5, odom_clb);
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ros::ServiceServer rot_service;
  rot_service = nh.advertiseService("/rotate_robot", service_clb);

  /* Multithread, to allow service server acces odometry callback */
  ros::MultiThreadedSpinner spinner(2); // 2 threads

  while (ros::ok()) {
    spinner.spin();
  }

  return 0;
}