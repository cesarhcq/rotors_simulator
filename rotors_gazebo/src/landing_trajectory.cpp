
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

float linx, liny, linz, angX, angY, angZ;
float eyawp = 0;
bool teste = true;

const float DEG_2_RAD = 3.14159265359 / 180.0;

void subscriber_Callback(const geometry_msgs::Twist& msg)
{
  //Using the callback function just for subscribing  
   //Subscribing the message and storing it in 'linx' and 'angZ'
  linx = msg.linear.x;
  liny = msg.linear.y;
  linz = msg.linear.z;

  angX = msg.angular.x;
  angY = msg.angular.y; 
  angZ = msg.angular.z;
}

float landing_pose(float erro_eyaw)
{
//-- rotation ori z
float k = 0.04;
float ki = 0.01;
float eyaw = erro_eyaw;
double desired_yaw = erro_eyaw;

  if (abs(eyaw)>1){
    desired_yaw = k*eyaw+(eyaw+eyawp)*ki;
    eyawp = eyaw;
    //ROS_INFO("eyaw = %lf",erro_eyaw);
    //print('rotação z')
    return desired_yaw;
  }
  else{
    return desired_yaw;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_trajectory");
  ros::NodeHandle nh;

  //-- Publusher
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  //-- Subscriber
  ros::Subscriber sub = nh.subscribe("/bebop2/camera_base/aruco_results", 10, subscriber_Callback);

  //-- Rate
  ros::Rate rate(1.0);

  //-- geometry mensages
  geometry_msgs::Twist msg;

  ROS_INFO("Started landing.");

  // Wait for 1 seconds to let the Gzaebo GUI show up.
  ros::Duration(1.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  Eigen::Vector3d desired_position(0.0, 0.0, 6.0);
  double desired_yaw = 0 * DEG_2_RAD;

  ROS_INFO("Publishing waypoint: [%f, %f, %f].", desired_position.x(), desired_position.y(), desired_position.z());

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

  trajectory_pub.publish(trajectory_msg);

  ROS_INFO("angZ = %lf",angZ);

  //Default desired position and yaw.
  desired_yaw = angZ * DEG_2_RAD;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), desired_position.x(),
           desired_position.y(), desired_position.z());
  //trajectory_pub.publish(trajectory_msg);

  while(ros::ok()){

    msg.linear.x = linx;
    msg.linear.y = liny;
    msg.linear.z = linz;

    msg.angular.x = angX;
    msg.angular.y = angY;
    msg.angular.z = angZ;
    if(teste && angZ!=0){
      // Default desired position and yaw.
      ROS_INFO("angZ = %lf",angZ);

      //desired_yaw = landing_pose(angZ);
      //ROS_INFO("desired_yaw = %lf",desired_yaw);

      desired_yaw = angZ;
      ROS_INFO("correct_yaw = %lf",desired_yaw);
      desired_yaw = desired_yaw * DEG_2_RAD;

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        desired_position, desired_yaw, &trajectory_msg);

      trajectory_pub.publish(trajectory_msg);
      teste = false;
    }

    //ROS_INFO("x: [%lf] y: [%lf] z: [%lf] - angX: [%lf] angY: [%lf] angZ: [%lf]", linx, liny, linz, angX, angY, angZ);

    // if (angZ != NULL){
    //     //ROS_INFO_STREAM("Subscriber position:"<<" linear="<<linx<<" angular="<<angZ);
    //     ROS_INFO("I heard: [%lf]", linx);
    //     ROS_INFO("I heard: [%lf]", angZ);
    //     //The above line doesn't publish. It's like printf (but not exactly)
    //     //pub.publish(msg);   //This line is for publishing. It publishes to 'turtle1/pose'
    //  }else{
    //     ROS_INFO("Erro!");
    //  }


    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
