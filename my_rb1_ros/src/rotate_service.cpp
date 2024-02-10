// my_rb1_ros/src/rotate_service.cpp

#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <tf/transform_datatypes.h>

class RotateService {
public:
  RotateService() {
    // Initialize the ROS node handle
    nh = ros::NodeHandle("~");

    // Create the rotate_robot service
    service = nh.advertiseService("/rotate_robot",
                                  &RotateService::rotateCallback, this);

    // Subscribe to the odometry topic
    odomSubscriber =
        nh.subscribe("/odom", 10, &RotateService::odomCallback, this);

    // Publish velocity commands to the cmd_vel topic
    cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Set the stabilization factor (adjust as needed)
    stabilizationFactor = 0.2; // Adjust the stabilization factor as needed
  }

  bool rotateCallback(my_rb1_ros::Rotate::Request &req,
                      my_rb1_ros::Rotate::Response &res) {
    // Store the requested degrees
    requestedDegrees = req.degrees;

    // Set the rotation flag to true
    rotating = true;

    // Calculate angular velocity for rotation
    double angularVelocity = (req.degrees > 0) ? rotationSpeed : -rotationSpeed;

    // Create a Twist message for velocity commands
    geometry_msgs::Twist twist;
    twist.angular.z = angularVelocity;

    // Publish the Twist message to cmd_vel
    cmdVelPublisher.publish(twist);

    // Set the target yaw angle
    double targetYaw = initialYaw + req.degrees;

    // Wait for the rotation to complete or timeout
    ros::Time startTime = ros::Time::now();
    while (ros::ok() &&
           (ros::Time::now() - startTime).toSec() < rotationTimeout) {
      ros::spinOnce();
      ros::Rate(10).sleep();

      // Get the current yaw angle from the odometry message
      double currentYaw = tf::getYaw(odomMsg->pose.pose.orientation);

      // Check if the robot has reached the target yaw angle for positive and
      // negative rotations
      if ((req.degrees > 0 && currentYaw >= targetYaw) ||
          (req.degrees < 0 && currentYaw <= targetYaw)) {
        // Reduce the angular velocity for stabilization
        twist.angular.z = angularVelocity * stabilizationFactor;
        cmdVelPublisher.publish(twist);

        // Check if the stabilization period is completed
        if ((ros::Time::now() - startTime).toSec() >= stabilizationDuration) {
          rotating = false;
          rotationSuccessful = true;
        }
      }
    }

    // Stop the robot after stabilization
    twist.angular.z = 0.0;
    cmdVelPublisher.publish(twist);

    // Check if the rotation was successful
    if (rotationSuccessful) {
      res.result = "Rotation completed successfully";
    } else {
      res.result = "Rotation failed";
    }

    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &receivedOdomMsg) {
    // Store the received odometry message
    odomMsg = receivedOdomMsg;

    // Store the initial yaw angle when not rotating
    if (!rotating) {
      initialYaw = tf::getYaw(odomMsg->pose.pose.orientation);
    }
  }

private:
  ros::NodeHandle nh;
  ros::ServiceServer service;
  ros::Subscriber odomSubscriber;
  ros::Publisher cmdVelPublisher;
  bool rotating = false;
  bool rotationSuccessful = false;
  double requestedDegrees = 0.0;
  double initialYaw = 0.0;
  const double rotationSpeed = 0.5; // Adjust the angular velocity as needed
  const double rotationTimeout =
      10.0; // Adjust the timeout as needed (in seconds)
  const double angleTolerance = 0.01; // Adjust the angle tolerance as needed
  double stabilizationFactor;         // Added member variable
  const double stabilizationDuration =
      1.0; // Adjust the stabilization duration as needed
  nav_msgs::Odometry::ConstPtr odomMsg;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");
  RotateService rotateService;
  ros::spin();

  return 0;
}
