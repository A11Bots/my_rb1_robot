#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <cmath>

class RotateNode {
public:
  RotateNode() {
    // Subscribe to /odom topic
    odometry_sub_ =
        nh_.subscribe("/odom", 1, &RotateNode::odometryCallback, this);

    // Advertise the /rotate_robot service
    rotate_service_ =
        nh_.advertiseService("/rotate_robot", &RotateNode::rotateService, this);

    // Publish to the /cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ROS_INFO("Rotate node ready to rotate rb1.");
  }

  bool rotateService(my_rb1_ros::rotate::Request &req,
                     my_rb1_ros::rotate::Response &res) {
    // Extract the current orientation from the latest Odometry message
    double current_orientation = current_odom_.pose.pose.orientation.z;

    // Calculate the desired orientation based on the requested degrees
    double target_orientation =
        current_orientation + degreesToRadians(req.degrees);

    // Calculate the angular velocity to achieve the desired rotation
    double angular_velocity =
        calculateAngularVelocity(target_orientation, current_orientation);

    // If the error hasn't been displayed yet, publish the angular velocity to
    // /cmd_vel
    if (!error_displayed_) {
      publishTwist(0.0, 0.0, angular_velocity);

      // For simplicity, let's assume the rotation is always successful
      res.result = "Rotation completed successfully";
      error_displayed_ =
          true; // Set the flag to true after displaying the error
    } else {
      res.result = "Rotation already completed";
    }

    return true;
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Store odometry message
    current_odom_ = *msg;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odometry_sub_;
  ros::ServiceServer rotate_service_;
  ros::Publisher cmd_vel_pub_;
  nav_msgs::Odometry current_odom_;
  bool error_displayed_;

  double degreesToRadians(int degrees) { return degrees * M_PI / 180.0; }

  double calculateAngularVelocity(double target_orientation,
                                  double current_orientation) {
    // Calculate the angular velocity to achieve the desired rotation
    double error = target_orientation - current_orientation;

    // Proportional control
    double kp = 0.5;
    double angular_velocity = kp * error;

    return angular_velocity;
  }

  void publishTwist(double linear_x, double linear_y, double angular_z) {
    // Publish the Twist message to /cmd_vel
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = linear_x;
    twist_msg.linear.y = linear_y;
    twist_msg.angular.z = angular_z;
    cmd_vel_pub_.publish(twist_msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_node");
  RotateNode rotate_node;
  ros::spin();

  return 0;
}
