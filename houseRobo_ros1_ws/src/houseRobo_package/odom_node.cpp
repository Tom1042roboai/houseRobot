/**
 * No more ports on Arduino Uno board for wheel encoders due to motor shield driver.
 * Use theoretical calculations of odom values for initial prototype.
 * Future plans will be to use sensor data for calculating odom data.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

#define _USE_MATH_DEFINES
#include <cmath>

float linear_vel_x;
float angular_vel_z;

// ROS callback function
void vel_callback(const geometry_msgs::Twist& vel)
{
    //Follows ROS REP 103 convention
    linear_vel_x = vel.linear.x;
    angular_vel_z = vel.angular.z;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50); // msg queue size of 50
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, vel_callback);

    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;

    ros::Time current_time_ms = ros::Time::now();
    ros::Time prev_time_ms = ros::Time::now();

    ros::Rate r(20.0); // 20 Hz publish rate

    while (ros::ok()) {
        current_time_ms = ros::Time::now();
        float dt = (current_time_ms - prev_time_ms).toSec();

        // ROS REP 103 convention, linear_vel_x is the resultant velocity vector (hypotenuse)
        float robot_vel_x = linear_vel_x * sin(theta);
        if (theta > M_PI) {
            robot_vel_x *= -1;
        }

        float robot_vel_y = linear_vel_x * cos(theta);
        if ((theta < (0.5 * M_PI)) || (theta > (1.5 * M_PI))) {
            robot_vel_y *= -1;
        }

        // Assuming constant velocity over dt
        x += (robot_vel_x * dt);
        y += (robot_vel_y * dt);
        float new_theta = theta + (angular_vel_z * dt);
        // Keep theta in range [0, 2*pi] rad, by performing modulo operation
        float new_theta_factor = new_theta / (2 * M_PI);
        float new_theta_decimal = new_theta_factor - floor(new_theta_factor);
        theta = new_theta_decimal * (2 * M_PI);

        // Publish odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time_ms;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        odom_msg.pose.pose.orientation = odom_quat;

        odom_msg.twist.twist.linear.x = linear_vel_x;
        odom_msg.twist.twist.angular.z = angular_vel_z;

        odom_pub.publish(odom_msg);

        prev_time_ms = current_time_ms;
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}


