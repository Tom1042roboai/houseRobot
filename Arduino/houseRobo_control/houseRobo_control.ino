/**
 * Implementation references the following open source project file:
 * github src: https://github.com/XRobots/Navigation-Unit/blob/main/Arduino/openloopWheels/openloopWheels.ino
 * 
 * Current implementation only allows vehicle to drive forwards and turn left
 * Future improvements:
 *  - Allow negative values for linear and angular velocities to enable vehicle to reverse and turn right.
 *    Loads on left and right side motors can then be evenly distributed for turning maneuvers.
 */

#include <AFMotor.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <math.h>

ros::NodeHandle nh;

// Motor Pin Setups
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// cmd_vel from Ros navigation stack
float linear_vel_x;
float angular_vel_z;

unsigned long current_ms;
unsigned long previous_ms;
int loop_time = 50;
// Custom topic for no wheel encoder hack
geometry_msgs::Vector3 data;
ros::Publisher vector_pub("ard_vel_vector", &data);

// ROS callback function
void vel_callback(const geometry_msgs::Twist& vel)
{
    linear_vel_x = vel.linear.x; // linear velocity x set in range of [0, 0.3] m/s
    angular_vel_z = vel.angular.z; // angular velocity z set in the range [0, 1] rad/s
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", vel_callback);


void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(vector_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();

  current_ms = millis();

  // Robot control for cmd_vel
  if ((current_ms - previous_ms) >= loop_time) {
    previous_ms = current_ms;

    if (linear_vel_x == 0 & angular_vel_z == 0) {
      //stop vehicle
      drive_left_wheels(0, RELEASE);
      drive_right_wheels(0, RELEASE);
    }
    else {
      // Equations from experimental estimates
      float ard_linear_vel_x = (500 * linear_vel_x) + 45;
      float ard_angular_vel_z = (4 * (180 / M_PI) * angular_vel_z) + 60;
  
      int ard_speed_left_wheels = round(ard_linear_vel_x);
      int ard_speed_right_wheels = ard_speed_left_wheels + round(ard_angular_vel_z);

      ard_speed_left_wheels = constrain(ard_speed_left_wheels, 0, 255);
      ard_speed_right_wheels = constrain(ard_speed_right_wheels, 0, 255);
  
      if (drive_left_wheels == 0 & drive_right_wheels > 0) { // turn left
        drive_left_wheels(ard_speed_left_wheels, RELEASE);
        drive_right_wheels(ard_speed_right_wheels, FORWARD);
      }
      else { // drive forwards
        drive_left_wheels(ard_speed_left_wheels, FORWARD);
        drive_right_wheels(ard_speed_right_wheels, FORWARD);
      }
    }
    data.x = linear_vel_x;
    data.y = 0.0;
    data.z = angular_vel_z;
    vector_pub.publish(&data);
  }
}


void drive_left_wheels(int speed, uint8_t cmd) {
  motor1.setSpeed(speed);
  motor1.run(cmd);
  motor2.setSpeed(speed);
  motor2.run(cmd);
}

void drive_right_wheels(int speed, uint8_t cmd) {
  motor3.setSpeed(speed);
  motor3.run(cmd);
  motor4.setSpeed(speed);
  motor4.run(cmd);
}
