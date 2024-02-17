#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt8.h>
#include <Servo.h>
#include <string.h>
#include "SYSTEM_STATE.h"
#include "enumToString.h"

// create servos
Servo SERVO_TOP;
Servo SERVO_BOTTOM;
Servo SERVO_LEFT;
Servo SERVO_RIGHT;



// motor pins
int PIN_SERVO_TOP = 0;
int PIN_SERVO_BOTTOM = 0;
int PIN_SERVO_LEFT = 0;
int PIN_SERVO_RIGHT = 0;


// twist variables
double radius_norm;
double theta;

// splitting out the motor actuations by theta angle
double lr_norm;
double tb_norm;

// robot limits (degrees of servo rotation from 90)
double lr_limit;
double tb_limit;

// absolute commands
double lr_abs;
double tb_abs;

ros::NodeHandle nh;

// ROS publisher
std_msgs::UInt8 state;
ros::Publisher system_state("system_state", &state);  // TODO: topicname

// ROS callbacks
// shoule be able to switch to a vector3
void vec3CB(const geometry_msgs::Vector3 msg) {
  // extract message data
  radius_norm = msg.x;
  theta = msg.y;
  // breakout by motor
  lr_norm = radius_norm * cos(theta);
  tb_norm = radius_norm * sin(theta)
  // commands to motors
  lr_abs = lr_norm * lr_limit;
  tb_abs = tb_nrom * tb_limit;
  
  // write the motors
  SERVO_LEFT.write(90 - lr_abs);
  SERVO_RIGHT.write(90 + lr_abs);
  SERVO_TOP.write(90 + tb_abs);
  SERVO_BOTTOM.write(90 - tb_abs);

}

// ROS subscribers
ros::Subscriber<geometry_msgs::Twist> vec3_sub("/new/topic", vec3CB);
// ros::Subscriber<std_msgs::Bool> master_sub("/arduino/cmdCalibrationMaster", masterCB);
// ros::Subscriber<std_msgs::Bool> aux_sub("/arduino/cmdCalibrationAux", auxCB);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(vec3_sub);
  
  // attach the servos
  SERVO_TOP.attach(PIN_SERVO_TOP);
  SERVO_BOTTOM.attach(PIN_SERVO_BOTTOM);
  SERVO_LEFT.attach(PIN_SERVO_LEFT);
  SERVO_RIGHT.attach(PIN_SERVO_RIGHT);
}

void loop() {
  // put your main code here, to run repeatedly:
  state.data = sys;
  // system_state.publish(&state); // might want to use this to publish something though
  nh.spinOnce();
  delay(5);
}
