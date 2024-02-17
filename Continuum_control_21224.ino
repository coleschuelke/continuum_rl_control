#include <MeanFilterLib.h>


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
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

// current reference pin
int SP_REF = 0;

// motor current pins
int SP_Right = 0;
int SP_LEFT = 0;
int SP_UP = 0;
int SP_DOWN = 0;


// bools
bool CALIBRATION_MASTER = false;
// bool CALIBRATION_AUX = false;
// bool last_master_cal = false;
// bool last_aux_cal = false;

SYSTEM_STATE sys;

// mapped values
double moveUp;
double moveRight;


// twist variables
double moveUp_norm;
double moveRight_norm;

double maxCurr = 500; // max current in mA

double idleCurrentUp;
double idleCurrentDown;
double idleCurrentRight;
double idleCurrentLeft;

double currLeft_mA;
double currRight_mA;
double currUp_mA;
double curr_Down_mA;

// rolling avg init
MeanFilter<double> currentFilter_Right(50);
MeanFilter<double> currentFilter_Left(50);
MeanFilter<double> currentFilter_Up(50);
MeanFilter<double> currentFilter_Down(50);


// raw and filtered value inits
double currRight_raw;
double currLeft_raw;
double currUp_raw;
double currDown_raw;


double currRight_filtered;
double currLeft_filtered;
double currUp_filtered;
double currDown_filtered;

// current calculation
double currentCalculation(int refValue, int measuredValue) {
  return (refValue - measuredValue) * 0.00488 * 1000 / 1.3;
}

// read sensors
void readSensors() {
  // read all the pins
  currREF = analogRead(SP_REF) * 0.0048 * 1000;
  currRight_raw = currentCalculation(currREF, analogRead(SP_Right));  // for now since we are only measuring current for the roll servos, I just put in the right ilde current as a dummy value for everything else
  currLeft_raw = currentCalculation(currREF, analogRead(SP_Left));
  currUp_raw = currentCalculation(currREF, analogRead(SP_Up));
  currDown_raw = currentCalculation(currREF, analogRead(SP_Down));


  // add to the filter
  currentFilter_Right.AddValue(currRight_raw);
  currentFilter_Left.AddValue(currLeft_raw);
  currentFilter_Up.AddValue(currUp_raw);
  currentFilter_Down.AddValue(currDown_raw);


  // Filter
  currRight_filtered = currentFilter_Right.GetFiltered();
  currLeft_filtered = currentFilter_Left.GetFiltered();
  currUp_filtered = currentFilter_Up.GetFiltered();
  currDown_filtered = currentFilter_Down.GetFiltered();

}

ros::NodeHandle nh;

// ROS publisher
std_msgs::UInt8 state;
ros::Publisher system_state("system_state", &state);  // TODO: topicname

// ROS callbacks
void masterCB(const std_msgs::Bool msg) {
  last_master_cal = CALIBRATION_MASTER;
  CALIBRATION_MASTER = msg.data;
}

void twistCB(const geometry_msgs::Twist msg) {  // might switch to int8's
  // extract message data
  translateX_norm = msg.linear.x;
  translateY_norm = msg.linear.y;
  translateZ_norm = msg.linear.z;
  roll_norm = msg.angular.x;
  pitch_norm = msg.angular.y;
  yaw_norm = msg.angular.z;

  translateX = translateX_norm * 90;
  translateY = translateX_norm * 90;
  translateZ = translateX_norm * 90;
  roll = roll_norm * 90;  // right/supination = positive
  pitch = pitch_norm * 90;
  yaw = yaw_norm * 90;


  if (CALIBRATION_MASTER && last_master_cal != CALIBRATION_MASTER) {  // final calibration state
                             // how to make sure this only runs once if it is being sent continuously
    TRANSLATE_TOP.write(90);
    TRANSLATE_BOTTOM.write(90);
    TRANSLATE_LEFT.write(90);
    TRANSLATE_RIGHT.write(90);
    ROLL_LEFT.write(90);
    ROLL_RIGHT.write(90);
    PITCH.write(90);
    YAW.write(90);

    // tighten tethers
    ROLL_RIGHT.write(106);
    for (int iter = 0; iter < 70; iter++) {
      readSensors();
    }
    idleCurrentRight = currentFilter_RollRight.GetFiltered();  // gets the filtered idle value

    // readSensors();
    while (currRight_filtered < 1.2 * idleCurrentRight) {
      ROLL_RIGHT.write(106);
      readSensors();
    }
    ROLL_RIGHT.write(90);

    ROLL_LEFT.write(106);
    for (int iter = 0; iter < 70; iter++) {
      readSensors();
    }
    idleCurrentLeft = currentFilter_RollLeft.GetFiltered();

    while (currLeft_filtered < 1.2 * idleCurrentLeft) {
      ROLL_LEFT.write(106);
      readSensors();
    }
    ROLL_LEFT.write(90);

    sys = MasterCalibration;
  } else if (CALIBRATION_AUX && last_aux_cal != CALIBRATION_AUX) {  // intermediate calibration state while screws are tightened 
    TRANSLATE_TOP.write(105);
    TRANSLATE_BOTTOM.write(105);
    TRANSLATE_LEFT.write(105);
    TRANSLATE_RIGHT.write(105);
    ROLL_LEFT.write(90);
    ROLL_RIGHT.write(90);
    PITCH.write(90);
    YAW.write(90);


    sys = AuxCalibration;
  } else {

    // if statement to set sys based on what the motors are actually doing
    // conditions for system states

    // not sure we want to set everything else to 90 for each cue, bc that can make a lot of motions all at once and can feel like a relative opposite of the last cue given
    if (roll_norm != 0) {
      // so we don't really have a way of restarting the cue once we come off of it, because the current just drops immediately
      // unless with the way the callback works you might get a natural pulse in the right direction for the duration of time that the current is "maxed out"
      // hacky but interesting
      // if current is less than xx
      //roll
      if (currRollRight_filtered < maxCurr) { // give the cue
        TRANSLATE_TOP.write(90);
        TRANSLATE_BOTTOM.write(90);
        TRANSLATE_LEFT.write(90);
        TRANSLATE_RIGHT.write(90);
        ROLL_LEFT.write(90 - roll);   // should these be just standard values? probably
        ROLL_RIGHT.write(90 + roll);  // really the conditions for the current control should go inside of here
        PITCH.write(90);
        YAW.write(90);
      } else { // stop everything
        TRANSLATE_TOP.write(90);
        TRANSLATE_BOTTOM.write(90);
        TRANSLATE_LEFT.write(90);
        TRANSLATE_RIGHT.write(90);
        ROLL_LEFT.write(90);   // should these be just standard values? probably
        ROLL_RIGHT.write(90);  // really the conditions for the current control should go inside of here
        PITCH.write(90);
        YAW.write(90);
      }
      sys = Roll;
    } else if (translateZ_norm != 0) {
      TRANSLATE_TOP.write(90 + translateZ);
      TRANSLATE_BOTTOM.write(90 - translateZ);
      TRANSLATE_LEFT.write(90);
      TRANSLATE_RIGHT.write(90);
      ROLL_LEFT.write(90);
      ROLL_RIGHT.write(90);
      PITCH.write(90);
      YAW.write(90);
      sys = TranslateZ;
    } else if (translateX_norm != 0) {
      TRANSLATE_TOP.write(90);
      TRANSLATE_BOTTOM.write(90);
      TRANSLATE_LEFT.write(90 + translateY);
      TRANSLATE_RIGHT.write(90 - translateY);
      ROLL_LEFT.write(90);
      ROLL_RIGHT.write(90);
      PITCH.write(90);
      YAW.write(90);
      sys = TranslateZ;
    } else if (translateY_norm != 0) {
      TRANSLATE_TOP.write(90 + translateY);
      TRANSLATE_BOTTOM.write(90 + translateY);
      TRANSLATE_LEFT.write(90 + translateY);
      TRANSLATE_RIGHT.write(90 + translateY);
      ROLL_LEFT.write(90);
      ROLL_RIGHT.write(90);
      PITCH.write(90);
      YAW.write(90);
      sys = TranslateY;
    } else if (pitch_norm != 90) {
      TRANSLATE_TOP.write(90);
      TRANSLATE_BOTTOM.write(90);
      TRANSLATE_LEFT.write(90);
      TRANSLATE_RIGHT.write(90);
      ROLL_LEFT.write(90);
      ROLL_RIGHT.write(90);
      PITCH.write(90 + pitch);
      YAW.write(90);
      sys = Pitch;
    } else if (yaw_norm != 90) {
      TRANSLATE_TOP.write(90);
      TRANSLATE_BOTTOM.write(90);
      TRANSLATE_LEFT.write(90);
      TRANSLATE_RIGHT.write(90);
      ROLL_LEFT.write(90);
      ROLL_RIGHT.write(90);
      PITCH.write(90);
      YAW.write(90 + yaw);
      sys = Yaw;
    } else {
      TRANSLATE_TOP.write(90);
      TRANSLATE_BOTTOM.write(90);
      TRANSLATE_LEFT.write(90);
      TRANSLATE_RIGHT.write(90);
      ROLL_LEFT.write(90);
      ROLL_RIGHT.write(90);
      PITCH.write(90);
      YAW.write(90);
      sys = Invalid;
    }
  }
}

// ROS subscribers
ros::Subscriber<geometry_msgs::Twist> twist_sub("/arduino/cmdNormalized", twistCB);
// TODO: subscribe to calibration topics
ros::Subscriber<std_msgs::Bool> master_sub("/arduino/cmdCalibrationMaster", masterCB);
ros::Subscriber<std_msgs::Bool> aux_sub("/arduino/cmdCalibrationAux", auxCB);


void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(twist_sub);
  nh.subscribe(master_sub);
  nh.subscribe(aux_sub);
  nh.advertise(system_state);
  // attach the servos
  TRANSLATE_TOP.attach(PIN_TRANSLATE_TOP);
  TRANSLATE_BOTTOM.attach(PIN_TRANSLATE_BOTTOM);
  TRANSLATE_LEFT.attach(PIN_TRANSLATE_LEFT);
  TRANSLATE_RIGHT.attach(PIN_TRANSLATE_RIGHT);
  ROLL_LEFT.attach(PIN_ROLL_LEFT);
  ROLL_RIGHT.attach(PIN_ROLL_RIGHT);
  PITCH.attach(PIN_PITCH);
  YAW.attach(PIN_YAW);

  // sensor reading setup
  pinMode(SP_Right, INPUT);
  pinMode(SP_Left, INPUT);
  pinMode(SP_Top, INPUT);
  pinMode(SP_Bottom, INPUT);
  pinMode(SP_RollRight, INPUT);
  pinMode(SP_RollLeft, INPUT);
  pinMode(SP_Yaw, INPUT);
  pinMode(SP_Pitch, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  state.data = sys;
  // nh.loginfo(enumToString(sys).c_str());
  system_state.publish(&state);
  nh.spinOnce();
  delay(5);
}
