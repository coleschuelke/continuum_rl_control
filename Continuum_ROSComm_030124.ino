#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h> //sends floating point numbers of single precision

//Segment 1 Servo Declaration
Servo seg1Up;
Servo seg1Down;
Servo seg1Left;
Servo seg1Right;

//Segment 2 Servo Declaration
Servo seg2Up;
Servo seg2Down;
Servo seg2Left;
Servo seg2Right;

//Segment 3 Servo Declaration
Servo seg3Up;
Servo seg3Down;
Servo seg3Left;
Servo seg3Right;


//Number of servos
const int numServos = 12;

//Creates an instance representing a connection point to the ROS network
ros::NodeHandle nh;

//Array for servos
float jointAngles[numServos] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
//jointAngles[numServos] = {seg1Up, seg1Down, seg1Left, seg1Right, seg2Up...}

//this callback function is supposed to be receiving the 12 float variables from ROS
//I used float values because I figured the angles might have atleast one decimal point
void angleCallback(const std_msgs::Float32MultiArray& msg) 
{
  if (msg.data_length != numServos) 
  {
    return; // Discard the message if the length doesn't match
  }

  else
  {
    for (int i = 0; i < msg.data_length; i++) //loop through the msg length which will equal the number of servos
    {
      jointAngles[i] = msg.data[i]; //assign each joint angle to the corresponding message length
    }
  }

  //Assign the servos to the joint angles received within the array
  seg1Up.write(jointAngles[0]);//TODO: Find way to increment
  seg1Down.write(jointAngles[1]);
  seg1Right.write(jointAngles[2]);
  seg1Left.write(jointAngles[3]);

  seg2Up.write(jointAngles[4]);
  seg2Down.write(jointAngles[5]);
  seg2Right.write(jointAngles[6]);
  seg2Left.write(jointAngles[7]);

  seg3Up.write(jointAngles[8]);
  seg3Down.write(jointAngles[9]);
  seg3Right.write(jointAngles[10]);
  seg3Left.write(jointAngles[10]);
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("jointAngles", &angleCallback);

void setup() {
  nh.initNode(); //Establishes communication between board and ROS
  nh.subscribe(sub); //Sets up subscription for arduino to receive messages on a specified ROS topic, enables communication between diff nodes

  //Need to attach servos
    //Assign the servos to the joint angles received within the array
  seg1Up.attach(27);
  seg1Down.attach(35);
  seg1Right.attach(31);
  seg1Left.attach(23);

  seg2Up.attach(0);
  seg2Down.attach(0);
  seg2Right.attach(0);
  seg2Left.attach(0);

  seg3Up.attach(0);
  seg3Down.attach(0);
  seg3Right.attach(0);
  seg3Left.attach(0);
}

void loop() {
  nh.spinOnce(); //processes incomming ROS messages , allows arduino to respond to messages from ROS
  delay(5); //delay serves to avoid overwhelming the system, prevents exxessive CPU resource concumption
}
