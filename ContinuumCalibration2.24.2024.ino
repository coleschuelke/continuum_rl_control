#include <Servo.h>

//create servos
Servo upServo; 
Servo downServo;
Servo rightServo;
Servo leftServo; 

//motor pins
int upPos = 0;
int downPos = 0;
int rightPos = 0;
int leftPos = 0;

void setup() 
{
  upServo.attach(27);
  downServo.attach(35);
  rightServo.attach(23);
  leftServo.attach(31);
}

void loop() 
{
    upServo.write(upPos);
    downServo.write(downPos);
    rightServo.write(rightPos); 
    leftServo.write(leftPos);                        
}