#include <Servo.h>

//create servos
Servo upServo; 
Servo downServo;
Servo rightServo;
Servo leftServo; 

//motor positions
int upPos = 90;
int downPos = 90;
int rightPos = 90;
int leftPos = 90;

void setup() 
{
  upServo.attach(27);
  downServo.attach(35);
  rightServo.attach(23);
  leftServo.attach(31);

  upServo.write(upPos);
  downServo.write(downPos);
  rightServo.write(rightPos); 
  leftServo.write(leftPos); 
}

void loop() 
{                      
}
