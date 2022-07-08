#include <AccelStepper.h>

// Motor pin definitions:
#define motorPin1  4      // IN1 
#define motorPin2  5      // IN2 
#define motorPin3  6     // IN3 
#define motorPin4  7     // IN4 

#define motor2Pin1  8      // IN1 
#define motor2Pin2  9      // IN2 
#define motor2Pin3  10     // IN3 
#define motor2Pin4  11     // IN4 

#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper Joint1 = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);
AccelStepper Joint2 = AccelStepper(MotorInterfaceType, motor2Pin1, motor2Pin3, motor2Pin2, motor2Pin4);

long int incomingByte = 0x0;

void setup() 
{
    
    Serial.begin(9600);

    Joint1.setMaxSpeed(1000);    
    Joint1.setCurrentPosition(0);

    Joint2.setMaxSpeed(1000);    
    Joint2.setCurrentPosition(0);

}

signed short int qd[2] = { 0 , 0 };

unsigned int RX_Buffer[5] = { 0 }; 
int RX_Index = 0;
int joint_index = 0;

unsigned long previousMillis = 0;        // will store last time LED was updated
// constants won't change :
const long interval = 100;           // interval at which to blink (milliseconds)

void loop() 
{
  
  if (Serial.available() > 0) 
  {
      long int curByte = Serial.read();
            
      RX_Index++;

      if(curByte == 0xA)
      {   
          RX_Index = 0;
                
          qd[0] = (RX_Buffer[1] << 8) | RX_Buffer[0];
          qd[1] = (RX_Buffer[3] << 8) | RX_Buffer[2];
          
      } 
      else
      {
            RX_Buffer[RX_Index - 1] = curByte;
      }
      
  }

  int max_vel = 300;
  
  if(qd[0] > max_vel)
  {
    qd[0] = max_vel;
  }

  
  if(qd[1] > max_vel)
  {
    qd[1] = max_vel;
  }
  
  Joint1.setSpeed(qd[0]);
  Joint1.runSpeed(); 

  Joint2.setSpeed(qd[1]);
  Joint2.runSpeed(); 
}
