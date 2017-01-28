

#include "UltraSonic.h"
// front UT sensor's signal pinout:
Ultrasonic frontUT(44);

#include "WheelEncoders.h"
#include "PID.h"
#include "IMU.h"
#include "BehaviourTree.h"
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products

*/



/*
 * Motor control module needs can use 99% for PWM max, can't use 100% :(
 * if 255 is max value for analogWrite, then 99% is about 252.45 -> 252 max!
 *if value 255 is set, then DC engine cogs ...
 * max tested is 254 without cogging 
 */
//pin and global variable definitions







/* Wheel encoder.
       A pin -- digital 2
       B pin -- digital 4
       
    Beacuse of the interrupt port with different board, the following code is only used in UNO and mega2560.
    If you want to use Leonardo or Romeo,you should change digital pin 3 instead of digital pin 2.
    See the link for detail http://arduino.cc/en/Reference/AttachInterrupt
*/



//global variables










// the setup function runs once when you press reset or power the board
void setup() {
 //serial monitor
  Serial.begin(9600);
  
  // initialize digital external LED as an output.
  pinMode(48, OUTPUT);

  //intialize motor pins as output
  pinMode(LHS_CW, OUTPUT);
  pinMode(LHS_CCW, OUTPUT);
  pinMode(RHS_CW, OUTPUT);
  pinMode(RHS_CCW, OUTPUT);
  // set h-bridge pins to 0
  digitalWrite(LHS_CW, LOW);
  digitalWrite(LHS_CCW, LOW);
  digitalWrite(RHS_CW, LOW);
  digitalWrite(RHS_CCW, LOW);

  sensor.begin();       //initialize sensors on 10DOF board


  EncoderInit();  //Initialize the interrupt module for the encoder 0 and 1
  
//Behavior Tree: Add nodes to selectors -----------------------------
upperlevel_sequence.Attach_Node(5001);//get data
    get_data.Attach_Node(1000);//updateorientation
    get_data.Attach_Node(1001);//ReadUT
upperlevel_sequence.Attach_Node(5002);//emergency triggers
    emergency_triggers.Attach_Node(1002); //tilt sensing
    emergency_triggers.Attach_Node(1003);//this is the UT clear--------------might want to put this in with drive? Or put one layer down and if it fails go backwards....

//----------------------------------------------------

  
  //delay to ensure boards are initialized
  delay(300);
}

// the loop function runs over and over again forever
void loop() {
  String new_string = "hello";
  upperlevel_sequence.execute();
  //heartbeat
  digitalWrite(48, HIGH);   // turn the LED off (HIGH is the voltage level)

  //get and set new speed
  
  //test drive mode
 /* 
Serial.println(RangeInInches);
  if (RangeInInches >=7){//assuming nothing in the UT sensor keep driving
       setspeed_PID(2.0,2.0); // right target, left target in in/s
       Serial.println("fwd");
  }else{//or back up and try again
     //Serial.println("stop");
     while ((round(smooth_R_spd*10)/10.0 != 0) && (round(smooth_L_spd*10)/10.0 !=0)){
        
        setspeed_PID(0,0);
        delay (100);
        Serial.println("stop");
      }
     while (RangeInInches <=10){
      
        Serial.println(RangeInInches);
        Serial.println("backward");
        setspeed_PID(-1.5,-1.5); // right target, left target in in/s 
        delay(100);
        frontUT.DistanceMeasure();// get the current signal time;
        RangeInInches = frontUT.microsecondsToInches();//convert the time to inches;
     }
     while ((round(smooth_R_spd*10)/10.0 != 0) && (round(smooth_L_spd*10)/10.0 !=0)){
        
        setspeed_PID(0,0);
        delay (100);
        Serial.println("stop");
      }
     long time_snapshot = millis();
     while ((millis()-time_snapshot) <= 3500){//for xx time
      //turn right
      Serial.println("turn");
      Serial.println(millis()-time_snapshot);
        
        setspeed_PID(-1.5,1.5); // right target, left target in in/s 
        delay(100);
     }
      while ((round(smooth_R_spd*10)/10.0 != 0) && (round(smooth_L_spd*10)/10.0 !=0)){
        
        setspeed_PID(0,0);
        delay (100);
        Serial.println("stop");
      }
  }*/
   digitalWrite(48, LOW);    // turn the LED on by making the voltage LOW
   //end heartbeat  
  //stability delay
  delay(200);
   
}




