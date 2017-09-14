/*
 * Motor control module needs can use 99% for PWM max, can't use 100% :(
 * if 255 is max value for analogWrite, then 99% is about 252.45 -> 252 max!
 *if value 255 is set, then DC engine cogs ...
 * max tested is 254 without cogging 
 */



#include <PID_v1.h>
// define max motor speed, can't be higher than around 250
#define MAX_motor_spd 200 //pwm pulses around 200 is ususally good

// define pins for motor controllers
//LHS motor (Drivers perspective)
#define LHS_CW 10
#define LHS_CCW 11

//RHS motor (drivers perspective)
#define RHS_CW 12
#define RHS_CCW 13

//Define Variables we'll be connecting to
double LH_Setpoint =0, LH_Input =0, LH_Output;
double RH_Setpoint=0, RH_Input=0, RH_Output;
double Speed_Output;//speed  is in inches/sec but output should be PWM's to make the math easy
double Heading_Output; //output is in motor PWM can be +/- I guess (this may need to change)
double adjuster = 0;

//Specify the links and initial tuning parameters

//PID Heading_PID(&charlie.current_heading_est, &Heading_Output, &charlie.current_desired_heading,10,100,0, DIRECT);//old
PID Heading_PID(&charlie.heading_error, &Heading_Output, 0,80,80,.1, DIRECT);//10/100/0 has to be geographic heading or this doesn't make sense (keeps going until heading achieved).  target is 0 heading error.  error is updated in IMU update loop.
PID Speed_PID(&charlie.current_speed, &Speed_Output, &charlie.set_speed,50,80,0.1, DIRECT);

void boot_PIDs()
{
  
  //use max/min motor speed for each, to save variable space.  mean nothing anyway since it is scaled with P
  Speed_PID.SetOutputLimits(-MAX_motor_spd, MAX_motor_spd);
  Heading_PID.SetOutputLimits(-MAX_motor_spd, MAX_motor_spd);

  //turn the PID on
  Speed_PID.SetMode(AUTOMATIC);
  Heading_PID.SetMode(AUTOMATIC);
}

void update_PIDs()//takes a speed and heading and sets PWM of both motors accordingly
{
  CaptureBothWheelSpeeds();
  Heading_PID.Compute();//;gives a value of how far off we are in heading returned in PWM's
  Speed_PID.Compute();//looks at average speed of two tracks and returns an adjustment in PWM's
 
  
  if (abs(Speed_Output) + abs(Heading_Output)> MAX_motor_spd){adjuster = MAX_motor_spd-(Speed_Output + Heading_Output);}
  LH_Setpoint = Speed_Output - Heading_Output-adjuster;//<--might have this backwards
  RH_Setpoint = Speed_Output + Heading_Output-adjuster;//<--need something here to prevent saturation, i.e. adjuster, ensure that heading differential can be maxed out even if speed is saturated
  if (charlie.set_speed ==0){LH_Setpoint=0;RH_Setpoint=0;}//full stop if we want to stop
  
  //now send the PWM values to the motors...(LH_Output is the desired PWM set point)
  if (RH_Setpoint >= 0){//forward
      analogWrite(RHS_CCW,LOW);
      analogWrite(RHS_CW,abs(RH_Setpoint));
      charlie.direction_RH = 1;
   }else{//backwards
      analogWrite(RHS_CW,LOW);
      analogWrite(RHS_CCW,abs(RH_Setpoint));
      charlie.direction_RH = 0;
   }
  if (LH_Setpoint >= 0){
      analogWrite(LHS_CCW,LOW);
      analogWrite(LHS_CW,abs(LH_Setpoint));
      charlie.direction_LH = 1;
  }else{
      analogWrite(LHS_CW,LOW);
      analogWrite(LHS_CCW,abs(LH_Setpoint));
      charlie.direction_LH = 0;
   }
  
  
   //Serial.print ("curr speed: "); Serial.print (charlie.current_speed);Serial.print (" set speed: ");Serial.println (charlie.set_speed);
   //Serial.print ("curr heading: "); Serial.print (charlie.current_heading_est);Serial.print (" set heading: ");Serial.println (charlie.current_desired_heading);
   //Serial.print ("RH PWM: "); Serial.print (RH_Setpoint);Serial.print (" LH PWM: ");Serial.println (LH_Setpoint);

   
}



