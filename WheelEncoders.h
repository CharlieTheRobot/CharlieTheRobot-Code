/* Wheel encoder.
       A pin -- digital 2
       B pin -- digital 4
       
    Beacuse of the interrupt port with different board, the following code is only used in UNO and mega2560.
    If you want to use Leonardo or Romeo,you should change digital pin 3 instead of digital pin 2.
    See the link for detail http://arduino.cc/en/Reference/AttachInterrupt
*/


// define max motor speed, can't be higher than around 250
#define MAX_motor_spd 220
// define pins for motor controllers
//LHS motor (Drivers perspective)
#define LHS_CW 10
#define LHS_CCW 11

//RHS motor (drivers perspective)
#define RHS_CW 12
#define RHS_CCW 13

//right wheel encoder 0
const byte encoder0pinA = 19;                  //A pin -> the interrupt pin 2
const byte encoder0pinB = 18;                  //B pin -> the digital pin 3
byte encoder0PinALast;
int num_pulses0;                               //the number of the pulses
boolean Direction0;                            //the rotation direction 
long last_pulse_time0 =0; //tracks when the last pulse time was to remove spurious pulses

//left wheel encoder 1
const byte encoder1pinA = 2;                  //A pin -> the interrupt pin 19
const byte encoder1pinB = 3;                  //B pin -> the digital pin 18
byte encoder1PinALast;
int num_pulses1;                               //the number of the pulses
boolean Direction1;                            //the rotation direction 
long last_pulse_time1 =0; //tracks when the last pulse time was to remove spurious pulses

//PID control variables
unsigned long lastPIDTime = 0;
double PIDerrSumR, PIDlastErrR;
double PIDerrSumL, PIDlastErrL;
double PIDerrSumTheta, PIDlastErrTheta;
double kp = 10;
double ki = .1; 
double kd = 50;

double current_R_spd = 0.;// right wheel speed in in/s
double current_L_spd = 0.;//left wheel speed in in/s
double last_wheel_R_spd = 0;
double last_wheel_L_spd = 0;
double smooth_R_spd = 0; //smoothed right wheel speed in in/s
double smooth_L_spd = 0; //smoothed left wheel speed in in/s
long last_spd_read_time =0;//last time the wheel speed was calculated
long second_last_spd_read_time = 0;  //time before last that speed was read

void WheelAddTick1(){
        if(Direction1)  num_pulses1++;//duraction is now named num_pulses
        else  num_pulses1--;

}
void WheelAddTick0(){
      if(Direction0)  num_pulses0++;//duraction is now named num_pulses
      else  num_pulses0--;
 }
void WheelPulses0(){
  long this_pulse_time0 =0;//used to check for valid pulse time increment
  int curr_state0 = digitalRead(encoder0pinA);

  if((encoder0PinALast == LOW) && curr_state0==HIGH){//rising change only
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction0){
      Direction0 = true;                       //Forward
    }
    else if(val == HIGH && !Direction0){
      Direction0 = false;                        //reverse
    }
  }
  encoder0PinALast = curr_state0;
  
   //only increment if it is possible we've pulsed
  this_pulse_time0 = millis();
  if(this_pulse_time0-last_pulse_time0 > 1){
      if(Direction0)  num_pulses0++;//duration is now named num_pulses
      else  num_pulses0--;
      last_pulse_time0 =this_pulse_time0;
  }
  
}
 
void WheelPulses1(){
  long this_pulse_time1 =0;//used to check for valid pulse time increment
  int curr_state1 = digitalRead(encoder1pinA);

  if((encoder1PinALast == LOW) && curr_state1==HIGH){//rising change only
    int val = digitalRead(encoder1pinB);
    if(val == LOW && Direction1){
      Direction1 = false;                       //forward
    }
    else if(val == HIGH && !Direction1){
      Direction1 = true;                        //reverse
    }
  }
  encoder1PinALast = curr_state1;
  
   //only increment if it is possible we've pulsed
  this_pulse_time1 = millis();
  if(this_pulse_time1-last_pulse_time1 > 1){
      if(Direction1)  num_pulses1++;//duraction is now named num_pulses
      else  num_pulses1--;
      last_pulse_time1 =this_pulse_time1;
  }
}

//calculates the actual expected land speed based on the number of encoder pulses
//returns the speed in inches/s
double GetWheelSpeed(int pulses_since_last_read, long last_speed_read_time){
 long current_time = millis();

 if ((current_time - last_speed_read_time) == 0){//check to make sure not divide by 0
  return -1;//error
 }else{
  return (1000 * 3.14*1.9/40 * pulses_since_last_read) /(current_time - last_speed_read_time);  //.301" per pulse convert to seconds
 } 
}

double Smooth(double measurement, double curr_value, double alpha){
  return   alpha * measurement + (1 - alpha)*curr_value;
}

//saves both wheel speeds to global variables
void CaptureBothWheelSpeeds(){
  double alpha = 0.8;//smoothing factor
  last_wheel_R_spd = current_R_spd;
  last_wheel_L_spd = current_L_spd;
  current_R_spd = GetWheelSpeed(num_pulses0,last_spd_read_time);
  current_L_spd = GetWheelSpeed(num_pulses1,last_spd_read_time);
  num_pulses0 = 0;//right reset this until next time we read speed
  num_pulses1 = 0;//left
  second_last_spd_read_time = last_spd_read_time;
  last_spd_read_time= millis();//reset read time until next time we read speed
  smooth_R_spd = Smooth(current_R_spd, smooth_R_spd, alpha);
  smooth_L_spd = Smooth(current_L_spd, smooth_L_spd, alpha);
}

void EncoderInit(){
  Direction0 = true;                            //default -> Forward  
  pinMode(encoder0pinA,INPUT);
  pinMode(encoder0pinB,INPUT);
  pinMode(encoder1pinA,INPUT);
  pinMode(encoder1pinB,INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), WheelPulses0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0pinB), WheelAddTick0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1pinA), WheelPulses1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1pinB), WheelAddTick1, CHANGE);
}
double estimate_distance_since_last_update(long time_old, long time_new, double speed_est){//estimates amount moved since last wheel reading
    // speed estimate (eventually with IMU smoothing)
    return speed_est * (time_new-time_old);//
}


