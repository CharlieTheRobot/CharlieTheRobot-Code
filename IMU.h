//setup Gsensor (10DOF), pins must be SDA/STL pins on mega board
#include <Wire.h>
#include <GY80.h>
GY80 sensor = GY80(); //create GY80 instance

const float alpha_a = 0.5;//smoothing for the accelerometer calcs
const float alpha_g = 0.5;//smoothing for the gyro calcs
double MAX_PITCH = 30;
double MAX_ROLL = 30;
double bs=0;
//components for gyro reads
double Xg_old = 0;
double Yg_old = 0;
double Zg_old = 0;
int new_time_stamp =0;//time stamps for gyros
int old_time_stamp =0;//time stamps for gyros

//variables for complimentary filters
float tau=0.075;
float a=0.0;
double roll, pitch, yaw;//filtered angles

//complimentary filter for combininggyro and accel data quickly.
// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float Complementary(float last_estimate, float accelerometer_angle, float gyro_rate, int looptime) {
float dtC = float(looptime)/1000.0;
float accurate_estimate;
a=tau/(tau+dtC);
accurate_estimate = a* (last_estimate + gyro_rate * dtC) + (1-a) * (accelerometer_angle);
return accurate_estimate;
}

void update_positional_awareness(){  
  double pitch_a, roll_a, Xa, Ya, Za;//components for pitch and roll and acceleration as calculated (not measured)
  double Xg, Yg, Zg;//components from gyro
  double Xm, Ym, Zm; //components from magnet

  int dtime; //delta t bewteen this and last time stamp
  
  GY80_scaled val = sensor.read_scaled();       //get values from all sensors on 10dof board

 // components for accelerometer
 //units of in g where 9.81m/s is one g
   Xa = val.a_x;
   Ya = val.a_y;
   Za = val.a_z;
   
 // components for gyros
 //units in degrees per second (6dps = 1rpm)
   new_time_stamp = millis();
   Xg = val.g_x;
   Yg = val.g_y;
   Zg = val.g_z;

  dtime=(new_time_stamp-old_time_stamp);//loop time in milliseconds

 //components from magnetometer
 //units in microTesla [ÂµT]
   Xm = val.m_x;
   Ym = val.m_y;
   Zm = val.m_z;
//calculate angle using gyro
//these aren't needed since done  as part of the complimentary filter...but leaving to show maths
  //Xg_angle += Xg/()/1000;//estimate of angle from gyro (rate/deltatime)
  //Yg_angle += Yg/(new_time_step-old_time_step)/1000;
  //Zg_angle += Zg/(new_time_step-old_time_step)/1000;
  
  //transfer these readings to be used on the next pass
  //could be used to give a better estimate for average when program interrupted
  //Xg_old = Xg;
  //Yg_old = Yg;
 // Zg_old = Zg;
  old_time_stamp = new_time_stamp;
    

  //Roll and Pitch Equations from accelerometers, unfiltered
 
  roll_a = (atan2(-Ya, Za)*180.0)/M_PI;//this is before complimentary filter, from accelerometer only
  pitch_a = (atan2(Xa, sqrt(Ya*Ya + Za*Za))*180.0)/M_PI;//this is before complimentary filter, from accelerometer only

  roll = Complementary(roll, roll_a, Xg,dtime);//last est, accel angle, gyro rate, loop time
  pitch = Complementary(pitch, pitch_a, Yg,dtime);



bs = bs +Xg*dtime/1000;
//Serial.print("pitch_a ");
//Serial.print(pitch_a);
Serial.print("pitch filtered ");
Serial.print(pitch);
/*
Serial.print("Roll_a ");
Serial.print(roll_a);
Serial.print("Roll filtered ");
Serial.print(roll);
Serial.print("loop time");
Serial.print(dtime);
Serial.print("gyro delta angle");
Serial.print(Xg*dtime/1000);
Serial.print("bs ");
Serial.print(bs);
Serial.println();
*/
}



