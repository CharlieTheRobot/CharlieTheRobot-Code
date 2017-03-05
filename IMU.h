//setup Gsensor (10DOF), pins must be SDA/STL pins on mega board
#include <Wire.h>
#include <GY80.h>

GY80 sensor = GY80(); //create GY80 instance

const float alpha_a = 0.5;//smoothing for the accelerometer calcs
const float alpha_g = 0.5;//smoothing for the gyro calcs

// complimentary filters
//tau around .075 is recommended
float Complementary(float last_estimate_integral_sensor, float truth_Sensor_value, float rate_of_change, int looptime, float tau) {
  float dtC = float(looptime)/1000.0;
  float accurate_estimate;
  float a=0.0;
  a=tau/(tau+dtC);
  accurate_estimate = a* (last_estimate_integral_sensor + rate_of_change * dtC) + (1-a) * (truth_Sensor_value);
  return accurate_estimate;
}


   
void Robot::update_imu_values (){
        
    GY80_scaled val = sensor.read_scaled();       //get values from all sensors on 10dof board
    // components for accelerometer
    //units of in g where 9.81m/s is one g
    Xa = val.a_x-accel_cal_X;
    Ya = val.a_y-accel_cal_Y;
    Za = val.a_z-accel_cal_Z;

    // components for gyros
    //units in degrees per second (6dps = 1rpm) converted to RAD/s
    Xg = (val.g_x * PI /180-gyro_cal_X)*gyro_scale_X;
    Yg = (val.g_y* PI /180-gyro_cal_Y)*gyro_scale_Y;
    Zg = (val.g_z* PI /180-gyro_cal_Z)*gyro_scale_Z;

    //components from magnetometer
    //units in microTesla [ÂµT]
    //these are adjusted for centre point and scale (hard and soft iron errors)
    Xm = (val.m_x - mag_cal_X) * avg_rad/mag_scale_X;
    Ym = (val.m_y - mag_cal_Y)* avg_rad/mag_scale_Y;
    Zm = (val.m_z - mag_cal_Z)* avg_rad/mag_scale_Z;
 }
void Robot::update_heading (){
    
    //reset time since last update
    old_time_stamp = new_time_stamp;
    new_time_stamp = millis();
    dtime=(new_time_stamp-old_time_stamp);//loop time in milliseconds  
    
    //Roll and Pitch Equations from accelerometers, unfiltered
    roll_a = atan2(-Ya, Za);//this is before complimentary filter, from accelerometer only RADIANS
    pitch_a = atan2(Xa, sqrt(Ya*Ya + Za*Za));//this is before complimentary filter, from accelerometer only RADIANS
   
    //smooth roll and pitch over time if it is within bounds
    if ((abs(roll_a) - abs(roll_a_last))/dtime*180/PI > 25){//25 deg/s
      //roll change exceeds bounds
    }else{//roll change looks OK, lets use it but use complementary to low pass filter
        roll_a_smooth = Complementary(roll_a_smooth, roll_a, 0, dtime, .2);
        pitch_a_smooth = Complementary(roll_a_smooth, pitch_a, 0, dtime, .2);
   }
    
    //smooth roll and pitch using gyros
    roll = Complementary(roll, roll_a_smooth, Xg, dtime, .2);//last est, accel angle, gyro rate, loop time
    pitch = Complementary(pitch, pitch_a_smooth, Yg,dtime, .2);//multiply by 180/M_PI to get deg

    //compensate the magnetic angles for angle of pitch and roll
    magX=Xm*cos(pitch)+Ym*sin(pitch)*sin(roll)-Zm*cos(roll)*sin(pitch);
    magY=Ym*cos(roll)+Zm*sin(roll);
    magZ = Ym * sin(roll) + Zm * cos(roll);
    //compute magnetic angles and ultimately YAW using compensated angles
    //average out the yaw caculation from magnet over time (low pass filter)
    yaw_mag=Complementary (yaw_mag, atan2(magY,magX) * (180 / PI)-90, 0, dtime, .075); // angle in degrees

    //yaw_mag = atan2(magY,magX) * (180 / PI)-90;//just angle calc, not using complimentary
    if (yaw_mag>0){yaw_mag=yaw_mag-360;}
    yaw_mag=360+yaw_mag;
    yaw_mag = yaw_mag * PI / 180;
    
    //calibrations
    roll = roll - roll_cal;
    pitch = pitch - pitch_cal;
    
    //update change in angle using gyro about Z axis
    gyro_angle_change =  (Zg * dtime/1000);//dtime is ms,gyro is in s
    //update yaw fusing gyro about Z and magnets
     yaw = Complementary(yaw,yaw_mag, Zg, dtime, 0.075);//original (was working if calibration is good)
    
    //yaw = yaw +gyro_angle_change;//no magnet, only gyro
    }
 
void Robot::report_imu (int report_value){
  if (report_value ==1){
      Serial.print("Xm: ");
      Serial.print(Xm);
      Serial.print(" MagX: ");
      Serial.print(magX);
      Serial.print(" Ym: ");
      Serial.print(Ym);
      Serial.print(" magY: ");
      Serial.print(magY);
      Serial.print(" Zm: ");
      Serial.print(Zm);
      Serial.print(" magZ: ");
      Serial.println(magZ);
  }
  if (report_value ==2){  
      Serial.print("Pitch: ");
      Serial.print(pitch*180/M_PI);
      Serial.print(" Roll: ");
      Serial.print(roll*180/M_PI);
      Serial.print(" Yaw: ");
      Serial.println(yaw*180/M_PI);
  }
  if (report_value ==3){
      Serial.print("mag_cal_X "); 
      Serial.print(mag_cal_X); //compus calibration for x direction
      Serial.print("mag_cal_Y "); 
      Serial.print(mag_cal_Y); //compus calibration for Y direction
      Serial.print("mag_cal_Z "); 
      Serial.println(mag_cal_Z); //compus calibration for z direction

     Serial.print("mag_scale_X "); 
      Serial.print(mag_scale_X); //compus calibration for x direction
      Serial.print("mag_scale_Y "); 
      Serial.print(mag_scale_Y); //compus calibration for Y direction
      Serial.print("mag_scale_Z "); 
      Serial.println(mag_scale_Z); //compus calibration for z direction
      
      Serial.print("YAw_mag "); 
      Serial.println(yaw_mag*180/M_PI);
      
  }
  if (report_value ==4){
      Serial.print("Xg: ");
      Serial.print(Xg);
      Serial.print(" Yg: ");
      Serial.print(Yg);
      Serial.print(" Zg: ");
      Serial.print(Zg);

  }
    if (report_value ==5){
      Serial.print("Xa: ");
      Serial.print(Xa);
      Serial.print(" Ya: ");
      Serial.print(Ya);
      Serial.print(" Za: ");
      Serial.print(Za);

  }
}
  
void Robot::update_positional_awareness (){
    int i_max = 4;//total records taken is i_Max + 1
    double imu_values[9][8];//5 is the max value and 6 is the position, 7 is the average
    for( int i = 0; i <= 8; i = i + 1 ){//initialize array
      for(int a= 0; a <= 7;a = a+1){
        imu_values[i][a] = 0;//all as 0's, every call
      }
    } 

   //load imu values into array. 9dof x i_Max readings
    for( int i = 0; i <= i_max; i = i + 1 ) {
        update_imu_values ();
        
        imu_values[0][i]= Xa; 
        imu_values[1][i]= Ya;
        imu_values[2][i]= Za;
        
        imu_values[3][i]= Xg;
        imu_values[4][i]= Yg;
        imu_values[5][i]= Zg;

        imu_values[6][i]= Xm;
        imu_values[7][i]= Ym;
        imu_values[8][i]= Zm;
       
        for(int a= 0; a <= 8;a = a+1){
          //check to see if any of these are the largest of their axis yet
          //if they are the largest, save them in 5 and their index in 6
          
          if (abs(imu_values[a][i]) >= imu_values[a][5]) {
            imu_values[a][5] = abs(imu_values[a][i]); 
            imu_values[a][6] = i;
            
          } 
        }
          
        delay(20);
    }
    //get the average of the reading, there are 5 readings but we only uses 4
    //throw out the highest, as it is the most likely to be an error
    for(int a= 0; a<=8;a = a+1){
      
      for (int i = 0;i<=i_max; i = i+1){
        
          if (i != imu_values[a][6]){//don't add the biggest one to the average
          
            imu_values[a][7] = imu_values[a][7] + imu_values[a][i]/(i_max);
          
          }else{//do nothing
          }
        
       }
      
    }
         //now save the averages into the components for use in other functions
         Xa = imu_values[0][7]; 
         Ya= imu_values[1][7];
         Za= imu_values[2][7];

         Xg= imu_values[3][7];
         Yg= imu_values[4][7];
         Zg= imu_values[5][7];

         Xm= imu_values[6][7];
         Ym= imu_values[7][7];
         Zm= imu_values[8][7];
    update_heading();  
} 

void calibrate_compass(boolean partial = 0){
  long start_time_setup; 
  start_time_setup = millis();
  double cal_angle_moved = 0;
  
  delay(1000);
  if (partial == 1){//if this is a partial cal only
    setspeed_PID(+100,-100);//set speed to desired
  
    while (millis() - start_time_setup < 40000){//give you 40 seconds to calibrate from when you turn it on
      //grab magnetometer values
      charlie.update_positional_awareness();
      cal_angle_moved = cal_angle_moved - charlie.gyro_angle_change;
      Serial.print ("cal_angle_moved"); Serial.println (cal_angle_moved);
      //filter out bad readings

          if (charlie.Xm > charlie.max_Xm ) {charlie.max_Xm = charlie.Xm;}
          if (charlie.Xm < charlie.min_Xm ) {charlie.min_Xm = charlie.Xm;}
    
          if (charlie.Ym > charlie.max_Ym ) {charlie.max_Ym = charlie.Ym;}
          if (charlie.Ym < charlie.min_Ym ) {charlie.min_Ym = charlie.Ym;}

          if(partial ==0){
              if (charlie.Zm > charlie.max_Zm ) {charlie.max_Zm = charlie.Zm;}
              if (charlie.Zm < charlie.min_Zm ) {charlie.min_Zm = charlie.Zm;}
          }
          charlie.report_imu(1);
          if (cal_angle_moved >= charlie.min_cal_angle){break;}//exit if we reach the desired limit
      //}
    };
    setspeed_PID(0,0);
  } else {
    while (millis() - start_time_setup < 40000){//give you 40 seconds to calibrate from when you turn it on
      //grab magnetometer values
      charlie.update_positional_awareness();
      
      //filter out bad readings

          if (charlie.Xm > charlie.max_Xm ) {charlie.max_Xm = charlie.Xm;}
          if (charlie.Xm < charlie.min_Xm ) {charlie.min_Xm = charlie.Xm;}
    
          if (charlie.Ym > charlie.max_Ym ) {charlie.max_Ym = charlie.Ym;}
          if (charlie.Ym < charlie.min_Ym ) {charlie.min_Ym = charlie.Ym;}

          if (charlie.Zm > charlie.max_Zm ) {charlie.max_Zm = charlie.Zm;}
          if (charlie.Zm < charlie.min_Zm ) {charlie.min_Zm = charlie.Zm;}
          
          charlie.report_imu(1);
      //}
    };
  }
  
  if ((partial ==1 and cal_angle_moved >= charlie.min_cal_angle) or partial ==0){
    charlie.mag_cal_X = (charlie.max_Xm + charlie.min_Xm)/2;
    charlie.mag_cal_Y = (charlie.max_Ym + charlie.min_Ym)/2;
    if(partial ==0){charlie.mag_cal_Z = (charlie.max_Zm + charlie.min_Zm)/2;}

    charlie.mag_scale_X = (charlie.max_Xm - charlie.min_Xm)/2;
    charlie.mag_scale_Y = (charlie.max_Ym - charlie.min_Ym)/2;
    if(partial == 0){charlie.mag_scale_Z = (charlie.max_Zm - charlie.min_Zm)/2;}
  }
  Serial.println("final orientation report");
  charlie.report_imu(3);
}
void calibrate_gyros(){
  long start_time_setup; 
  start_time_setup = millis();
  delay(5000);
  while (millis() - start_time_setup < 40000){//give you 40 seconds to calibrate from when you turn it on
      charlie.update_positional_awareness();
      charlie.report_imu (4);
  }
}

void calibrate_accelerometers(){
  long start_time_setup; 
  start_time_setup = millis();
  delay(5000);
  while (millis() - start_time_setup < 40000){//give you 40 seconds to calibrate from when you turn it on
      charlie.update_positional_awareness();
      charlie.report_imu (5);
  }
}

