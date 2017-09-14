//sets up charlie as a class so I can access variables from other classes and functions
class Robot{
  public:
      double pitch_a, roll_a, Xa, Ya, Za;//components for pitch and roll and acceleration as calculated (not measured)RADIANS
      double Xg, Yg, Zg;//components from gyro
      double Xm, Ym, Zm; //components from magnet
      double magX, magY, magZ; //adjusted mag angles in radians
      double MAX_PITCH = 30*PI/180;//rads
      double MAX_ROLL = 30*PI/180;//rads
      //components for gyro reads
      double Xg_old = 0;
      double Yg_old = 0;
      double Zg_old = 0;
      int new_time_stamp =0;//time stamps for gyros
      int old_time_stamp =0;//time stamps for gyros
      double roll, pitch, yaw=0;//filtered angles RADIANS
      double roll_a_last, pitch_a_last, roll_a_smooth, pitch_a_smooth;//last accel estimates for smoothing
      double yaw_mag, gyro_angle_change; //last yaw for smoothing       
      double current_heading_est = 0, current_desired_heading = 0; //in radians.  SHould match with IMU compus settings. I guess north is 0 and east is +ve?  Always above 0.
      double current_speed = 0, set_speed = 0;// in in/s
      double heading_error = 0; //+ means need to turn right, negative meand need to turn left. error compared to where we want to be
      
      //max and min values for compus calibration
      double max_Xm = 0;
      double max_Ym = 0;
      double max_Zm = 0;
      double min_Xm = 0;
      double min_Ym = 0;
      double min_Zm = 0;
      
      double mag_cal_X = -6.14; //compus calibration for x direction
      double mag_cal_Y = -18.64; //compus calibration for Y direction
      double mag_cal_Z = -17.07; //compus calibration for z direction

      double mag_scale_X = 51.14;//axis scale based on max/min (rad) for magnet
      double mag_scale_Y = 51.36;
      double mag_scale_Z = 53.41;
      double avg_rad = (mag_scale_X + mag_scale_Y +mag_scale_Y)/3.;//average radius of mag netic calibration circle
       
      double pitch_cal = -0*PI/180;//adjusts to read 0 (note can't apply before magnet reading since on same board)
      double roll_cal = -1.5*PI/180;

      double gyro_cal_X = .02, gyro_scale_X = 1; //compus calibration for x direction
      double gyro_cal_Y = .01,gyro_scale_Y = 1; //compus calibration for Y direction
      double gyro_cal_Z = .01,gyro_scale_Z = 1.13; //compus calibration for z direction

      double accel_cal_X = -.05; //compus calibration for x direction
      double accel_cal_Y = -.08; //compus calibration for Y direction
      double accel_cal_Z = -.02; //compus calibration for z direction

      double min_cal_angle = 2*M_PI*1.1;//minimum rotation required to consider a good calibration

      boolean direction_RH = 1;                            //the rotation direction of the right wheels, 0backwards 1 forwards
      boolean direction_LH = 1;                            //the rotation direction of the left wheels

      long servo_position = 0;//servo position in degrees.  0 is full right 180 is full left.
      long servo_max = 5000;
      long servo_min =1000;

      static const int num_grids =9;//setup UT grid so it can be used from whereever
      long ut_grid[num_grids];//initiate the grid array with 9 spaces, element 4 is the centre element 0 is fully ccw (left side).
      long grid_size = (180)/num_grids;//in degrees

      void update_positional_awareness();
      void update_imu_values ();
      void update_heading();
      void report_imu (int);
      boolean check_grid_position (int, long);

      double activity_level;//how active is charlie
      double novelty_level;//how novel is what we are doing

   private:
      int dtime; //delta t bewteen this and last time stamp
      
  };
boolean Robot::check_grid_position (int position_to_check, long comparison_value){//checks if a grid position is less than or equal to a value
  //true means there is something there, false means nothing there
  if(ut_grid[position_to_check] <=comparison_value){//it is less than or equal to the comparator
    return true;
  }else{return false;}

}


