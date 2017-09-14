
#include "servo_control.h"
#include "robot_class.h"
Robot charlie;

#include "UltraSonic.h"
// front UT sensor's signal pinout:
Ultrasonic frontUT(44, 44);//trig echo
Ultrasonic scanUT(40,41);//should be 40 and 41


#include "WheelEncoders.h"
#include "PID.h"
#include "IMU.h"
#include "BehaviourTree.h"

//global variables

//-----------------------------------------------------------------------
// the setup function runs once when you press reset or power the board
void setup() {
 //make a robot instance that holds all global variables

 boot_PIDs();
 //serial monitor
  Serial.begin(9600);
  Serial3.begin(15600);//start servo serial port (pin 14)
  
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
  //--setup parameters for BT nodes
  backup_a_bit.timeout = 20000; 
  backup_a_bit.distance_to_move = -6; 
  backup_a_bit.desired_speed = -1.5;

  stop_moving.timeout = 20000; 
  stop_moving.desired_speed = 0;

  turn_90.timeout = 200000;
  turn_90.angle_to_move = 90*PI/180;
  turn_90.desired_speed = 1.5;
  turn_90.direction_to_move = 1;//1 = to the right
  turn_90.angle_error_for_success=3*PI/180;

  turn_90_ccw.timeout = 20000;
  turn_90_ccw.angle_to_move = 90*PI/180;
  turn_90_ccw.desired_speed = 1.5;
  turn_90_ccw.direction_to_move = -1;//1 = to the right
  turn_90_ccw.angle_error_for_success=3*PI/180;

  carpet_sweep.timeout = 40000;//move length of carpet
  carpet_sweep.distance_to_move = 16;
  carpet_sweep.desired_speed = 2.5;

  half_charlie_dist.timeout = 20000;
  half_charlie_dist.distance_to_move = 10;
  half_charlie_dist.desired_speed = 2;

  cw_servo_sweep.desired_position = 0;
  cw_servo_sweep.servo_speed = 8;

  ccw_servo_sweep.desired_position = 180;
  ccw_servo_sweep.servo_speed =8;

  readfrontUT.setUT(1);//set front UT to read front UT
  readsweepUT.setUT(2);//set sweep UT to read sweep UT

  back_up_and_turn_sequence.SkipToRunning() ;//set this to follow the sequence when running
  follow_path.SkipToRunning();//set this to follow the sequence when running
  dont_be_stuck_selector.SkipToRunning();//set this to follow the sequence when running
  UTsweep.SkipToRunning();//set this to follow the sequence when running
  UTsweepSuccess.AlwaysSuccess();
  //----------------------------

  
  //Behavior Tree: Add nodes to selectors -----------------------------
upperlevel_sequence.Attach_Node(5001);//get data
    get_data.Attach_Node(1000);//updateorientation
    get_data.Attach_Node(1001);//ReadfrontUT
    get_data.Attach_Node(1012);//ReadsweepUT
    //get_data.Attach_Node(1013);//populate grid<--move to computer
    
upperlevel_sequence.Attach_Node(5007);//UTsweep<--might be easier to make a 'always keep executing even if something is running' for main process?
    UTsweepSuccess.Attach_Node(5006);
        UTsweep.Attach_Node(1010);//sweep cw
        UTsweep.Attach_Node(1011);//sweep ccw

upperlevel_sequence.Attach_Node(5003);//don't be stuck selector
    dont_be_stuck_selector.Attach_Node(5002);//check for being stuck (emergency trigger)
       emergency_triggers.Attach_Node(1002); //tilt sensing
       emergency_triggers.Attach_Node(1003);//this is the front UT all clear
    dont_be_stuck_selector.Attach_Node(5004);//back up and turn around sequence
       back_up_and_turn_sequence.Attach_Node(1004);//backup
       back_up_and_turn_sequence.Attach_Node(1005);//stop
       back_up_and_turn_sequence.Attach_Node(1006);//turn 90 cw
       back_up_and_turn_sequence.Attach_Node(1005);//stop
       back_up_and_turn_sequence.Attach_Node(1008);//fwd small inches
       back_up_and_turn_sequence.Attach_Node(1005);//stop
       back_up_and_turn_sequence.Attach_Node(1009);//turn 90 ccw
       back_up_and_turn_sequence.Attach_Node(1005);//stop*/
/*
upperlevel_sequence.Attach_Node(500X);//grid base avoidance strategy


///this should be a couple of layers deep to check that the left side is clear before turning left etc.
  avoid_obstacles_selector.Attach_Node(500x);//check right side
      right_side_check.Attach_Node(100x);//grid position 0
      right_side_check.Attach_Node(100x);//turn_left_a_bit
   avoid_obstacles_selector.Attach_Node(500x);//check right side
      right_side_check.Attach_Node(100x);//grid position 1
      right_side_check.Attach_Node(100x);//turn_left_a_bit
*/   
upperlevel_sequence.Attach_Node(5005);//Follow Path
    follow_path.Attach_Node (1007);//fwd xxinches 
    follow_path.Attach_Node(1005);//stop
    follow_path.Attach_Node (1006);//turn 90 cc
    follow_path.Attach_Node(1005);//stop
    follow_path.Attach_Node (1008);//fwd small inches
    follow_path.Attach_Node(1005);//stop
    follow_path.Attach_Node (1009);//turn 90 cw
    follow_path.Attach_Node(1005);//stop

//----------------------------------------------------
//Magnet calibration x, y, z if desired
//-------------------------------------------------
//calibrate_compass(1);//partial calibration is (1)
//calibrate_gyros();
//calibrate_accelerometers();
  
  
  
  //delay to ensure boards are initialized
  delay(300);

}
//--------------------------------------------------------------

// the loop function runs over and over again forever
void loop() {
 
  upperlevel_sequence.execute();//behaviour tree
  /*for( int i = 0; i <= charlie.num_grids-1; i = i + 1 ){
    Serial.print ("pos: "); Serial.print (i); Serial.print(" = "); Serial.print (charlie.ut_grid[i]);
  }*/
   //Serial.println("!!!!!!!!!!!!!!!!Tick!!!!!!!!!!!!!!!!!!!!!"); 
  //charlie.update_positional_awareness();
  //charlie.report_imu(1);
  //charlie.report_imu(2);
  //charlie.report_imu(3);
  //charlie.report_imu(4);
 
  //stability delay
  Serial.print (charlie.current_speed);Serial.print (" ");  Serial.println (charlie.set_speed);
  //Serial.println(charlie.heading_error);
  delay(1);
   
}




