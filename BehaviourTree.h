// Behaviour tree 
//activity and novelty seeking
//pre-defines functions used here---------------------
int FunctionRunTable(int function_to_run);//predefine
//---------------------------------------------------

class Node {  // This class represents each node in the behaviour tree.
  //put everything in here that is common to all or more nodes
  public:
    int STATUS = 0 ;//0 = (default) not running, 1 = running, 2 =  success, 3 = fail, 4 = error//should figure out how to take advantage of this->..***********
    int report_status(){
      return STATUS;
    }

};

class UpdateOrientation: public Node{//always returns success unless there is an unhandled error!
  public:
    void execute(){
      //stuff to do
        charlie.update_positional_awareness();
        //Serial.print(F("Heading Est: "));Serial.print(charlie.current_heading_est* 180/PI);Serial.print (" Desired Angle: ");Serial.println (charlie.current_desired_heading * 180/PI);
        update_PIDs();
        
        //return 1;//running
        //Serial.println ("update orientation success");
        charlie.activity_level = charlie.activity_level + activity_importance;//how active is charlie
        charlie.novelty_level=charlie.novelty_level + novelty_importance;
        this->STATUS = 2;
        //return 2;//success
        //return 3;//fail
        //return 4;//error
     }
   private:
    double activity_importance = 1;
    double novelty_importance = 1;
};
class ReadUT: public Node{//always returns success
    public:
        //long RangeInInches=-1; //no longer required
        //void init(){}//initialization loop, probably needs a flag to be set
      void setUT(int _utID){
          utID = _utID;
      }
      void execute(){
        
        //stuff to do
        if (utID ==1){
          frontUT.DistanceMeasure();// get the current signal time;
          frontUT.last_measurement_in_inches = frontUT.microsecondsToInches();//convert the time to inches;
          //Serial.print(F("Last Front UT Measurement"));Serial.println(frontUT.last_measurement_in_inches);
        } else if (utID ==2){
          scanUT.DistanceMeasure();// get the current signal time;
          scanUT.last_measurement_in_inches = scanUT.microsecondsToInches();//convert the time to inches;
          //Serial.print(scanUT.last_measurement_in_inches);
        }

        //return 1;//running
        this->STATUS = 2;
        
      }
    private:
      int utID;
};
class Check_Tilt: public Node{//always returns success
    public:
        
        //void init(){}//initialization loop, probably needs a flag to be set
      void execute(){
        //stuff to do
        //check to see if we are getting into trouble
        if (charlie.pitch > charlie.MAX_PITCH or charlie.pitch <(-1*charlie.MAX_PITCH) ){
            //Serial.println ("Max Pitch Exceeded");
            this->STATUS = 3;
           
        }else if (charlie.roll >charlie.MAX_ROLL or charlie.roll<(-1*charlie.MAX_ROLL)){
           //Serial.println ("Max Roll Exceeded");
           this->STATUS = 3;
          
        }else {
           //Serial.println ("Pitch and Roll are OK");
           //return 1;//running
           this->STATUS = 2;
        }
      
      }
};
class Drive_Distance: public Node{//if not achieved in time then failure
    public:
        long timeout;//howlong the move has untilfailure
        double distance_to_move;//how far in inches you want to move
        double desired_speed;//how fast you would like to move
        
        //void init(){}//initialization loop, probably needs a flag to be set
      void execute(){
        //stuff to do
        if (this->STATUS != 1 ){//capture start time and intialize distance if this is not currently running
          //Serial.println ("Start Moving ");
          start_time = millis(); 
          time_new = millis(); 
          distance_moved = 0;
          start_angle = charlie.yaw;
         }
        if ((millis()-start_time) > timeout){
          //Serial.println ("Drive Distance timeout "); 
          this->STATUS = 3;return;//skip out of loop
        }//timeout reached
        time_old = time_new;
        time_new = millis();

        //find distance moved so far
        distance_moved = distance_moved + estimate_distance_since_last_update(time_old, time_new, (smooth_L_spd+smooth_R_spd)/2/1000 );//get the move distance
              
        if (abs(distance_moved) < abs(distance_to_move)){//if you have not reached destination keep going
          charlie.set_speed = desired_speed;
         //Serial.print ("Have driven ");
          //Serial.print (distance_moved);
          //Serial.print (" of ");
          //Serial.print (distance_to_move);
          //Serial.println ("inches");
          this->STATUS = 1;
          
        } else { //Serial.println ("Drive Distance Achieved "); 
          this->STATUS = 2;
        }//you've achieved the desired location so return success
        //this->STATUS = 4;//error
        
      }
    protected:
      long start_time, time_old, time_new;//time movement started
      double distance_moved =0;
      double start_angle =0;//starting angle for the movement (track this later)
      
};
class Set_Speed: public Node{//if not achieved in time then failure
    public:
        long timeout;//howlong the move has untilfailure
        double desired_speed;//how fast you would like to move
      void execute(){
        //stuff to do
        if (this->STATUS != 1 ){start_time = millis();}//capture start time if this is not currently running
        if ((millis()-start_time) > timeout){
          //Serial.println ("Speed timeout "); 
          this->STATUS = 3; return;
         }//timeout reached---------------------has to go to a shutdown node
        
        if ((round((smooth_R_spd+smooth_L_spd)/2*10)/10.0 != desired_speed) ){//if you have not reached speed rnd to 1 dp
          charlie.set_speed = desired_speed;
          //Serial.print ("Speed is ");
          //Serial.print ((smooth_R_spd+smooth_L_spd)/2);
          //Serial.print (" in/s and target is ");
          //Serial.print (desired_speed);
          //Serial.println (" in/s");
          this->STATUS = 1;//still going so return running
        } else { 
         // Serial.println ("Speed Achieved ");
          this->STATUS = 2; 
        }//you've achieved the desired speed so return success
        
        //this->STATUS = 4;
        
        
        //return 1;//running
        //return 2;//success
        //return 3;//fail
        //return 4;//error
      }
    protected:
      long start_time;//time movement started
      
};
class Turn: public Node{//if not achieved in time then failure<---check angle error for roll over and initiate it!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    public:
        long timeout;//howlong the move has untilfailure
        double desired_speed;//how fast you would like to move
        double angle_to_move;//HOW FAR TO TURN IN RADIANS
        double direction_to_move;//CW = 1 ccw = -1
        double angle_error_for_success;
        //double angle_error;
          //void init(){}//initialization loop, probably needs a flag to be set
      void execute(){
        //stuff to do
        if (this->STATUS != 1 ){//capture start time and intialize angle if this is not currently running
          start_time = millis(); 
           //angle_error = 0;
          charlie.set_speed = desired_speed;//update desired speed for PID<--I think this should not be here, I think speed and turning is separate.

          //<--this should be in update heading behaviour
          charlie.current_desired_heading = charlie.current_heading_est + (direction_to_move *angle_to_move);//update desired heading for PID, direction is neg for CCW and so subtracts!

          //correct for rollover/roll under angle
          if (charlie.current_desired_heading <0){
              charlie.current_desired_heading =charlie.current_desired_heading+2*PI;
          }else if(charlie.current_desired_heading > (2*PI)){
              charlie.current_desired_heading =charlie.current_desired_heading-2*PI;
          }
          
          
          //Serial.print ("Start Turning ");Serial.println (angle_to_move * 180/PI);
        }
        //angle_error = charlie.current_desired_heading-charlie.current_heading_est;//update current error use error value
       // Serial.print ("charlie heading Error ");Serial.println (charlie.heading_error * 180/PI);
        if ((millis()-start_time) > timeout){
          this->STATUS = 3; //fail
          //Serial.println ("Timeout on Turning "); 
          
        }//timeout reached
                
       //lets do an if angle error is less than thing here instead of angle moved
        else if (abs(charlie.heading_error) < angle_error_for_success && this->STATUS == 1){//if error is small enough send success, but only if you have gone through once                     
             //Serial.println ("Turn Completed "); 
             this->STATUS = 2;// return completed
        } else {          
             this->STATUS = 1;//you've NOT achieved the desired angle so return running
        }
        //this->STATUS = 4;//unexpected condition
        
        //return 1;//running
        //return 2;//success
        //return 3;//fail
        //return 4;//error
      }
    protected:
      long start_time;//time movement started
      double start_angle;
  
};

class Move_Servo: public Node {//moves the servo head to a postion, updates servo position based on time and speed.
  public:
    int servo_speed;
    int desired_position;//degrees from 0-180
    
    
    void execute(){
        //stuff to do
        if (this->STATUS != 1 ){//first loop
            last_time = millis(); 
            amount_moved = 0;

        }
        servoSetSpeed(servo0,servo_speed);//max min is 1-127 
        servoMove(servo0,desired_position/180*(charlie.servo_max-charlie.servo_min)+charlie.servo_min);//max min is 1000-5000
         //Serial.print ("servo move command: ");Serial.println (desired_position/180*(charlie.servo_max-charlie.servo_min)+charlie.servo_min);
                
        //confirm what direction we have to go to get to desired position
         if (desired_position >= charlie.servo_position){//we have to go ccw to get to where we want (increase angle)
              servo_direction = 1;
         } else {//have to go CW to get to where we want to go
              servo_direction = -1;
         }
        amount_moved = servo_speed*speed_const*(millis()-last_time)/1000;//always positive
        //Serial.print ("amount moved: " );Serial.println (amount_moved);
        charlie.servo_position = charlie.servo_position + amount_moved*servo_direction;
        //Serial.print ("charlie.servo_position: " );Serial.println (charlie.servo_position);
        //Serial.print ("servo desired_position: " );Serial.println (desired_position);
        
        //check to see if how far we moved from last time means we've past our target
        last_time = millis();
        if (charlie.servo_position >= (desired_position -amount_moved) and charlie.servo_position <= (desired_position +amount_moved)){//we are basically there
          
          charlie.servo_position = desired_position;//assume we get there
          this->STATUS = 2;//success
          //Serial.print ("servo pos achieved: " );
        } else{ //otherwise we are still going
          this->STATUS = 1; //running
          
        }
        
       
        //return 1;//running
        //return 2;//success
        //return 3;//fail
        //return 4;//error

    }
  protected:
    long last_time;
    long speed_const = 4.9;//multiply servo speed by this number to get speed in deg/sec
    int servo_direction;//+1 is right to left, -1 is left to right
    double amount_moved=0;
    
};


class UTAreClear: public Node{//checks that UT sensors are not reading a problem
    public:
      long trigger_dist = 4;//inches  <------this should be in the modifiable
        //void init(){}//initialization loop, probably needs a flag to be set
      void execute(){
        //stuff to do
        //check to see if we are getting into trouble
        if (frontUT.last_measurement_in_inches < trigger_dist){//something is too close
            digitalWrite(48, LOW);   // turn the LED off (HIGH is the voltage level)
           //Serial.println ("UT Block Detected ");
           this->STATUS = 3;//fail
        }else{ //return success if UT doesn't see anyhing
            digitalWrite(48, HIGH);   // turn the LED off (HIGH is the voltage level)
            this->STATUS = 2;//success
        }
        
        //return 1;//running
        // return 2;// success
        //return 3; //fail
        //return 4;//error
      }
};

class Populate_Grid: public Node{//populates a grid location with a distance or -1 if empty or not read
  public:

 
  void execute(){
     for( int i = 0; i < charlie.num_grids; i = i + 1 ) {
        this_grid_min = i*charlie.grid_size;
        this_grid_max = (i+1)*charlie.grid_size;
        //Serial.print("servo pos ");Serial.print(charlie.servo_position);Serial.print(" this_grd_min: ");Serial.print(this_grid_min);
         //Serial.print(" this_grd_max: ");Serial.print(this_grid_max);
        if(charlie.servo_position >=this_grid_min and charlie.servo_position < this_grid_max){
          //this is our current grid
            
            if (i ==4){
                  //Serial.print ("UT at 4 = "); Serial.print (scanUT.last_measurement_in_inches);Serial.print (" pos: 4 = "); Serial.print (charlie.ut_grid[4]); Serial.println();
            }
            if (i != current_grid){//we have switched grids so update the last grid
              //Serial.print("grid switch");
              charlie.ut_grid[current_grid] = current_grid_min_distance;//populate the grid location with the nearest object
              current_grid_min_distance = 50;//reset the grid first time through
              current_grid = i;//update the current grid              
            }
            
              
         }
        if (scanUT.last_measurement_in_inches < current_grid_min_distance){//this scan detected something closer
              current_grid_min_distance = scanUT.last_measurement_in_inches;//set flag so we don't set to zero later
              
        } 
     
        
     }
     this->STATUS = 2;//success
     
  }
  
  private:
    int current_grid = 0;
    long current_grid_min_distance = 0;
    long this_grid_min =0;
    long this_grid_max =180;
};

//setup behaviour tree nodes ---------------------------------------
  UpdateOrientation updateorientation; //makes an update orientation node
  ReadUT readfrontUT;//1001 read UT sensor node
  Check_Tilt checktilt;//check to make sure tilts are OK
  Drive_Distance backup_a_bit; //----------------I need these to work!!!!!!!!
  Set_Speed stop_moving; //1005
  Turn turn_90;//1006
  Drive_Distance carpet_sweep;//1007
  Drive_Distance half_charlie_dist;//1008
  Turn turn_90_ccw;//1009
  Move_Servo cw_servo_sweep;// 1010
  Move_Servo ccw_servo_sweep;//1011
  ReadUT readsweepUT;//1012 read UT sensor node
  Populate_Grid populate_grid;//1013
//----------------------------------------------------------

//setup rest of behaviour tree nodes ---------------------------------------
UTAreClear UTareclear;//1003 checks that UT sensors are clear from obstacles has to be down here since UTAreClear depends on UT being read...
//----------------------------------------------------------
class Selector: public Node{//group of child nodes
  //executres each child sequenctially left to right
  //returns success if ANY (or the required number of...) node(s) returns success
  //returns running if ANY nodes are running
   //_attached_nodes  is a variable with the name of the functions in it
   //nodes are passed in by the Selector.attachnode function name call
   public:
   //read in child nodes as integers
      void Attach_Node(int node_to_attach){
        _attached_nodes[current_attached_index] = node_to_attach;
        current_attached_index++; 
      }
      void SkipToRunning (){//makes this selector always skip to a running node, otherwise it starts from scratch every tick
        _skiptorunning =1;
      }
      void AlwaysSuccess (){//makes this selector always skip to a running node, otherwise it starts from scratch every tick
        _alwayssuccess =true;
      }
      void execute(){//runs when ticked
      
        for (int i=0;i<current_attached_index;i++){//increment through each node, run and report status
            //Serial.print(" looking at node: "); Serial.print(_attached_nodes[i]);
            if (this->STATUS == 1 && _skiptorunning == 1){
              i=curr_running;//if something is running, skip to it  <---------bad!!!!!! need to only do this when success
            }
            this->STATUS = FunctionRunTable (_attached_nodes[i]);//run this node and get its status
            //Serial.print(" status is: ");Serial.println(this->STATUS);
            if (this->STATUS == 3) {
                 //_node has failured
                //so continue to next node
            }else if (this->STATUS ==1){
                //_node is currently running, so status is running unless something fails
                //and since it is still running you can't look at the next node, so set to running and leave
                curr_running = i;
                this->STATUS = 1;//RUNNING leaves the status as 1 EVEN IF SET TO ALWAYS RETURN SUCCESS!!!!!
                //Serial.println(" set sequence to RUNNING ");
                break;
            }else if(this->STATUS ==2){//this task has succeeded so sequence stops right away
                this->STATUS = 2;//success
                //Serial.println(" set sequence to SUCCESS ");
                break;
            }
          
        } 
        //if we got to the end of this loop status is either 1 or 2 (exit criteria) or if it hasn't been set, should be 3 if all failed
        if (_alwayssuccess==true){this->STATUS=2;}//return success if set to always return success
        
        
        
      }
    protected:
      int curr_running;//if a node is running, this logs it
      int  _attached_nodes [10];
      int current_attached_index=0;
      int _skiptorunning = 0;
      boolean _alwayssuccess = false;
};
class Sequence : public Node {//this is a group of child nodes
  //executes each child sequentially left to right
  //returns success if ALL nodes return success, returns running if any child is running
  //_attached_nodes  is a variable with the name of the functions in it
  //node was passed in by the Sequence.Attachnode(function_name) call
  public:
      
      //read in the child nodes as ints...this could be a Vector but that is moving a lot of memory around
      
      void Attach_Node(int node_to_attach){
        _attached_nodes[current_attached_index] = node_to_attach;
        current_attached_index++; 
      }
      void SkipToRunning (){//makes this selector always skip to a running node, otherwise it starts from scratch every tick
        _skiptorunning =1;
      }
      void AlwaysSuccess (){//makes this selector always true
        _alwayssuccess =true;
      }
      // If ALL children succeed, the entire operation succeeds.  Failure results if any children fail.

      void execute(){//runs when ticked
    
        
        for (int i=0;i<current_attached_index;i++){//increment through each node, run and report status
            //Serial.print(" looking at node: "); Serial.println(_attached_nodes[i]);
            
            if (this->STATUS == 1 && _skiptorunning ==1){//<---currently doesn't handle skip to running and return success. could use curr_running to help
              i=curr_running;//if something is running, skip to it
            } 
            this->STATUS = FunctionRunTable (_attached_nodes[i]);//run this node and get its status
            //Serial.print(" status of: ");Serial.print(_attached_nodes[i]); Serial.print(" is: ");Serial.println(this->STATUS);
            if (this->STATUS == 2) {
                
                 //_node is a success
                //so continue to next node
            }else if (this->STATUS ==1){
                //_node is currently running, so status is running unless something fails
                //and since it is still running you can't look at the next node, so set to running and leave
                //even if set to always success you still want to test for running to you make sure the correct functions are run
                
                  curr_running = i;
                  this->STATUS = 1;
                  //Serial.print(" set running and leave ");
                  break;//leaves the status as 1
           

            }else {//this task has failed so sequence fails right away
                this->STATUS=3; 
                break;
            }//if you get here, status is 2
            
        }  
        
        if (_alwayssuccess==true){
           this->STATUS=2;
          
        }//return success if set to always return success
       // All children succeeded so the entire run() operation succeeds.
        
        
      }
   protected:
      int curr_running;//if a node is running, this logs it
      int  _attached_nodes [10];
      int current_attached_index=0;
      int _skiptorunning = 0;
      boolean _alwayssuccess = false;
  
      

};


//defines the selectors---------------------
  Sequence upperlevel_sequence;//5000
  Sequence get_data;//5001
  Sequence emergency_triggers;//5002
  Selector dont_be_stuck_selector;//5003
  Sequence back_up_and_turn_sequence;//5004
  Sequence follow_path;//5005
  Sequence UTsweep;//5006
  Sequence UTsweepSuccess;//5007
//-----------------------------------------------------------

int FunctionRunTable(int function_to_run){//send all function calls here so it can select and run them
  //for each function the return will be a status that is passed on to the higher level composite node
 if (function_to_run == 1000){
    updateorientation.execute();
    return updateorientation.report_status();//return status 
 } else if(function_to_run == 1001){
    readfrontUT.execute();
    return readfrontUT.report_status();//return status
 }  else if(function_to_run == 1002){
    checktilt.execute();
    return checktilt.report_status();//return status
 }  else if(function_to_run == 1003){
    UTareclear.execute();
    return UTareclear.report_status();//return status
 }   else if(function_to_run == 1004){
    backup_a_bit.execute();
    return backup_a_bit.report_status();//return status
 }   else if(function_to_run == 1005){
    stop_moving.execute();
    return stop_moving.report_status();//return status
 }   else if(function_to_run == 1006){
    turn_90.execute();
    return turn_90.report_status();//return status
 }  else if(function_to_run == 1007){
    carpet_sweep.execute();
    return carpet_sweep.report_status();//return status
 }   else if(function_to_run == 1008){
    half_charlie_dist.execute();
    return half_charlie_dist.report_status();//return status
 }  else if(function_to_run == 1009){
    turn_90_ccw.execute();
    return turn_90_ccw.report_status();//return status
 }  else if(function_to_run == 1010){
    cw_servo_sweep.execute();
    return cw_servo_sweep.report_status();//return status
 }  else if(function_to_run == 1011){
    ccw_servo_sweep.execute();
    return ccw_servo_sweep.report_status();//return status
 }  else if(function_to_run == 1012){
    readsweepUT.execute();
    return readsweepUT.report_status();//return status
 }  else if(function_to_run == 1013){
    populate_grid.execute();
    return populate_grid.report_status();//return status    
 }  else if(function_to_run == 1014){
    return charlie.check_grid_position(0,8);//return status
 }  else if(function_to_run == 1015){
    return charlie.check_grid_position(1,8);//return status
 }  else if(function_to_run == 1016){
    return charlie.check_grid_position(2,8);//return status
 }  else if(function_to_run == 1017){
    return charlie.check_grid_position(3,8);//return status
 }  else if(function_to_run == 1018){
    return charlie.check_grid_position(4,8);//return status
 }  else if(function_to_run == 1019){
    return charlie.check_grid_position(5,8);//return status
 }  else if(function_to_run == 1020){
    return charlie.check_grid_position(6,8);//return status
 }  else if(function_to_run == 1021){
    return charlie.check_grid_position(7,8);//return status
 }  else if(function_to_run == 1022){
    return charlie.check_grid_position(8,8);//return status
 }  else if(function_to_run == 1023){
    return charlie.check_grid_position(9,8);//return status   
      
 } else if(function_to_run == 5000){
     upperlevel_sequence.execute();
     return upperlevel_sequence.report_status(); //return status
 } else if(function_to_run == 5001){
     get_data.execute();
     return get_data.report_status(); //return status
 }  else if(function_to_run == 5002){
     emergency_triggers.execute();
     return emergency_triggers.report_status(); //return status
 }  else if(function_to_run == 5003){
     dont_be_stuck_selector.execute();
     return dont_be_stuck_selector.report_status(); //return status
 } else if(function_to_run == 5004){
     back_up_and_turn_sequence.execute();
     return back_up_and_turn_sequence.report_status(); //return status
 }  else if(function_to_run == 5005){
     follow_path.execute();
     return follow_path.report_status(); //return status
 } else if(function_to_run == 5006){
     UTsweep.execute(); //return status
     return UTsweep.report_status();
 } else if(function_to_run == 5007){
     UTsweepSuccess.execute(); //return status
     return UTsweepSuccess.report_status();
 }

 
}



