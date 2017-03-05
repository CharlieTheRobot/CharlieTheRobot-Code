// Behaviour tree 

//pre-defines functions used here---------------------
int FunctionRunTable(int function_to_run);//predefine
//---------------------------------------------------

class Node {  // This class represents each node in the behaviour tree.
  //put everything in here that is common to all or more nodes
  public:
    int STATUS = 0 ;//0 = (default) not running, 1 = running, 2 =  success, 3 = fail, 4 = error//should figure out how to take advantage of this->..***********
};

class UpdateOrientation: public Node{//always returns success unless there is an unhandled error!
  public:
    int execute(){
      //stuff to do
        charlie.update_positional_awareness();
        //return 1;//running
        Serial.println ("update orientation success");
        this->STATUS = 2;
        return 2;//success
        //return 3;//fail
        //return 4;//error
     }
};
class ReadUT: public Node{//always returns success
    public:
        long RangeInInches=-1;
        //void init(){}//initialization loop, probably needs a flag to be set
      int execute(){
        //stuff to do

        frontUT.DistanceMeasure();// get the current signal time;
        RangeInInches = frontUT.microsecondsToInches();//convert the time to inches;
        Serial.print ("UT Range");
        Serial.println(RangeInInches); //output UT position
        //return 1;//running
        this->STATUS = 2;
        return 2;//success
        //return 3;//fail
        //return 4;//error
      }
};
class Check_Tilt: public Node{//always returns success
    public:
        
        //void init(){}//initialization loop, probably needs a flag to be set
      int execute(){
        //stuff to do
        //check to see if we are getting into trouble
      if (charlie.pitch > charlie.MAX_PITCH or charlie.pitch <(-1*charlie.MAX_PITCH) ){
          Serial.println ("Max Pitch Exceeded");
          this->STATUS = 3;
           return 3;//fail
      }else if (charlie.roll >charlie.MAX_ROLL or charlie.roll<(-1*charlie.MAX_ROLL)){
          Serial.println ("Max Roll Exceeded");
          this->STATUS = 3;
          return 3;//fail
      }
        Serial.println ("Pitch and Roll are OK");
        //return 1;//running
        this->STATUS = 2;
        return 2;//success
        //return 3;//fail
        //return 4;//error
      }
};
class Drive_Distance: public Node{//if not achieved in time then failure
    public:
        long timeout;//howlong the move has untilfailure
        double distance_to_move;//how far in inches you want to move
        double desired_speed;//how fast you would like to move
        
        //void init(){}//initialization loop, probably needs a flag to be set
      int execute(){
        //stuff to do
        if (this->STATUS != 1 ){//capture start time and intialize distance if this is not currently running
          Serial.println ("Start Moving ");
          start_time = millis(); 
          time_new = millis(); 
          distance_moved = 0;
          start_angle = charlie.yaw;
         }
        if ((millis()-start_time) > timeout){Serial.println ("Drive Distance timeout "); this->STATUS = 3; return 3;}//timeout reached
        time_old = time_new;
        time_new = millis();

        //find distance moved so far
        distance_moved = distance_moved + estimate_distance_since_last_update(time_old, time_new, (smooth_L_spd+smooth_R_spd)/2/1000 );//get the move distance

        angle_error = charlie.yaw - start_angle;//yaw should always be a positive angle from 0 2pi(360), possitive angle error means need to turn left to get on track
        if (angle_error < 0) angle_error = angle_error + 2 * M_PI;//if angle is less  than zero it has wrapped
        
        
        if (abs(distance_moved) < abs(distance_to_move)){//if you have not reached destination keep going
          setspeed_PID(desired_speed,desired_speed, angle_error);//set speed to desired
          Serial.print ("Have driven ");
          Serial.print (distance_moved);
          Serial.print (" of ");
          Serial.print (distance_to_move);
          Serial.println ("inches");
          this->STATUS = 1;
          return 1;//still going so return running
        } else { Serial.println ("Drive Distance Achieved "); this->STATUS = 2; return 2;}//you've achieved the desired location so return success
        this->STATUS = 4;
        return 4;//unexpected condition
        
        //return 1;//running
        //return 2;//success
        //return 3;//fail
        //return 4;//error
      }
    protected:
      long start_time, time_old, time_new;//time movement started
      double distance_moved =0;
      double start_angle =0;//starting angle for the movement (track this later)
      double angle_error = 0;
};
class Set_Speed: public Node{//if not achieved in time then failure
    public:
        long timeout;//howlong the move has untilfailure
        double desired_speed;//how fast you would like to move
      int execute(){
        //stuff to do
        if (this->STATUS != 1 ){start_time = millis();}//capture start time if this is not currently running
        if ((millis()-start_time) > timeout){Serial.println ("Speed timeout "); this->STATUS = 3; return 3;}//timeout reached---------------------has to go to a shutdown node
        
        if ((round((smooth_R_spd+smooth_L_spd)/2*10)/10.0 != desired_speed) ){//if you have not reached speed
          setspeed_PID(desired_speed,desired_speed);//set speed to desired
          Serial.print ("Speed is ");
          Serial.print ((smooth_R_spd+smooth_L_spd)/2);
          Serial.print (" in/s and target is ");
          Serial.print (desired_speed);
          Serial.println (" in/s");
          this->STATUS = 1;
          return 1;//still going so return running
        } else { Serial.println ("Speed Achieved ");this->STATUS = 2; return 2;}//you've achieved the desired speed so return success
        this->STATUS = 4;
        return 4;//unexpected condition
        
        //return 1;//running
        //return 2;//success
        //return 3;//fail
        //return 4;//error
      }
    protected:
      long start_time;//time movement started
      
};
class Turn: public Node{//if not achieved in time then failure
    public:
        long timeout;//howlong the move has untilfailure
        double desired_speed;//how fast you would like to move
        double angle_to_move;//HOW FAR TO TURN IN RADIANS
        double direction_to_move;
          //void init(){}//initialization loop, probably needs a flag to be set
      int execute(){
        //stuff to do
        if (this->STATUS != 1 ){start_time = millis(); start_angle = charlie.yaw; angle_moved = 0; Serial.print ("Start Turning ");Serial.println (angle_to_move * 180/PI);}//capture start time and intialize angle if this is not currently running
        if ((millis()-start_time) > timeout){this->STATUS = 3; Serial.println ("Timeout on Turning "); return 3;}//timeout reached
        angle_moved = angle_moved + charlie.gyro_angle_change;
       
        if (abs(angle_moved) < angle_to_move){//if you have not reached destination keep going
          if(direction_to_move == 1){//right
              setspeed_PID(-desired_speed,desired_speed);//set speed to desired
              Serial.println("CW turn ongoing");
              this->STATUS = 1;
              return 1;//still going so return running
          }else{
              setspeed_PID(desired_speed,-desired_speed);//set speed to desired
              this->STATUS = 1;
              return 1;//still going so return running
          }
        } else { this->STATUS = 2; return 2;}//you've achieved the desired angle so return success
        this->STATUS = 4;
        return 4;//unexpected condition
        
        //return 1;//running
        //return 2;//success
        //return 3;//fail
        //return 4;//error
      }
    protected:
      long start_time;//time movement started
      double angle_moved;
      double start_angle;
  
};
//setup behaviour tree nodes ---------------------------------------
  UpdateOrientation updateorientation; //makes an update orientation node
  ReadUT readUT;//1001 read UT sensor node
  Check_Tilt checktilt;//check to make sure tilts are OK
  Drive_Distance backup_a_bit; //----------------I need these to work!!!!!!!!
  Set_Speed stop_moving; //1005
  Turn turn_90;
  Drive_Distance carpet_sweep;//1007
  Drive_Distance half_charlie_dist;//1008
  Turn turn_90_ccw;//1009
//----------------------------------------------------------

class UTAreClear: public Node{//checks that UT sensors are not reading a problem
    public:
        
        //void init(){}//initialization loop, probably needs a flag to be set
      int execute(){
        //stuff to do
        //check to see if we are getting into trouble
      if (readUT.RangeInInches < 8){//something is too close
           Serial.println ("UT Block Detected ");
           this->STATUS = 3;
           return 3;//fail
      }else{ //return success if UT doesn't see anyhing
            Serial.println ("NO UT Block Detected ");
            this->STATUS = 2;
            return 2;//success
      }
        //return 1;//running
        //return 4;//error
      }
};
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
      int execute(){//runs when ticked
        for (int i=0;i<current_attached_index;i++){//increment through each node, run and report status
            if (this->STATUS == 1 && _skiptorunning == 1){i=curr_running;}//if something is running, skip to it  <---------bad!!!!!! need to only do this when sucess
            
            this->STATUS = FunctionRunTable (_attached_nodes[i]);//run this node and get its status
            if (this->STATUS == 3) {
                 //_node has failured
                //so continue to next node
            }else if (this->STATUS ==1){
                //_node is currently running, so status is running unless something fails
                //and since it is still running you can't look at the next node, so set to running and leave
                curr_running = i;
                this->STATUS = 1;
                return 1;//leaves the status as 1
            }else if(this->STATUS ==2){//this task has succeeded so sequence stops right away
                this->STATUS = 2;
                return 2;
            }
          
        } 
        this->STATUS = 3;   
        return 3;  // All children failed so the entire run() operation fails.
      }
    protected:
      int curr_running;//if a node is running, this logs it
      int  _attached_nodes [10];
      int current_attached_index=0;
      int _skiptorunning = 0;
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
      // If ALL children succeed, the entire operation succeeds.  Failure results if any children fail.

      int execute(){//runs when ticked
        
        for (int i=0;i<current_attached_index;i++){//increment through each node, run and report status
            if (this->STATUS == 1 && _skiptorunning ==1){i=curr_running;}//if something is running, skip to it
            
             this->STATUS = FunctionRunTable (_attached_nodes[i]);//run this node and get its status
            if (this->STATUS == 2) {
                 //_node is a success
                //so continue to next node
            }else if (this->STATUS ==1){
                //_node is currently running, so status is running unless something fails
                //and since it is still running you can't look at the next node, so set to running and leave
                curr_running = i;
                this->STATUS = 1;
                return 1;//leaves the status as 1
            }else {//this task has failed so sequence fails right away
                this->STATUS=3; 
                return 3;
            }
          
        }  
        this->STATUS = 2;  
        return 2;  // All children succeeded so the entire run() operation succeeds.
      }
   protected:
      int curr_running;//if a node is running, this logs it
      int  _attached_nodes [10];
      int current_attached_index=0;
      int _skiptorunning = 0;

};


//defines the selectors---------------------
  Sequence upperlevel_sequence;//5000
  Sequence get_data;//5001
  Sequence emergency_triggers;//5002
  Selector dont_be_stuck_selector;//5003
  Sequence back_up_and_turn_sequence;//5004
  Sequence follow_path;//5005
//-----------------------------------------------------------

int FunctionRunTable(int function_to_run){//send all function calls here so it can select and run them
  //for each function the return will be a status that is passed on to the higher level composite node
 if (function_to_run == 1000){
    return updateorientation.execute();//return status
       
 } else if(function_to_run == 1001){
    return readUT.execute();//return status
 }  else if(function_to_run == 1002){
    return checktilt.execute();//return status
 }  else if(function_to_run == 1003){
    return UTareclear.execute();//return status
 }   else if(function_to_run == 1004){
    return backup_a_bit.execute();//return status
 }   else if(function_to_run == 1005){
    return stop_moving.execute();//return status
 }   else if(function_to_run == 1006){
    return turn_90.execute();//return status
 }      else if(function_to_run == 1007){
    return carpet_sweep.execute();//return status
 }   else if(function_to_run == 1008){
    return half_charlie_dist.execute();//return status
 }      else if(function_to_run == 1009){
    return turn_90_ccw.execute();//return status
 }else if(function_to_run == 5000){
     return upperlevel_sequence.execute(); //return status
 } else if(function_to_run == 5001){
     return get_data.execute(); //return status
 }  else if(function_to_run == 5002){
     return emergency_triggers.execute(); //return status
 }  else if(function_to_run == 5003){
     return dont_be_stuck_selector.execute(); //return status
 } else if(function_to_run == 5004){
     return back_up_and_turn_sequence.execute(); //return status
 }  else if(function_to_run == 5005){
     return follow_path.execute(); //return status
 }
}



