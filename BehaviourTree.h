// Behaviour tree 

/*

      Root
             |
             |
  Selector (only one of these children need to succeed)  
       /             \
      /               \
     /                 \
Door is open?      Sequence (all of these children need to succeed)
(if door is                /           \
already open,             /             \
we are done)             /               \
                   Approach door      Open the door
                  (if this fails
                  then the door
                  cannot be opened)
*/
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
        update_positional_awareness();
        //return 1;//running
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
        Serial.println(RangeInInches); //output UT position
        //return 1;//running
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
      if (pitch > MAX_PITCH or pitch <(-1*MAX_PITCH) ){
           //asdf
      }else if (roll >MAX_ROLL or roll<(-1*MAX_ROLL)){
          //asdf
      }
        //return 1;//running
        return 2;//success
        //return 3;//fail
        //return 4;//error
      }
};


//setup behaviour tree nodes ---------------------------------------
  UpdateOrientation updateorientation; //makes an update orientation node
  ReadUT readUT;//read UT sensor node
  Check_Tilt checktilt;//check to make sure tilts are OK
  
//----------------------------------------------------------

class UTAreClear: public Node{//checks that UT sensors are not reading a problem
    public:
        
        //void init(){}//initialization loop, probably needs a flag to be set
      int execute(){
        //stuff to do
        //check to see if we are getting into trouble
      if (readUT.RangeInInches < 8){//something is too close
           return 3;//fail
      }else{ //return success if UT doesn't see anyhing
            return 2;//success
      }
        //return 1;//running
        //return 4;//error
      }
};
//setup rest of behaviour tree nodes ---------------------------------------
UTAreClear UTareclear;//checks that UT sensors are clear from obstacles has to be down here since UTAreClear depends on UT being read...
//----------------------------------------------------------

class Sequence : public Node {//this is a group of child nodes
  //executes each child sequentially left to right
  //returns success if ALL nodes return success, returns running if any child is running
  //_node  is a variable with the name of the functions in it
  //node was passed in by the Sequence.Attachnode(function_name) call
  public:
      
      //read in the child nodes as strings...this could be a Vector (include Vector)-----*******************
      //<------need to finding which child is 'running' and skipping straight to it some how
      
      void Attach_Node(int node_to_attach){
        _attached_nodes[current_attached_index] = node_to_attach;
        current_attached_index++; 
      }

      // If ALL children succeed, the entire operation succeeds.  Failure results if any children fail.

      int execute(){//runs when ticked
        
        for (int i=0;i<current_attached_index;i++){//increment through each node, run and report status
            if (this->STATUS == 1){i=curr_running;}//if something is running, skip to it
            
             this->STATUS = FunctionRunTable (_attached_nodes[i]);//run this node and get its status
            if (this->STATUS == 2) {
                 //_node is a success
                //so continue to next node
            }else if (this->STATUS ==1){
                //_node is currently running, so status is running unless something fails
                //and since it is still running you can't look at the next node, so set to running and leave
                curr_running = i;
                return 1;//leaves the status as 1
            }else {//this task has failed so sequence fails right away
                this->STATUS=3; 
                return 3;
            }
          
        }    
        return 2;  // All children succeeded so the entire run() operation succeeds.
      }
   protected:
      int curr_running;//if a node is running, this logs it
      int  _attached_nodes [10];
      int current_attached_index=0;

};


//defines the selectors---------------------
  Sequence upperlevel_sequence;//5000
  Sequence get_data;//5001
  Sequence emergency_triggers;//5002
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
 }   else if(function_to_run == 5000){
     return upperlevel_sequence.execute(); //return status
 } else if(function_to_run == 5001){
     return get_data.execute(); //return status
 }  else if(function_to_run == 5002){
     return emergency_triggers.execute(); //return status
 }
}

/*
class Sequence : public CompositeNode {
  public:
    virtual bool run() override {
      for (Node* child : getChildren()) {  // The generic Sequence implementation.
        if (!child->run())  // If one child fails, then enter operation run() fails.  Success only results if all children succeed.
          return false;
      }
      return true;  // All children suceeded, so the entire run() operation succeeds.
    }
};

struct DoorStatus {
  bool doorIsOpen;
  int distanceToDoor;
};

class CheckIfDoorIsOpenTask : public Node {  // Each task will be a class (derived from Node of course).
  private:
    DoorStatus* status;
  public:
    CheckIfDoorIsOpenTask (DoorStatus* status) : status(status) {}
    virtual bool run() override {
      if (status->doorIsOpen == true)
        std::cout << "The person sees that the door is open." << std::endl;  // will return true
      else
        std::cout << "The person sees that the door is closed." << std::endl;  // will return false
      return status->doorIsOpen;
    }
};

class ApproachDoorTask : public Node {
  private:
    DoorStatus* status;
    bool obstructed;
  public:
    ApproachDoorTask (DoorStatus* status, bool obstructed) : status(status), obstructed(obstructed) {}
    virtual bool run() override {
      if (obstructed)
        return false;
      if (status->distanceToDoor > 0) {
        std::cout << "The person approaches the door." << std::endl;
        status->distanceToDoor--;  // thus run() is not a const function
        if (status->distanceToDoor > 1)
          std::cout << "The person is now " << status->distanceToDoor << " meters from the door." << std::endl;
        else if (status->distanceToDoor == 1)
          std::cout << "The person is now only one meter away from the door." << std::endl;
        else
          std::cout << "The person is at the door." << std::endl;
      }
      return true;
    }
};

class OpenDoorTask : public Node {
  private:
    DoorStatus* status;
  public:
    OpenDoorTask (DoorStatus* status) : status(status) {}
    virtual bool run() override {
      if (status->distanceToDoor > 0) {
        std::cout << "The person is still too far away from the door.  He cannot open the door." << std::endl;
        return false; 
      }
      status->doorIsOpen = true;  // run() not const because of this too
      std::cout << "The person opens the door." << std::endl;
      return true;
    }
};

int main() {
  Sequence *root = new Sequence, *sequence1 = new Sequence;  // Note that root can be either a Sequence or a Selector, since it has only one child.
  Selector* selector1 = new Selector;  // In general there will be several nodes that are Sequence or Selector, so they should be suffixed by an integer to distinguish between them.
  DoorStatus* doorStatus = new DoorStatus {false, 5};  // The door is initially closed and 5 meters away.
  CheckIfDoorIsOpenTask* checkOpen = new CheckIfDoorIsOpenTask (doorStatus);
  ApproachDoorTask* approach = new ApproachDoorTask (doorStatus, false);
  OpenDoorTask* open = new OpenDoorTask (doorStatus);
  
  root->addChild (selector1);
  
  selector1->addChild (checkOpen);
  selector1->addChild (sequence1);
  
  sequence1->addChild (approach);
  sequence1->addChild (open);
  
  while (!root->run())  // If the operation starting from the root fails, keep trying until it succeeds.
    std::cout << "--------------------" << std::endl;
  std::cout << std::endl << "Operation complete.  Behaviour tree exited." << std::endl;
  std::cin.get();
}

*/


