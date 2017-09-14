



// Arduino pin connected to Pololu Servo Controller
//const int servoController = 1;  //14??

// location of Servo plugged into Pololu Servo Controller (defaults 0-7)
const int servo0 = 0;//UT sweep sensor  
// example calls: servoMove(servo0,3000);
//                servoSetSpeed(servo0,127);

/*
    Move Servo using Pololu Servo Controller
    servo = servo number (default 0-7)
    hornPos = position (500-5500)  {Test your servos maximum positions}
*/
void servoMove(int servo, int pos)
{
  if( pos > 5500 || pos < 500 ) return;   

  // Build a Pololu Protocol Command Sequence 
  char cmd[6];   
  cmd[0] = 0x80;  // start byte   
  cmd[1] = 0x01;  // device id   
  cmd[2] = 0x04;  // command number   
  cmd[3] = servo; //servo number   
  cmd[4] = lowByte(pos >> 7);   // lower byte
  cmd[5] = lowByte(pos & 0x7f); // upper byte

  // Send the command to the Pololu Servo Controller
  for ( int i = 0; i < 6; i++ ){
    Serial3.write(cmd[i]);
  }

}

/*
*    Set Servo Speed using Pololu Servo Controller
*    servo = servo number (default 0-7)
*    speed = (1-127) (slowest - fastest) 0 to disable speed control
*/
void servoSetSpeed(int servo, int servo_speed){

   // Build a Pololu Protocol Command Sequence
   char cmd[5];
   cmd[0] = 0x80;     // start byte
   cmd[1] = 0x01;     // device id
   cmd[2] = 0x01;     // command number
   cmd[3] = lowByte(servo);    // servo number
   cmd[4] = lowByte(servo_speed);    // speed

  // Send the command to the Pololu Servo Controller
   for ( int i = 0; i < 5; i++ ){
      Serial3.write(cmd[i]);
   }
}
