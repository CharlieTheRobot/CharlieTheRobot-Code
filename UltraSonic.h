/*
Handles all of the functions and classes required to get information from the UT sensors

*/


//from SEEED
class Ultrasonic
{
    public:
      long duration;// the Pulse time received;
      Ultrasonic(int pin, int echo);
      void DistanceMeasure(void);
      long microsecondsToCentimeters(void);
      long microsecondsToInches(void);
      long last_measurement_in_inches;
    
    private:
      int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
      
      int _echo_pin;//echo pin, could be same as pin
      boolean fail_flag =0;
};

Ultrasonic::Ultrasonic(int pin, int echo_pin)
{
    _pin = pin;
    _echo_pin = echo_pin;
}
/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
    pinMode(_pin, OUTPUT);//set trigger pin to output
    
    
    digitalWrite(_pin, LOW);//drive low to get clean signal
    delayMicroseconds(2);//pause2
    digitalWrite(_pin, HIGH);//drive high to start ping
    delayMicroseconds(7);//wait for response was 5
    digitalWrite(_pin,LOW);//drive low to listen for SEED style sensors
    
    pinMode(_echo_pin,INPUT);//set echo pin to input, this may or may not be the same pin
    duration = pulseIn(_echo_pin,HIGH);//figure out what the echo pin duration is to calc time
    if (duration > 5000 or fail_flag ==1){
          if (fail_flag ==1){fail_flag =0;duration = 5000;}
          else if (duration > 5000){duration = 5000;fail_flag = 1;}
    
    }//clamp output
    
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
    return duration/29/2;
}
/*The measured distance from the range 0 to 157 Inches*/
long Ultrasonic::microsecondsToInches(void)
{
    return duration/74/2;
}


