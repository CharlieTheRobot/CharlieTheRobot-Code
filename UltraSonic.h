/*
Handles all of the functions and classes required to get information from the UT sensors

*/


//from SEEED
class Ultrasonic
{
    public:
      Ultrasonic(int pin);
      void DistanceMeasure(void);
      long microsecondsToCentimeters(void);
      long microsecondsToInches(void);
    
    private:
      int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
      long duration;// the Pulse time received;
};

Ultrasonic::Ultrasonic(int pin)
{
    _pin = pin;
}
/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_pin,LOW);
    pinMode(_pin,INPUT);
    duration = pulseIn(_pin,HIGH);
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


