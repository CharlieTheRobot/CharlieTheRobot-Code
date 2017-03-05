/*
 * Motor control module needs can use 99% for PWM max, can't use 100% :(
 * if 255 is max value for analogWrite, then 99% is about 252.45 -> 252 max!
 *if value 255 is set, then DC engine cogs ...
 * max tested is 254 without cogging 
 */

double Ltarget_last = 0;
double Rtarget_last = 0;
double angle_error_last =0;

void setspeed_PID(double Rtarget, double Ltarget, double angle_error = 0)
{
  CaptureBothWheelSpeeds();
  double PWM_Ratio = 1;
  double PWM_R = 0;
  double PWM_L = 0;
  double delta_PWM_theta = 0;
  
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastPIDTime);
  
   /*Compute all the working error variables*/
   double errorR = Rtarget - smooth_R_spd;
   double errorL = Ltarget - smooth_L_spd;

   if (Ltarget != Ltarget_last){//reset integral error if speed setpoint has changed
    PIDerrSumL = 0;
    PIDerrSumTheta = 0;
   }
   if (Rtarget != Rtarget_last){
    PIDerrSumR = 0;
    PIDerrSumTheta = 0;
   }

   
   PIDerrSumR += (errorR * timeChange);
   PIDerrSumL += (errorL * timeChange);
   PIDerrSumTheta +=(angle_error * timeChange);//error from heading angle
   
   double dErrR = (errorR - PIDlastErrR) / timeChange;
   double dErrL = (errorL - PIDlastErrL) / timeChange;
   double dErrTheta =(angle_error-PIDlastErrTheta)/timeChange;

   //clamp integral errors
   if (PIDerrSumR > 50000) PIDerrSumR = 50000;//clamp integral
   if (PIDerrSumL > 50000) PIDerrSumL = 50000;//clamp integral
   if (PIDerrSumR < -50000) PIDerrSumR = -50000;//clamp integral
   if (PIDerrSumL < -50000) PIDerrSumL = -50000;//clamp integral
   if (PIDerrSumTheta > 50000) PIDerrSumTheta = 50000;//clamp integral
   if (PIDerrSumTheta < -50000) PIDerrSumTheta = -50000;//clamp integral
   
   /*Compute PID Output*/
   PWM_R = (kp * errorR + ki * PIDerrSumR + kd * dErrR)/PWM_Ratio;
   PWM_L = (kp * errorL + ki * PIDerrSumL + kd * dErrL)/PWM_Ratio;
   delta_PWM_theta = (kp * angle_error + ki * PIDerrSumTheta + kd * dErrTheta)/PWM_Ratio;
   
   //adjust to make sure we are going straight.
   if (angle_error > 0){PWM_R = PWM_R - delta_PWM_theta;} else if (angle_error < 0){PWM_L = PWM_L - delta_PWM_theta;}
  
   if (PWM_R > MAX_motor_spd) PWM_R = MAX_motor_spd;
   if (PWM_R < - MAX_motor_spd) PWM_R = -MAX_motor_spd;

   if (PWM_L > MAX_motor_spd) PWM_L = MAX_motor_spd;
   if (PWM_L < - MAX_motor_spd) PWM_L = -MAX_motor_spd;

    Serial.print("PWR Right = "); Serial.print(PWM_R); Serial.print("PIDerrSumR Right = "); Serial.print(PIDerrSumR);
    Serial.print(" PWR RLEft = "); Serial.print(PWM_L);Serial.print("PIDerrSumL Right = "); Serial.println(PIDerrSumL);
   //now make the new speed the intermediate speed
   if(Rtarget ==0){//stop immediately
      analogWrite(RHS_CCW,LOW);
      analogWrite(RHS_CW,LOW);
      PIDerrSumR=0;
      PIDlastErrR = 0;
      errorR = 0;
    }else if (PWM_R >= 0){
      analogWrite(RHS_CCW,LOW);
      analogWrite(RHS_CW,abs(PWM_R));
   }else{
      analogWrite(RHS_CW,LOW);
      analogWrite(RHS_CCW,abs(PWM_R));
   }
   if(Ltarget ==0){//stop immediately
      analogWrite(LHS_CCW,LOW);
      analogWrite(LHS_CW,LOW);
      PIDerrSumL=0;
      PIDlastErrL = 0;
      errorL = 0;
    } else if (PWM_L >= 0){
      analogWrite(LHS_CCW,LOW);
      analogWrite(LHS_CW,abs(PWM_L));
   }else{
      analogWrite(LHS_CW,LOW);
      analogWrite(LHS_CCW,abs(PWM_L));
   }
   
   /*Remember some variables for next time*/
   PIDlastErrR = errorR;
   PIDlastErrL = errorL;
   PIDlastErrTheta = angle_error;
   lastPIDTime = now;
   
    Ltarget_last = Ltarget;
    Rtarget_last = Rtarget;
       
    }
