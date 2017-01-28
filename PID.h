void setspeed_PID(double Rtarget, double Ltarget)
{
    CaptureBothWheelSpeeds();
  double PWM_Ratio = 1;
  double PWM_R = 0;
  double PWM_L = 0;
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastPIDTime);
  
   /*Compute all the working error variables*/
   double errorR = Rtarget - smooth_R_spd;
   double errorL = Ltarget - smooth_L_spd;
   PIDerrSumR += (errorR * timeChange);
   PIDerrSumL += (errorL * timeChange);
   double dErrR = (errorR - PIDlastErrR) / timeChange;
   double dErrL = (errorL - PIDlastErrL) / timeChange;
   if (PIDerrSumR > 50000) PIDerrSumR = 50000;//clamp integral
   if (PIDerrSumL > 50000) PIDerrSumL = 50000;//clamp integral
      if (PIDerrSumR < -50000) PIDerrSumR = -50000;//clamp integral
   if (PIDerrSumL < -50000) PIDerrSumL = -50000;//clamp integral
  
   /*Compute PID Output*/
   PWM_R = (kp * errorR + ki * PIDerrSumR + kd * dErrR)/PWM_Ratio;
   PWM_L = (kp * errorL + ki * PIDerrSumL + kd * dErrL)/PWM_Ratio;
    Serial.println();
    Serial.print (ki * PIDerrSumR);
    Serial.print ("  ");
    Serial.println (ki * PIDerrSumL);
   if (PWM_R > MAX_motor_spd) PWM_R = MAX_motor_spd;
   if (PWM_R < - MAX_motor_spd) PWM_R = -MAX_motor_spd;

   if (PWM_L > MAX_motor_spd) PWM_L = MAX_motor_spd;
   if (PWM_L < - MAX_motor_spd) PWM_L = -MAX_motor_spd;


   //now make the new speed the intermediate speed
   if(Rtarget ==0){//stop immediately
      analogWrite(RHS_CCW,LOW);
      analogWrite(RHS_CW,LOW);
      PIDerrSumR=0;
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
   lastPIDTime = now;
}
