
#ifdef NOT_NEEDED
void  RobotMotor::servicePid(long encoderCount)
{
int pError;
int dError;
int pid; 
int output;


  const int Kp = 1; //PID proportional gain constant 
  const float Kd = 0; //0.5; //PID derivative gain constant
  const float Ki = 0.0; //PID integral gain constant

unsigned long timeDelta;

  if(!PID.isActive) {   
    if(getMotorPWM() != 0)
       debug_printf("setting PWM to 0\n");   
    stopMotor();     
    return;
  } 
  unsigned long timeNow = millis();
  if( timeNow - PID.startTime > PID.duration) {       
        debug_printf(" stopped- duration achieved\n"); 
        stopMotor(); 
        return;  
  } 
  if( isPidServiceNeeded() ) {     
     timeDelta = timeNow - PID.prevPidService;
     PID.prevPidService = timeNow;
   
    if( encoderCount != 0) {
       PID.prevPulseMillis = timeNow;
    }
    else if( timeNow - PID.prevPulseMillis > AUTO_STOP_INTERVAL) {       
        debug_printf("\nauto stop\n"); 
        stopMotor();   
    }  
    int targetTicks = map(timeDelta, 0, 1000, 0, PID.targetTicksPerSecond);
    pError = targetTicks - encoderCount;
    dError = (pError - PID.prevError); // derivative error
    pid = ((Kp*pError) + (Kd*dError) + (Ki*PID.iError)); //PID equation

    // acceleration limit
    if( pid > MAX_PWM_DELTA)
       pid = MAX_PWM_DELTA;
    else if (pid < -MAX_PWM_DELTA)
       pid = -MAX_PWM_DELTA;
    
    output = PID.prevOutput + pid;      
      
    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MAX_PWM)
      output = MAX_PWM;
    else if (output <= -MAX_PWM)
      output = -MAX_PWM;
   
    motorPwm = output;   
    setMotor();    
    if(pins[0] == 27)
   // debug_printf("%s svc after %dms, in=%2d, pErr=%2d, dErr=%2d, iErr=%2d, pid=%2d, out=%d (prev out=%d)\n", label, timeDelta, encoderCount, pError,dError, PID.iError, pid, output, PID.prevOutput);
    debug_printf("%s svc: in=%2d, pErr=%2d,(%s)  dErr=%2d, pid=%2d, out=%d (prev out=%d)\n", label, encoderCount, pError, (pError > 0 ? "slow" : "fast"), dError, pid, output, PID.prevOutput);

    
    PID.prevError = pError;
    PID.iError = PID.iError + pError; //add current error to integral error
    PID.prevOutput = output;
  } 
  else {
    debug_printf("!ERROR - PID service called too soon, check isPidServiceNeeded() method before calling to avoid losing encoder counts\n");
    delay(500);
  }

}
#endif