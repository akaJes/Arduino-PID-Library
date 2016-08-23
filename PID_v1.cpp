/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  PID::SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits
  PID::SetSampleTime(100);							//default Controller Sample Time is 0.1 seconds
  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(Kp, Ki, Kd);
  _initialized=false;
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID::Compute()
{
  if (!inAuto) return false;
  if (!_initialized)
	_initialize();
  unsigned long now = millis();
  if(now - lastTime < _setted_pid.interval) return false;
  lastTime = now;
  /*Compute all the working error variables*/
  double input = *myInput;
  double error = *mySetpoint - input;
  ITerm += (_working_pid.ki * error);
  _keep_range(&ITerm);

  double dInput = (input - lastInput);
  lastInput = input;

  /*Compute PID Output*/
  *myOutput = _working_pid.kp * error + ITerm - _working_pid.kd * dInput;
  _keep_range(myOutput);
  return true;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd)
{
  if (Kp<0 || Ki<0 || Kd<0) return;
  _setted_pid.kp=Kp;
  _setted_pid.ki=Ki;
  _setted_pid.kd=Kd;
  _initialized=false;
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime <= 0) return;
  _setted_pid.interval=NewSampleTime;
  _initialized=false;
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
  if(Min >= Max) return;
  _output_range.min = Min;
  _output_range.max = Max;

  if(inAuto){
	_keep_range(myOutput);
	_keep_range(&ITerm);
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
  if(inAuto = (Mode == AUTOMATIC))
	_initialized=false;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::_initialize()
{
  ITerm = *myOutput;
  _keep_range(&ITerm);
  lastInput = *myInput;
  lastTime = millis();

  _working_pid=_setted_pid;
  double grow = _working_pid.grow?1:-1;
  double SampleTimeInSec = (grow*_working_pid.interval)/1000;

  _working_pid.kp *= grow;
  _working_pid.ki *= SampleTimeInSec;
  _working_pid.kd /= SampleTimeInSec;
  _initialized=true;
}
void PID::_keep_range(double *val){
  if(*val > _output_range.max)
	*val = _output_range.max;
  else
	if(*val < _output_range.min)
	  *val = _output_range.min;
}
/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
  if(_setted_pid.grow!=Direction==DIRECT){
	_setted_pid.grow=Direction==DIRECT;
	_initialized=false;
  }
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  _setted_pid.kp; }
double PID::GetKi(){ return  _setted_pid.ki;}
double PID::GetKd(){ return  _setted_pid.kd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return _setted_pid.grow?DIRECT:REVERSE;}
