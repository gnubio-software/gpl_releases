/*********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 *
 * This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>
 *********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID.h"


/* Approximated log function ***********************************************/
int32_t log_approx(int32_t error) {
    if (error > 0) {
        error = 256 + ((error - 256) >> 3);
    } else {
        error = -256 - ((error + 256) >> 3);
    }
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(uint16_t* Input, uint16_t* Output, uint16_t* Setpoint,
        uint16_t Kp, uint16_t Ki, uint16_t Kd, uint8_t Multiplier, int ControllerDirection)
{
    
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    multiplier = Multiplier;
    PID::SetOutputLimits(0, 255); //default output limit corresponds to 
                                  //the arduino pwm limits

    SampleTime = 1;             //default Controller Sample Time is 1 milliseconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);
    
    ITerm = 0;

    lastTime = millis()-SampleTime;             
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);

   if(timeChange >= SampleTime)
   {
      /*Compute all the working error variables*/
      uint16_t input = *myInput;
      uint16_t setpoint = *mySetpoint;
      int32_t error = ((int32_t)(setpoint) - (int32_t)(input));
      int32_t log_error;
      int32_t PTerm;
      int32_t output;

      if (error < 256 || error > -256) {
          // Unshifted error & coef
          ITerm += (double)(ki * error * multiplier) / (double) 512000.0;
          PTerm = ((int32_t)(kp) * error) >> 9;
      } else if (error < 2048 || error > -2048) {
          // shifted 3 bits in total on error & coef
          log_error = log_approx(error) >> 3;
          error = error >> 3;
          ITerm += (double)(ki * error * multiplier) / (double) 64000.0;
          PTerm = ((int32_t)(kp) * error) >> 6;
      } else {
          // shifted 6 bits in total on error & coef
          log_error = log_approx(error) >> 6;
          error = error >> 6;
          ITerm += (double)(ki * error * multiplier) / (double) 8000.0;
          PTerm = ((int32_t)(kp) * error) >> 3;
      }

      /* At this moment, both ITerm and PTerm are scaled into 10-bit resolution
       * for OCR1B and OCR2B.
       * Both ITerm and PTerm are supposed to run in 10-bit space, which is the
       * PWM resolution we have.
       *
       * ************************************************************************
       * Safety Clamps:
       * 
       * 1. Lock ITerm when pressure drop is larger than 2psi (~1160 transducer counts)
       *    ITerm = half of 10-bit = 511
       * 2. If ITerm reaches 5/6 of its max value, throw it to 40% of max value
       * 3. If Iterm reaches 1/6 of its max value, throw it to 60% of max value
       */
       
      // Clamp Section ***********************************************************
      // 1/6 * 1023 = 852.5 =~ 852 (54613)
      if (ITerm > 852.0) {
         // 4/10 * 1023 = 409.2 =~ (26214)
         ITerm = 409.0; 
      }
      // 1/6 * 1023 = 170.5 =~ 171 (10923)
      else if (ITerm <= 171.0) {
        // 6/10 * 1023 = 613.8 =~ 614 (39322)
         ITerm = 614.0; 
      }
      //
      
      /* Finalize output value */
      output = (uint16_t) ITerm + PTerm * multiplier;
      
      if(output > 1023) output = 1023;
      else if(output < 0) output = 0;
      *myOutput = (uint16_t) output;
      
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(uint16_t Kp, uint16_t Ki, uint16_t Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   kp = Kp;
   ki = Ki;
   kd = 0;
 
  if(controllerDirection == REVERSE)
   {
      kp = -kp;
      ki = -ki;
   }
}

void PID::SetMultiplier(uint8_t new_multiplier) {
    multiplier = new_multiplier;
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed  
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(uint16_t Min, uint16_t Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
       if(*myOutput > outMax) *myOutput = outMax;
       else if(*myOutput < outMin) *myOutput = outMin;
     
       if(ITerm > outMax) ITerm= outMax;
       else if(ITerm < outMin) ITerm= outMin;
   }
}


/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction != controllerDirection)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
int32_t PID::GetKp(){ return  kp;}
int32_t PID::GetKi(){ return  ki;}
int32_t PID::GetKd(){ return  kd;}
uint8_t PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
uint8_t PID::GetDirection(){ return controllerDirection;}

