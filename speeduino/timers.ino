/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
Timers are used for having actions performed repeatedly at a fixed interval (Eg every 100ms)
They should not be confused with Schedulers, which are for performing an action once at a given point of time in the future

Timers are typically low resolution (Compared to Schedulers), with maximum frequency currently being approximately every 10ms
*/
#include "timers.h"
#include "globals.h"
#include "sensors.h"
#include "scheduler.h"
#include "scheduledIO.h"
#include "auxiliaries.h"

#if defined(CORE_AVR)
  #include <avr/wdt.h>
#endif

void initialiseTimers()
{
  lastRPM_100ms = 0;
  loop33ms = 0;
  loop66ms = 0;
  loop100ms = 0;
  loop250ms = 0;
  loopSec = 0;
}


//Timer2 Overflow Interrupt Vector, called when the timer overflows.
//Executes every ~1ms.
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //This MUST be no block. Turning NO_BLOCK off messes with timing accuracy
#else
void oneMSInterval() //Most ARM chips can simply call a function
#endif
{
  ms_counter++;

  //Increment Loop Counters
  loop33ms++;
  loop66ms++;
  loop100ms++;
  loop250ms++;
  loopSec++;

  unsigned long targetOverdwellTime;

  //Overdwell check
  targetOverdwellTime = micros() - dwellLimit_uS; //Set a target time in the past that all coil charging must have begun after. If the coil charge began before this time, it's been running too long
  bool isCrankLocked = configPage4.ignCranklock && (currentStatus.RPM < currentStatus.crankRPM); //Dwell limiter is disabled during cranking on setups using the locked cranking timing. WE HAVE to do the RPM check here as relying on the engine cranking bit can be potentially too slow in updating
  //Check first whether each spark output is currently on. Only check it's dwell time if it is

  if(ignitionSchedule1.Status == RUNNING) { if( (ignitionSchedule1.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil1Charge(); ignitionSchedule1.Status = OFF; } }
  if(ignitionSchedule2.Status == RUNNING) { if( (ignitionSchedule2.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil2Charge(); ignitionSchedule2.Status = OFF; } }
  if(ignitionSchedule3.Status == RUNNING) { if( (ignitionSchedule3.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil3Charge(); ignitionSchedule3.Status = OFF; } }
  if(ignitionSchedule4.Status == RUNNING) { if( (ignitionSchedule4.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil4Charge(); ignitionSchedule4.Status = OFF; } }
  if(ignitionSchedule5.Status == RUNNING) { if( (ignitionSchedule5.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil5Charge(); ignitionSchedule5.Status = OFF; } }



  //30Hz loop
  if (loop33ms == 33)
  {
    loop33ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_30HZ);
  }

  //15Hz loop
  if (loop66ms == 66)
  {
    loop66ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_15HZ);
  }

  //Loop executed every 100ms loop
  //Anything inside this if statement will run every 100ms.
  if (loop100ms == 100)
  {
    loop100ms = 0; //Reset counter
    BIT_SET(TIMER_mask, BIT_TIMER_10HZ);

    currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10; //This is the RPM per second that the engine has accelerated/decelleratedin the last loop
    lastRPM_100ms = currentStatus.RPM; //Record the current RPM for next calc
  }

  //Loop executed every 250ms loop (1ms x 250 = 250ms)
  //Anything inside this if statement will run every 250ms.
  if (loop250ms == 250)
  {
    loop250ms = 0; //Reset Counter
    BIT_SET(TIMER_mask, BIT_TIMER_4HZ);
    #if defined(CORE_STM32) //debug purpose, only visal for running code
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    #endif

    #if defined(CORE_AVR)
      //Reset watchdog timer (Not active currently)
      //wdt_reset();
      //DIY watchdog
      //This is a sign of a crash:
      //if( (initialisationComplete == true) && (last250msLoopCount == mainLoopCount) ) { setup(); }
      //else { last250msLoopCount = mainLoopCount; }
    #endif
  }

  //Loop executed every 1 second (1ms x 1000 = 1000ms)
  if (loopSec == 1000)
  {
    loopSec = 0; //Reset counter.
    BIT_SET(TIMER_mask, BIT_TIMER_1HZ);

    dwellLimit_uS = (1000 * configPage4.dwellLimit); //Update uS value incase setting has changed
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10);

    //**************************************************************************************************************************************************
    //This updates the runSecs variable
    //If the engine is running or cranking, we need ot update the run time counter.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
    { //NOTE - There is a potential for a ~1sec gap between engine crank starting and ths runSec number being incremented. This may delay ASE!
      if (currentStatus.runSecs <= 254) //Ensure we cap out at 255 and don't overflow. (which would reset ASE and cause problems with the closed loop fueling (Which has to wait for the O2 to warmup))
        { currentStatus.runSecs++; } //Increment our run counter by 1 second.
    }
    //**************************************************************************************************************************************************
    //This records the number of main loops the system has completed in the last second
    currentStatus.loopsPerSecond = mainLoopCount;
    mainLoopCount = 0;
    //**************************************************************************************************************************************************
    //increament secl (secl is simply a counter that increments every second and is used to track whether the system has unexpectedly reset
    currentStatus.secl++;
    //**************************************************************************************************************************************************
    //Check the fan output status
//[PJSC v1.01]    if (configPage6.fanEnable == 1)
    if ( (configPage6.fanEnable == 1) && (currentStatus.testOutputs == 0) )        //[PJSC v1.01]
    {
       fanControl();            // Fucntion to turn the cooling fan on/off
    }

    //Check whether fuel pump priming is complete
    if(fpPrimed == false)
    {
      if(currentStatus.secl >= configPage2.fpPrime)
      {
        fpPrimed = true; //Mark the priming as being completed
        if(currentStatus.RPM == 0)
        {
          //If we reach here then the priming is complete, however only turn off the fuel pump if the engine isn't running
          digitalWrite(pinFuelPump, LOW);
          currentStatus.fuelPumpOn = false;
        }
      }
    }
    //**************************************************************************************************************************************************
    //Set the flex reading (if enabled). The flexCounter is updated with every pulse from the sensor. If cleared once per second, we get a frequency reading
    if(configPage2.flexEnabled == true)
    {
      if(flexCounter < 50)
      {
        currentStatus.ethanolPct = 0; //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol). Subtracting 50 from the frequency therefore gives the ethanol percentage.
        flexCounter = 0;
      }
      else if (flexCounter > 151) //1 pulse buffer
      {

        if(flexCounter < 169)
        {
          currentStatus.ethanolPct = 100;
          flexCounter = 0;
        }
        else
        {
          //This indicates an error condition. Spec of the sensor is that errors are above 170Hz)
          currentStatus.ethanolPct = 0;
          flexCounter = 0;
        }
      }
      else
      {
        currentStatus.ethanolPct = flexCounter - 50; //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol). Subtracting 50 from the frequency therefore gives the ethanol percentage.
        flexCounter = 0;
      }

      //Off by 1 error check
      if (currentStatus.ethanolPct == 1) { currentStatus.ethanolPct = 0; }

    }

  }
#if defined(CORE_AVR) //AVR chips use the ISR for this
    //Reset Timer2 to trigger in another ~1ms
    TCNT2 = 131;            //Preload timer2 with 100 cycles, leaving 156 till overflow.
    TIFR2  = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
#endif
}

