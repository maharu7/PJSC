/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

#include "globals.h"
#include "scheduler.h"
#include "scheduledIO.h"


void initialiseSchedulers()
{
    nullSchedule.Status = OFF;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
    //Much help in this from http://arduinomega.blogspot.com.au/2011/05/timer2-and-overflow-interrupt-lets-get.html
    //Fuel Schedules, which uses timer 3
    TCCR3B = 0x00;          //Disable Timer3 while we set it up
    TCNT3  = 0;             //Reset Timer Count
    TIFR3  = 0x00;          //Timer3 INT Flag Reg: Clear Timer Overflow Flag
    TCCR3A = 0x00;          //Timer3 Control Reg A: Wave Gen Mode normal
    TCCR3B = (1 << CS12);   //Timer3 Control Reg B: Timer Prescaler set to 256. Refer to http://www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg
    //TCCR3B = 0x03;   //Timer3 Control Reg B: Timer Prescaler set to 64. Refer to http://www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg

    //Ignition Schedules, which uses timer 5. This is also used by the fast version of micros(). If the speed of this timer is changed from 4uS ticks, that MUST be changed as well. See globals.h and timers.ino
    TCCR5B = 0x00;          //Disable Timer5 while we set it up
    TCNT5  = 0;             //Reset Timer Count
    TIFR5  = 0x00;          //Timer5 INT Flag Reg: Clear Timer Overflow Flag
    TCCR5A = 0x00;          //Timer5 Control Reg A: Wave Gen Mode normal
    //TCCR5B = (1 << CS12);   //Timer5 Control Reg B: Timer Prescaler set to 256. Refer to http://www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg
    TCCR5B = 0x03;         //aka Divisor = 64 = 490.1Hz

    #if defined(TIMER5_MICROS)
      TIMSK5 |= (1 << TOIE5); //Enable the timer5 overflow interrupt (See timers.ino for ISR)
      TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt
    #endif

    //The remaining Schedules (Schedules 4 for fuel and ignition) use Timer4
    TCCR4B = 0x00;          //Disable Timer4 while we set it up
    TCNT4  = 0;             //Reset Timer Count
    TIFR4  = 0x00;          //Timer4 INT Flag Reg: Clear Timer Overflow Flag
    TCCR4A = 0x00;          //Timer4 Control Reg A: Wave Gen Mode normal
    TCCR4B = (1 << CS12);   //Timer4 Control Reg B: aka Divisor = 256 = 122.5HzTimer Prescaler set to 256. Refer to http://www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg
  
#endif

    fuelSchedule1.Status = OFF;
    fuelSchedule2.Status = OFF;
    fuelSchedule3.Status = OFF;
    fuelSchedule4.Status = OFF;
    fuelSchedule5.Status = OFF;
    fuelSchedule6.Status = OFF;
    fuelSchedule7.Status = OFF;
    fuelSchedule8.Status = OFF;

    fuelSchedule1.schedulesSet = 0;
    fuelSchedule2.schedulesSet = 0;
    fuelSchedule3.schedulesSet = 0;
    fuelSchedule4.schedulesSet = 0;
    fuelSchedule5.schedulesSet = 0;
    fuelSchedule6.schedulesSet = 0;
    fuelSchedule7.schedulesSet = 0;
    fuelSchedule8.schedulesSet = 0;

    fuelSchedule1.counter = &FUEL1_COUNTER;
    fuelSchedule2.counter = &FUEL2_COUNTER;
    fuelSchedule3.counter = &FUEL3_COUNTER;
    fuelSchedule4.counter = &FUEL4_COUNTER;
    #if (INJ_CHANNELS >= 5)
    fuelSchedule5.counter = &FUEL5_COUNTER;
    #endif
    #if (INJ_CHANNELS >= 6)
    fuelSchedule5.counter = &FUEL6_COUNTER;
    #endif
    #if (INJ_CHANNELS >= 7)
    fuelSchedule5.counter = &FUEL7_COUNTER;
    #endif
    #if (INJ_CHANNELS >= 8)
    fuelSchedule5.counter = &FUEL8_COUNTER;
    #endif

    ignitionSchedule1.Status = OFF;
    ignitionSchedule2.Status = OFF;
    ignitionSchedule3.Status = OFF;
    ignitionSchedule4.Status = OFF;
    ignitionSchedule5.Status = OFF;
    ignitionSchedule6.Status = OFF;
    ignitionSchedule7.Status = OFF;
    ignitionSchedule8.Status = OFF;

    ignitionSchedule1.schedulesSet = 0;
    ignitionSchedule2.schedulesSet = 0;
    ignitionSchedule3.schedulesSet = 0;
    ignitionSchedule4.schedulesSet = 0;
    ignitionSchedule5.schedulesSet = 0;
    ignitionSchedule6.schedulesSet = 0;
    ignitionSchedule7.schedulesSet = 0;
    ignitionSchedule8.schedulesSet = 0;

    //****************** [PJSC v1.01] Set PWM frequency for Solenoid mode ***************************
    if (configPage2.squirtDeviceType == 1) //Check squirt device
    {
      for (unsigned int numInj = 0; numInj < NUM_SQUIRT_DEVICE; numInj++)
      {
        pjsc_pwm_max_count[numInj] = 1000000L / (16 * configPage2.pjscFreq * 2);
      }
      FUEL1_TIMER_ENABLE(); //Turn on the B compare unit (ie turn on the interrupt)
      FUEL2_TIMER_ENABLE();
      FUEL3_TIMER_ENABLE();
      FUEL4_TIMER_ENABLE();
    }
    //****************** [PJSC v1.01] Set PWM frequency for Solenoid mode ***************************
}

/*
These 8 function turn a schedule on, provides the time to start and the duration and gives it callback functions.
All 8 functions operate the same, just on different schedules
Args:
startCallback: The function to be called once the timeout is reached
timeout: The number of uS in the future that the startCallback should be triggered
duration: The number of uS after startCallback is called before endCallback is called
endCallback: This function is called once the duration time has been reached
*/

//Experimental new generic function
void setFuelSchedule(struct Schedule *targetSchedule, unsigned long timeout, unsigned long duration)
{
  if(targetSchedule->Status != RUNNING) //Check that we're not already part way through a schedule
  {
    //Callbacks no longer used, but retained for now:
    //fuelSchedule1.StartCallback = startCallback;
    //fuelSchedule1.EndCallback = endCallback;
    targetSchedule->duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >16x (Each tick represents 16uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
    noInterrupts();
    targetSchedule->startCompare = *targetSchedule->counter + timeout_timer_compare;
    targetSchedule->endCompare = targetSchedule->startCompare + uS_TO_TIMER_COMPARE(duration);
    targetSchedule->Status = PENDING; //Turn this schedule on
    targetSchedule->schedulesSet++; //Increment the number of times this schedule has been set

    *targetSchedule->compare = targetSchedule->startCompare;
    interrupts();
    FUEL1_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    targetSchedule->nextStartCompare = *targetSchedule->counter + uS_TO_TIMER_COMPARE(timeout);
    targetSchedule->nextEndCompare = targetSchedule->nextStartCompare + uS_TO_TIMER_COMPARE(duration);
    targetSchedule->hasNextSchedule = true;
  }
}


//void setFuelSchedule1(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
void setFuelSchedule1(unsigned long timeout, unsigned long duration)
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD_SLOW)
  {
    if(fuelSchedule1.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      //Callbacks no longer used, but retained for now:
      //fuelSchedule1.StartCallback = startCallback;
      //fuelSchedule1.EndCallback = endCallback;
      fuelSchedule1.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if ((timeout+duration) > MAX_TIMER_PERIOD_SLOW) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD_SLOW - 1 - duration) ); } // If the timeout is >16x (Each tick represents 16uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule1.startCompare = FUEL1_COUNTER + timeout_timer_compare;
      fuelSchedule1.endCompare = fuelSchedule1.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      fuelSchedule1.Status = PENDING; //Turn this schedule on
      fuelSchedule1.schedulesSet++; //Increment the number of times this schedule has been set
      //Schedule 1 shares a timer with schedule 5
      //if(channel5InjEnabled) { FUEL1_COMPARE = setQueue(timer3Aqueue, &fuelSchedule1, &fuelSchedule5, FUEL1_COUNTER); }
      //else { timer3Aqueue[0] = &fuelSchedule1; timer3Aqueue[1] = &fuelSchedule1; timer3Aqueue[2] = &fuelSchedule1; timer3Aqueue[3] = &fuelSchedule1; FUEL1_COMPARE = fuelSchedule1.startCompare; }
      //timer3Aqueue[0] = &fuelSchedule1; timer3Aqueue[1] = &fuelSchedule1; timer3Aqueue[2] = &fuelSchedule1; timer3Aqueue[3] = &fuelSchedule1;
      FUEL1_COMPARE = fuelSchedule1.startCompare;
      interrupts();
      FUEL1_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      noInterrupts();
      fuelSchedule1.nextStartCompare = FUEL1_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
      fuelSchedule1.nextEndCompare = fuelSchedule1.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      fuelSchedule1.duration = duration;
      fuelSchedule1.hasNextSchedule = true;
      interrupts();
    } //Schedule is RUNNING
  } //Timeout less than threshold
}

void setFuelSchedule2(unsigned long timeout, unsigned long duration)
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD_SLOW)
  {
    if(fuelSchedule2.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      //Callbacks no longer used, but retained for now:
      //fuelSchedule2.StartCallback = startCallback;
      //fuelSchedule2.EndCallback = endCallback;
      fuelSchedule2.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD_SLOW) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule2.startCompare = FUEL2_COUNTER + timeout_timer_compare;
      fuelSchedule2.endCompare = fuelSchedule2.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      FUEL2_COMPARE = fuelSchedule2.startCompare; //Use the B compare unit of timer 3
      fuelSchedule2.Status = PENDING; //Turn this schedule on
      fuelSchedule2.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL2_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule2.nextStartCompare = FUEL2_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
      fuelSchedule2.nextEndCompare = fuelSchedule2.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      fuelSchedule2.hasNextSchedule = true;
    }
  }
}
//void setFuelSchedule3(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
void setFuelSchedule3(unsigned long timeout, unsigned long duration)
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD_SLOW)
  {
    if(fuelSchedule3.Status != RUNNING)//Check that we're not already part way through a schedule
    {
      //Callbacks no longer used, but retained for now:
      //fuelSchedule3.StartCallback = startCallback;
      //fuelSchedule3.EndCallback = endCallback;
      fuelSchedule3.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD_SLOW) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule3.startCompare = FUEL3_COUNTER + timeout_timer_compare;
      fuelSchedule3.endCompare = fuelSchedule3.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      FUEL3_COMPARE = fuelSchedule3.startCompare; //Use the C copmare unit of timer 3
      fuelSchedule3.Status = PENDING; //Turn this schedule on
      fuelSchedule3.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL3_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule3.nextStartCompare = FUEL3_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
      fuelSchedule3.nextEndCompare = fuelSchedule3.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      fuelSchedule3.hasNextSchedule = true;
    }
  }
}
//void setFuelSchedule4(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
void setFuelSchedule4(unsigned long timeout, unsigned long duration) //Uses timer 4 compare B
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD_SLOW)
  {
    if(fuelSchedule4.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      //Callbacks no longer used, but retained for now:
      //fuelSchedule4.StartCallback = startCallback;
      //fuelSchedule4.EndCallback = endCallback;
      fuelSchedule4.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD_SLOW) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule4.startCompare = FUEL4_COUNTER + timeout_timer_compare;
      fuelSchedule4.endCompare = fuelSchedule4.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      FUEL4_COMPARE = fuelSchedule4.startCompare; //Use the C copmare unit of timer 3
      fuelSchedule4.Status = PENDING; //Turn this schedule on
      fuelSchedule4.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL4_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule4.nextStartCompare = FUEL4_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
      fuelSchedule4.nextEndCompare = fuelSchedule4.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
      fuelSchedule4.hasNextSchedule = true;
    }
  }
}

#if INJ_CHANNELS >= 5
void setFuelSchedule5(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(fuelSchedule5.Status != RUNNING) //Check that we're not already part way through a schedule
  {
    fuelSchedule5.StartCallback = startCallback; //Name the start callback function
    fuelSchedule5.EndCallback = endCallback; //Name the end callback function
    fuelSchedule5.duration = duration;

    /*
     * The following must be enclosed in the noIntterupts block to avoid contention caused if the relevant interrupts fires before the state is fully set
     */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
    noInterrupts();
    fuelSchedule5.startCompare = TCNT3 + (timeout >> 4); //As above, but with bit shift instead of / 16
    fuelSchedule5.endCompare = fuelSchedule5.startCompare + (duration >> 4);
    fuelSchedule5.Status = PENDING; //Turn this schedule on
    fuelSchedule5.schedulesSet++; //Increment the number of times this schedule has been set
    OCR3A = setQueue(timer3Aqueue, &fuelSchedule1, &fuelSchedule5, TCNT3); //Schedule 1 shares a timer with schedule 5
    interrupts();
    TIMSK3 |= (1 << OCIE3A); //Turn on the A compare unit (ie turn on the interrupt)
#endif
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    fuelSchedule5.nextStartCompare = FUEL5_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
    fuelSchedule5.nextEndCompare = fuelSchedule5.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
    fuelSchedule5.hasNextSchedule = true;
  }
}
#endif

#if INJ_CHANNELS >= 6
//This uses timer
void setFuelSchedule6(unsigned long timeout, unsigned long duration)
{
  if(fuelSchedule6.Status != RUNNING) //Check that we're not already part way through a schedule
  {
    //Callbacks no longer used, but retained for now:
    //fuelSchedule4.StartCallback = startCallback;
    //fuelSchedule4.EndCallback = endCallback;
    fuelSchedule6.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD_SLOW) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

    //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
    noInterrupts();
    fuelSchedule6.startCompare = FUEL6_COUNTER + timeout_timer_compare;
    fuelSchedule6.endCompare = fuelSchedule6.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
    FUEL6_COMPARE = fuelSchedule6.startCompare; //Use the C copmare unit of timer 3
    fuelSchedule6.Status = PENDING; //Turn this schedule on
    fuelSchedule6.schedulesSet++; //Increment the number of times this schedule has been set
    interrupts();
    FUEL6_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    fuelSchedule6.nextStartCompare = FUEL6_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
    fuelSchedule6.nextEndCompare = fuelSchedule6.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
    fuelSchedule6.hasNextSchedule = true;
  }
}
#endif

#if INJ_CHANNELS >= 7
//This uses timer
void setFuelSchedule7(unsigned long timeout, unsigned long duration)
{
  if(fuelSchedule7.Status != RUNNING) //Check that we're not already part way through a schedule
  {
    //Callbacks no longer used, but retained for now:
    //fuelSchedule4.StartCallback = startCallback;
    //fuelSchedule4.EndCallback = endCallback;
    fuelSchedule7.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
    noInterrupts();
    fuelSchedule7.startCompare = FUEL7_COUNTER + timeout_timer_compare;
    fuelSchedule7.endCompare = fuelSchedule7.startCompare + uS_TO_TIMER_COMPARE(duration);
    FUEL7_COMPARE = fuelSchedule7.startCompare; //Use the C copmare unit of timer 3
    fuelSchedule7.Status = PENDING; //Turn this schedule on
    fuelSchedule7.schedulesSet++; //Increment the number of times this schedule has been set
    interrupts();
    FUEL7_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    fuelSchedule7.nextStartCompare = FUEL7_COUNTER + uS_TO_TIMER_COMPARE(timeout);
    fuelSchedule7.nextEndCompare = fuelSchedule7.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
    fuelSchedule7.hasNextSchedule = true;
  }
}
#endif

#if INJ_CHANNELS >= 8
//This uses timer
void setFuelSchedule8(unsigned long timeout, unsigned long duration)
{
  if(fuelSchedule8.Status != RUNNING) //Check that we're not already part way through a schedule
  {
    //Callbacks no longer used, but retained for now:
    //fuelSchedule4.StartCallback = startCallback;
    //fuelSchedule4.EndCallback = endCallback;
    fuelSchedule8.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
    noInterrupts();
    fuelSchedule8.startCompare = FUEL8_COUNTER + timeout_timer_compare;
    fuelSchedule8.endCompare = fuelSchedule8.startCompare + uS_TO_TIMER_COMPARE(duration);
    FUEL8_COMPARE = fuelSchedule8.startCompare; //Use the C copmare unit of timer 3
    fuelSchedule8.Status = PENDING; //Turn this schedule on
    fuelSchedule8.schedulesSet++; //Increment the number of times this schedule has been set
    interrupts();
    FUEL8_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    fuelSchedule8.nextStartCompare = FUEL8_COUNTER + uS_TO_TIMER_COMPARE(timeout);
    fuelSchedule8.nextEndCompare = fuelSchedule8.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
    fuelSchedule8.hasNextSchedule = true;
  }
}
#endif

//Ignition schedulers use Timer 5
void setIgnitionSchedule1(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule1.Status != RUNNING) //Check that we're not already part way through a schedule
  {
    ignitionSchedule1.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule1.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule1.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    //timeout -= (micros() - lastCrankAngleCalc);
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule1.startCompare = IGN1_COUNTER + timeout_timer_compare; //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)
    if(ignitionSchedule1.endScheduleSetByDecoder == false) { ignitionSchedule1.endCompare = ignitionSchedule1.startCompare + uS_TO_TIMER_COMPARE(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN1_COMPARE = ignitionSchedule1.startCompare;
    ignitionSchedule1.Status = PENDING; //Turn this schedule on
    ignitionSchedule1.schedulesSet++;
    interrupts();
    IGN1_TIMER_ENABLE();
  }
}

static inline void refreshIgnitionSchedule1(unsigned long timeToEnd)
{
  if( (ignitionSchedule1.Status == RUNNING) && (timeToEnd < ignitionSchedule1.duration) )
  //Must have the threshold check here otherwise it can cause a condition where the compare fires twice, once after the other, both for the end
  //if( (timeToEnd < ignitionSchedule1.duration) && (timeToEnd > IGNITION_REFRESH_THRESHOLD) )
  {
    noInterrupts();
    ignitionSchedule1.endCompare = IGN1_COUNTER + uS_TO_TIMER_COMPARE(timeToEnd);
    IGN1_COMPARE = ignitionSchedule1.endCompare;
    interrupts();
  }
}

void setIgnitionSchedule2(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule2.Status != RUNNING) //Check that we're not already part way through a schedule
  {
    ignitionSchedule2.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule2.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule2.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule2.startCompare = IGN2_COUNTER + timeout_timer_compare; //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)
    if(ignitionSchedule2.endScheduleSetByDecoder == false) { ignitionSchedule2.endCompare = ignitionSchedule2.startCompare + uS_TO_TIMER_COMPARE(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN2_COMPARE = ignitionSchedule2.startCompare;
    ignitionSchedule2.Status = PENDING; //Turn this schedule on
    ignitionSchedule2.schedulesSet++;
    interrupts();
    IGN2_TIMER_ENABLE();
  }
}
void setIgnitionSchedule3(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule3.Status != RUNNING) //Check that we're not already part way through a schedule
  {

    ignitionSchedule3.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule3.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule3.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule3.startCompare = IGN3_COUNTER + timeout_timer_compare; //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)
    if(ignitionSchedule3.endScheduleSetByDecoder == false) { ignitionSchedule3.endCompare = ignitionSchedule3.startCompare + uS_TO_TIMER_COMPARE(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN3_COMPARE = ignitionSchedule3.startCompare;
    ignitionSchedule3.Status = PENDING; //Turn this schedule on
    ignitionSchedule3.schedulesSet++;
    interrupts();
    IGN3_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    ignitionSchedule3.nextStartCompare = IGN3_COUNTER + uS_TO_TIMER_COMPARE(timeout);
    ignitionSchedule3.nextEndCompare = ignitionSchedule3.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
    ignitionSchedule3.hasNextSchedule = true;
  }
}
void setIgnitionSchedule4(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule4.Status != RUNNING) //Check that we're not already part way through a schedule
  {

    ignitionSchedule4.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule4.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule4.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule4.startCompare = IGN4_COUNTER + timeout_timer_compare;
    if(ignitionSchedule4.endScheduleSetByDecoder == false) { ignitionSchedule4.endCompare = ignitionSchedule4.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN4_COMPARE = ignitionSchedule4.startCompare;
    ignitionSchedule4.Status = PENDING; //Turn this schedule on
    ignitionSchedule4.schedulesSet++;
    interrupts();
    IGN4_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    ignitionSchedule4.nextStartCompare = IGN4_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
    ignitionSchedule4.nextEndCompare = ignitionSchedule4.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
    ignitionSchedule4.hasNextSchedule = true;
  }
}
void setIgnitionSchedule5(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule5.Status != RUNNING)//Check that we're not already part way through a schedule
  {

    ignitionSchedule5.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule5.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule5.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule5.startCompare = IGN5_COUNTER + timeout_timer_compare;
    if(ignitionSchedule5.endScheduleSetByDecoder == false) { ignitionSchedule5.endCompare = ignitionSchedule5.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN5_COMPARE = ignitionSchedule5.startCompare;
    ignitionSchedule5.Status = PENDING; //Turn this schedule on
    ignitionSchedule5.schedulesSet++;
    interrupts();
    IGN5_TIMER_ENABLE();
  }
}
void setIgnitionSchedule6(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule6.Status != RUNNING) //Check that we're not already part way through a schedule
  {

    ignitionSchedule6.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule6.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule6.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule6.startCompare = IGN6_COUNTER + timeout_timer_compare;
    if(ignitionSchedule6.endScheduleSetByDecoder == false) { ignitionSchedule6.endCompare = ignitionSchedule6.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN6_COMPARE = ignitionSchedule6.startCompare;
    ignitionSchedule6.Status = PENDING; //Turn this schedule on
    ignitionSchedule6.schedulesSet++;
    interrupts();
    IGN6_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    ignitionSchedule6.nextStartCompare = IGN6_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
    ignitionSchedule6.nextEndCompare = ignitionSchedule6.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
    ignitionSchedule6.hasNextSchedule = true;
  }
}
void setIgnitionSchedule7(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule7.Status != RUNNING) //Check that we're not already part way through a schedule
  {

    ignitionSchedule7.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule7.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule7.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule7.startCompare = IGN4_COUNTER + timeout_timer_compare;
    if(ignitionSchedule7.endScheduleSetByDecoder == false) { ignitionSchedule7.endCompare = ignitionSchedule7.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN7_COMPARE = ignitionSchedule7.startCompare;
    ignitionSchedule7.Status = PENDING; //Turn this schedule on
    ignitionSchedule7.schedulesSet++;
    interrupts();
    IGN7_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    ignitionSchedule7.nextStartCompare = IGN7_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
    ignitionSchedule7.nextEndCompare = ignitionSchedule7.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
    ignitionSchedule7.hasNextSchedule = true;
  }
}
void setIgnitionSchedule8(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  if(ignitionSchedule8.Status != RUNNING) //Check that we're not already part way through a schedule
  {

    ignitionSchedule8.StartCallback = startCallback; //Name the start callback function
    ignitionSchedule8.EndCallback = endCallback; //Name the start callback function
    ignitionSchedule8.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE_SLOW(timeout); } //Normal case

    noInterrupts();
    ignitionSchedule8.startCompare = IGN8_COUNTER + timeout_timer_compare;
    if(ignitionSchedule8.endScheduleSetByDecoder == false) { ignitionSchedule8.endCompare = ignitionSchedule8.startCompare + uS_TO_TIMER_COMPARE_SLOW(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    IGN8_COMPARE = ignitionSchedule8.startCompare;
    ignitionSchedule8.Status = PENDING; //Turn this schedule on
    ignitionSchedule8.schedulesSet++;
    interrupts();
    IGN8_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    ignitionSchedule8.nextStartCompare = IGN8_COUNTER + uS_TO_TIMER_COMPARE_SLOW(timeout);
    ignitionSchedule8.nextEndCompare = ignitionSchedule8.nextStartCompare + uS_TO_TIMER_COMPARE_SLOW(duration);
    ignitionSchedule8.hasNextSchedule = true;
  }
}

/*******************************************************************************************************************************************************************************************************/
//This function (All 8 ISR functions that are below) gets called when either the start time or the duration time are reached
//This calls the relevant callback function (startCallback or endCallback) depending on the status of the schedule.
//If the startCallback function is called, we put the scheduler into RUNNING state
//Timer3A (fuel schedule 1) Compare Vector
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPA_vect) //fuelSchedules 1 and 5
#else
static inline void fuelSchedule1Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if ( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE)) { injTstPulseToggle1(); }                                   //[PJSC v1.01]The interrupt to control the Pulse output test mode
    else if ( configPage2.squirtDeviceType == 1 || BIT_CHECK(currentStatus.testMode, BIT_TEST_PWM)) { pjsc1Toggle(); }  //[PJSC v1.01]The interrupt to control the PJSC PWM
    else                                                                                                                //[PJSC v1.01]
    {                                                                                                                   //[PJSC v1.01]
      if (fuelSchedule1.Status == PENDING) //Check to see if this schedule is turn on
      {
        //To use timer queue, change fuelShedule1 to timer3Aqueue[0];
//[PJSC v1.02]        if (configPage2.injLayout == INJ_SEMISEQUENTIAL) { openInjector1and4(); }
//[PJSC v1.02]        else { openInjector1(); }
        openInjector1();                                                                                                //[PJSC v1.02]
        fuelSchedule1.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
        FUEL1_COMPARE = FUEL1_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule1.duration); //Doing this here prevents a potential overflow on restarts
      }
      else if (fuelSchedule1.Status == RUNNING)
      {
         //timer3Aqueue[0]->EndCallback();
//[PJSC v1.02]         if (configPage2.injLayout == INJ_SEMISEQUENTIAL) { closeInjector1and4(); }
//[PJSC v1.02]         else { closeInjector1(); }
         closeInjector1();                                                                                              //[PJSC v1.02]
         fuelSchedule1.Status = OFF; //Turn off the schedule
         fuelSchedule1.schedulesSet = 0;
         //FUEL1_COMPARE = fuelSchedule1.endCompare;

         //If there is a next schedule queued up, activate it
         if(fuelSchedule1.hasNextSchedule == true)
         {
           FUEL1_COMPARE = fuelSchedule1.nextStartCompare;
           fuelSchedule1.endCompare = fuelSchedule1.nextEndCompare;
           fuelSchedule1.Status = PENDING;
           fuelSchedule1.schedulesSet = 1;
           fuelSchedule1.hasNextSchedule = false;
         }
         else { FUEL1_TIMER_DISABLE(); }
      }
      else if (fuelSchedule1.Status == OFF) { FUEL1_TIMER_DISABLE(); } //Safety check. Turn off this output compare unit and return without performing any action
    }                                                                                                             //[PJSC]
  }

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //fuelSchedule2
#else
static inline void fuelSchedule2Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if ( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE)) { injTstPulseToggle2(); }                                   //[PJSC v1.01]The interrupt to control the Pulse output test mode
    else if ( configPage2.squirtDeviceType == 1 || BIT_CHECK(currentStatus.testMode, BIT_TEST_PWM)) { pjsc2Toggle(); }  //[PJSC v1.01]The interrupt to control the PJSC PWM
    else                                                                                                                //[PJSC v1.01]
    {                                                                                                                   //[PJSC v1.01]
      if (fuelSchedule2.Status == PENDING) //Check to see if this schedule is turn on
      {
        //fuelSchedule2.StartCallback();
//[PJSC v1.02]        if (configPage2.injLayout == INJ_SEMISEQUENTIAL) { openInjector2and3(); }
        if ( (configPage2.injLayout == INJ_SEMISEQUENTIAL) && (configPage2.nCylinders == 4) ) { openInjector2and3(); }  //[PJSC v1.02]
        else { openInjector2(); }
        fuelSchedule2.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
        FUEL2_COMPARE = FUEL2_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule2.duration); //Doing this here prevents a potential overflow on restarts
      }
      else if (fuelSchedule2.Status == RUNNING)
      {
         //fuelSchedule2.EndCallback();
//[PJSC v1.02]         if (configPage2.injLayout == INJ_SEMISEQUENTIAL) { closeInjector2and3(); }
         if ( (configPage2.injLayout == INJ_SEMISEQUENTIAL) && (configPage2.nCylinders == 4) ) { closeInjector2and3(); } //[PJSC v1.02]
         else { closeInjector2(); }
         fuelSchedule2.Status = OFF; //Turn off the schedule
         fuelSchedule2.schedulesSet = 0;

         //If there is a next schedule queued up, activate it
         if(fuelSchedule2.hasNextSchedule == true)
         {
           FUEL2_COMPARE = fuelSchedule2.nextStartCompare;
           fuelSchedule2.endCompare = fuelSchedule2.nextEndCompare;
           fuelSchedule2.Status = PENDING;
           fuelSchedule2.schedulesSet = 1;
           fuelSchedule2.hasNextSchedule = false;
         }
         else { FUEL2_TIMER_DISABLE(); }
      }
    }                                                                                         //[PJSC]
  }

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //fuelSchedule3
#else
static inline void fuelSchedule3Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if ( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE)) { injTstPulseToggle3(); }                                   //[PJSC v1.01]The interrupt to control the Pulse output test mode
    else if ( configPage2.squirtDeviceType == 1 || BIT_CHECK(currentStatus.testMode, BIT_TEST_PWM)) { pjsc3Toggle(); }  //[PJSC v1.01]The interrupt to control the PJSC PWM
    else                                                                                                                //[PJSC v1.01]
    {                                                                                                                   //[PJSC v1.01]
      if (fuelSchedule3.Status == PENDING) //Check to see if this schedule is turn on
      {
        //fuelSchedule3.StartCallback();
        //Hack for 5 cylinder
        if(channel5InjEnabled) { openInjector3and5(); }
        else { openInjector3(); }
        fuelSchedule3.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
        FUEL3_COMPARE = FUEL3_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule3.duration); //Doing this here prevents a potential overflow on restarts
      }
      else if (fuelSchedule3.Status == RUNNING)
      {
         //fuelSchedule3.EndCallback();
         //Hack for 5 cylinder
         if(channel5InjEnabled) { closeInjector3and5(); }
         else { closeInjector3and5(); }
         fuelSchedule3.Status = OFF; //Turn off the schedule
         fuelSchedule3.schedulesSet = 0;

         //If there is a next schedule queued up, activate it
         if(fuelSchedule3.hasNextSchedule == true)
         {
           FUEL3_COMPARE = fuelSchedule3.nextStartCompare;
           fuelSchedule3.endCompare = fuelSchedule3.nextEndCompare;
           fuelSchedule3.Status = PENDING;
           fuelSchedule3.schedulesSet = 1;
           fuelSchedule3.hasNextSchedule = false;
         }
         else { FUEL3_TIMER_DISABLE(); }
      }
    }                                                                                         //[PJSC]
  }

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //fuelSchedule4
#else
static inline void fuelSchedule4Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if ( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE)) { injTstPulseToggle4(); }                                   //[PJSC v1.01]The interrupt to control the Pulse output test mode
    else if ( configPage2.squirtDeviceType == 1 || BIT_CHECK(currentStatus.testMode, BIT_TEST_PWM)) { pjsc4Toggle(); }  //[PJSC v1.01]The interrupt to control the PJSC PWM
    else                                                                                                                //[PJSC v1.01]
    {                                                                                                                   //[PJSC v1.01]
      if (fuelSchedule4.Status == PENDING) //Check to see if this schedule is turn on
      {
        //fuelSchedule4.StartCallback();
        openInjector4();
        fuelSchedule4.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
        FUEL4_COMPARE = FUEL4_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule4.duration); //Doing this here prevents a potential overflow on restarts
      }
      else if (fuelSchedule4.Status == RUNNING)
      {
         //fuelSchedule4.EndCallback();
         closeInjector4();
         fuelSchedule4.Status = OFF; //Turn off the schedule
         fuelSchedule4.schedulesSet = 0;

         //If there is a next schedule queued up, activate it
         if(fuelSchedule4.hasNextSchedule == true)
         {
           FUEL4_COMPARE = fuelSchedule4.nextStartCompare;
           fuelSchedule4.endCompare = fuelSchedule4.nextEndCompare;
           fuelSchedule4.Status = PENDING;
           fuelSchedule4.schedulesSet = 1;
           fuelSchedule4.hasNextSchedule = false;
         }
         else { FUEL4_TIMER_DISABLE(); }
      }
    }                                                                                         //[PJSC]
  }

#if (INJ_CHANNELS >= 5)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER1_COMPC_vect) //fuelSchedule5
#else
static inline void fuelSchedule5Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule5.Status == PENDING) //Check to see if this schedule is turn on
  {
    openInjector5();
    fuelSchedule5.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL5_COMPARE = fuelSchedule5.endCompare;
  }
  else if (fuelSchedule5.Status == RUNNING)
  {
     closeInjector5();
     fuelSchedule5.Status = OFF; //Turn off the schedule
     fuelSchedule5.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule5.hasNextSchedule == true)
     {
       FUEL5_COMPARE = fuelSchedule5.nextStartCompare;
       fuelSchedule5.endCompare = fuelSchedule5.nextEndCompare;
       fuelSchedule5.Status = PENDING;
       fuelSchedule5.schedulesSet = 1;
       fuelSchedule5.hasNextSchedule = false;
     }
     else { FUEL5_TIMER_DISABLE(); }
  }
}
#endif

#if (INJ_CHANNELS >= 6)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //fuelSchedule6
#else
static inline void fuelSchedule6Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule6.Status == PENDING) //Check to see if this schedule is turn on
  {
    //fuelSchedule4.StartCallback();
    openInjector6();
    fuelSchedule6.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL6_COMPARE = fuelSchedule6.endCompare;
  }
  else if (fuelSchedule6.Status == RUNNING)
  {
     //fuelSchedule4.EndCallback();
     closeInjector6();
     fuelSchedule6.Status = OFF; //Turn off the schedule
     fuelSchedule6.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule6.hasNextSchedule == true)
     {
       FUEL6_COMPARE = fuelSchedule6.nextStartCompare;
       fuelSchedule6.endCompare = fuelSchedule6.nextEndCompare;
       fuelSchedule6.Status = PENDING;
       fuelSchedule6.schedulesSet = 1;
       fuelSchedule6.hasNextSchedule = false;
     }
     else { FUEL6_TIMER_DISABLE(); }
  }
}
#endif

#if (INJ_CHANNELS >= 7)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //fuelSchedule7
#else
static inline void fuelSchedule7Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule7.Status == PENDING) //Check to see if this schedule is turn on
  {
    openInjector7();
    fuelSchedule7.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL7_COMPARE = fuelSchedule7.endCompare;
  }
  else if (fuelSchedule7.Status == RUNNING)
  {
     closeInjector7();
     fuelSchedule7.Status = OFF; //Turn off the schedule
     fuelSchedule7.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule7.hasNextSchedule == true)
     {
       FUEL7_COMPARE = fuelSchedule7.nextStartCompare;
       fuelSchedule7.endCompare = fuelSchedule7.nextEndCompare;
       fuelSchedule7.Status = PENDING;
       fuelSchedule7.schedulesSet = 1;
       fuelSchedule7.hasNextSchedule = false;
     }
     else { FUEL7_TIMER_DISABLE(); }
  }
}
#endif

#if (INJ_CHANNELS >= 8)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //fuelSchedule8
#else
static inline void fuelSchedule8Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule8.Status == PENDING) //Check to see if this schedule is turn on
  {
    //fuelSchedule4.StartCallback();
    openInjector8();
    fuelSchedule8.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL8_COMPARE = fuelSchedule8.endCompare;
  }
  else if (fuelSchedule8.Status == RUNNING)
  {
     //fuelSchedule4.EndCallback();
     closeInjector8();
     fuelSchedule8.Status = OFF; //Turn off the schedule
     fuelSchedule8.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule8.hasNextSchedule == true)
     {
       FUEL8_COMPARE = fuelSchedule8.nextStartCompare;
       fuelSchedule8.endCompare = fuelSchedule8.nextEndCompare;
       fuelSchedule8.Status = PENDING;
       fuelSchedule8.schedulesSet = 1;
       fuelSchedule8.hasNextSchedule = false;
     }
     else { FUEL8_TIMER_DISABLE(); }
  }
}
#endif

#if IGN_CHANNELS >= 1
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //ignitionSchedule1
#else
static inline void ignitionSchedule1Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule1.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule1.StartCallback();
      ignitionSchedule1.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule1.startTime = micros();
      if(ignitionSchedule1.endScheduleSetByDecoder == true) { IGN1_COMPARE = ignitionSchedule1.endCompare; }
      else { IGN1_COMPARE = IGN1_COUNTER + uS_TO_TIMER_COMPARE(ignitionSchedule1.duration); } //Doing this here prevents a potential overflow on restarts
    }
    else if (ignitionSchedule1.Status == RUNNING)
    {
      ignitionSchedule1.EndCallback();
      //   *ign1_pin_port &= ~(ign1_pin_mask);
      ignitionSchedule1.Status = OFF; //Turn off the schedule
      ignitionSchedule1.schedulesSet = 0;
      ignitionSchedule1.hasNextSchedule = false;
      ignitionSchedule1.endScheduleSetByDecoder = false;
      ignitionCount += 1; //Increment the igintion counter
      IGN1_TIMER_DISABLE();
    }
  }
#endif

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //ignitionSchedule2
#else
static inline void ignitionSchedule2Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule2.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule2.StartCallback();
      ignitionSchedule2.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule2.startTime = micros();
      if(ignitionSchedule2.endScheduleSetByDecoder == true) { IGN2_COMPARE = ignitionSchedule2.endCompare; } //If the decoder has set the end compare value, assign it to the next compare
      else { IGN2_COMPARE = IGN2_COUNTER + uS_TO_TIMER_COMPARE(ignitionSchedule2.duration); } //If the decoder based timing isn't set, doing this here prevents a potential overflow that can occur at low RPMs
    }
    else if (ignitionSchedule2.Status == RUNNING)
    {
      ignitionSchedule2.Status = OFF; //Turn off the schedule
      ignitionSchedule2.EndCallback();
      ignitionSchedule2.schedulesSet = 0;
      ignitionSchedule2.endScheduleSetByDecoder = false;
      ignitionCount += 1; //Increment the igintion counter
      IGN2_TIMER_DISABLE();
    }
  }
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //ignitionSchedule3
#else
static inline void ignitionSchedule3Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule3.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule3.StartCallback();
      ignitionSchedule3.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule3.startTime = micros();
      if(ignitionSchedule3.endScheduleSetByDecoder == true) { IGN3_COMPARE = ignitionSchedule3.endCompare; } //If the decoder has set the end compare value, assign it to the next compare
      else { IGN3_COMPARE = IGN3_COUNTER + uS_TO_TIMER_COMPARE(ignitionSchedule3.duration); } //If the decoder based timing isn't set, doing this here prevents a potential overflow that can occur at low RPMs
    }
    else if (ignitionSchedule3.Status == RUNNING)
    {
       ignitionSchedule3.Status = OFF; //Turn off the schedule
       ignitionSchedule3.EndCallback();
       ignitionSchedule3.schedulesSet = 0;
       ignitionSchedule3.endScheduleSetByDecoder = false;
       ignitionCount += 1; //Increment the igintion counter

       //If there is a next schedule queued up, activate it
       if(ignitionSchedule3.hasNextSchedule == true)
       {
         IGN3_COMPARE = ignitionSchedule3.nextStartCompare;
         ignitionSchedule3.endCompare = ignitionSchedule3.nextEndCompare;
         ignitionSchedule3.Status = PENDING;
         ignitionSchedule3.schedulesSet = 1;
         ignitionSchedule3.hasNextSchedule = false;
       }
       else { IGN3_TIMER_DISABLE(); }
    }
  }
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //ignitionSchedule4
#else
static inline void ignitionSchedule4Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule4.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule4.StartCallback();
      ignitionSchedule4.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule4.startTime = micros();
      IGN4_COMPARE = IGN4_COUNTER + uS_TO_TIMER_COMPARE_SLOW(ignitionSchedule4.duration); //Doing this here prevents a potential overflow on restarts
    }
    else if (ignitionSchedule4.Status == RUNNING)
    {
       ignitionSchedule4.Status = OFF; //Turn off the schedule
       ignitionSchedule4.EndCallback();
       ignitionSchedule4.schedulesSet = 0;
       ignitionSchedule4.endScheduleSetByDecoder = false;
       ignitionCount += 1; //Increment the igintion counter

       //If there is a next schedule queued up, activate it
       if(ignitionSchedule4.hasNextSchedule == true)
       {
         IGN4_COMPARE = ignitionSchedule4.nextStartCompare;
         ignitionSchedule4.endCompare = ignitionSchedule4.nextEndCompare;
         ignitionSchedule4.Status = PENDING;
         ignitionSchedule4.schedulesSet = 1;
         ignitionSchedule4.hasNextSchedule = false;
       }
       else { IGN4_TIMER_DISABLE(); }
    }
  }
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER1_COMPC_vect) //ignitionSchedule5
#else
static inline void ignitionSchedule5Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule5.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule5.StartCallback();
      ignitionSchedule5.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule5.startTime = micros();
      IGN5_COMPARE = IGN5_COUNTER + uS_TO_TIMER_COMPARE_SLOW(ignitionSchedule5.duration); //Doing this here prevents a potential overflow on restarts
    }
    else if (ignitionSchedule5.Status == RUNNING)
    {
       ignitionSchedule5.Status = OFF; //Turn off the schedule
       ignitionSchedule5.EndCallback();
       ignitionSchedule5.schedulesSet = 0;
       ignitionSchedule5.endScheduleSetByDecoder = false;
       ignitionCount += 1; //Increment the igintion counter
       IGN5_TIMER_DISABLE();
    }
  }
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER1_COMPC_vect) //ignitionSchedule6  NOT CORRECT!!!
#else
static inline void ignitionSchedule6Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule6.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule6.StartCallback();
      ignitionSchedule6.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule6.startTime = micros();
      IGN6_COMPARE = IGN6_COUNTER + uS_TO_TIMER_COMPARE(ignitionSchedule6.duration); //Doing this here prevents a potential overflow on restarts
    }
    else if (ignitionSchedule6.Status == RUNNING)
    {
       ignitionSchedule6.Status = OFF; //Turn off the schedule
       ignitionSchedule6.EndCallback();
       ignitionSchedule6.schedulesSet = 0;
       ignitionSchedule6.endScheduleSetByDecoder = false;
       ignitionCount += 1; //Increment the igintion counter
       IGN6_TIMER_DISABLE();
    }
  }
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER1_COMPC_vect) //ignitionSchedule6  NOT CORRECT!!!
#else
static inline void ignitionSchedule7Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule7.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule7.StartCallback();
      ignitionSchedule7.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule7.startTime = micros();
      IGN7_COMPARE = IGN7_COUNTER + uS_TO_TIMER_COMPARE(ignitionSchedule7.duration); //Doing this here prevents a potential overflow on restarts
    }
    else if (ignitionSchedule7.Status == RUNNING)
    {
       ignitionSchedule7.Status = OFF; //Turn off the schedule
       ignitionSchedule7.EndCallback();
       ignitionSchedule7.schedulesSet = 0;
       ignitionSchedule7.endScheduleSetByDecoder = false;
       ignitionCount += 1; //Increment the igintion counter
       IGN7_TIMER_DISABLE();
    }
  }
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER1_COMPC_vect) //ignitionSchedule8  NOT CORRECT!!!
#else
static inline void ignitionSchedule8Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (ignitionSchedule8.Status == PENDING) //Check to see if this schedule is turn on
    {
      ignitionSchedule8.StartCallback();
      ignitionSchedule8.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule8.startTime = micros();
      IGN8_COMPARE = IGN8_COUNTER + uS_TO_TIMER_COMPARE(ignitionSchedule8.duration); //Doing this here prevents a potential overflow on restarts
    }
    else if (ignitionSchedule8.Status == RUNNING)
    {
       ignitionSchedule8.Status = OFF; //Turn off the schedule
       ignitionSchedule8.EndCallback();
       ignitionSchedule8.schedulesSet = 0;
       ignitionSchedule8.endScheduleSetByDecoder = false;
       ignitionCount += 1; //Increment the igintion counter
       IGN8_TIMER_DISABLE();
    }
  }
#endif

//*****************************************************************************************************************************************
//[PJSC]
//The control function to set PWM output to injector output channel.
void pjscControl()
{
  if( currentStatus.testOutputs == 0 )     //[PJSC v1.01]
  {                                           //[PJSC v1.01]
    if( configPage2.squirtDeviceType == 1 )
    {
//[PJSC v1.01]      pjsc_pwm_max_count = 1000000L / (16 * configPage2.pjscFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
      for (unsigned int numInj = 0; numInj < NUM_SQUIRT_DEVICE; numInj++)             //[PJSC v1.01]
      {                                                                               //[PJSC v1.01]
        pjsc_pwm_max_count[numInj] = 1000000L / (16 * configPage2.pjscFreq * 2);      //[PJSC v1.01]
      }                                                                               //[PJSC v1.01]

      if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
      {
        unsigned long intermediate;
        unsigned int loopCnt;

        if ( configPage2.multiVEmapEnabled && configPage2.mapSeparationEnabled ) { loopCnt = MULTI_VE_COUNT; }
        else { loopCnt = SINGLE_VE_COUNT; }

        for (unsigned int numVe = 0; numVe < loopCnt; numVe++)
        {
          if(currentStatus.mapSelectSw) { tempVEvalue[numVe] = selectVE(currentStatus.veMapSelectionSw2Pri[numVe]); }
          else { tempVEvalue[numVe] = selectVE(currentStatus.veMapSelectionSw1Pri[numVe]); }

          if( configPage2.dualFuelEnabled )               // Dual Fuel Load
          {
            if( currentStatus.mapSelectSw ) { dualFuelLoadVE = selectVE(currentStatus.veMapSelectionSw2Sec[numVe]); }
            else { dualFuelLoadVE = selectVE(currentStatus.veMapSelectionSw1Sec[numVe]); }

            dualFuelLoadVE += (unsigned int)tempVEvalue[numVe];
            if ( configPage2.secondaryFuelUsage )         // Additive
            {
              if( dualFuelLoadVE > 100 ) { dualFuelLoadVE = 100; }
            }
            else                                          // Multiply VE
            {
              dualFuelLoadVE = dualFuelLoadVE >> 1;
            }

            tempVEvalue[numVe] = (byte)dualFuelLoadVE;
          }

          if ( configPage2.fuelCorrectionEnabled ) { intermediate = (unsigned long)tempVEvalue[numVe] * (unsigned long)currentStatus.corrections / 100; }       //[PJSC v1.01]
          else { intermediate = (unsigned long)tempVEvalue[numVe]; }                                                                                            //[PJSC v1.01]
//[PJSC v1.01]          intermediate = (unsigned long)tempVEvalue[numVe] * (unsigned long)currentStatus.corrections / 100;

          if ( configPage2.multiVEmapEnabled && configPage2.mapSeparationEnabled ) 
          {
            pjscDuty[numVe] = (byte)intermediate;
          }
          else
          {
            for (unsigned int numPjsc = 0; numPjsc < MULTI_VE_COUNT; numPjsc++)
            {
              pjscDuty[numPjsc] = (byte)intermediate;
            }
          }
        }

        setPjsc1Duty();              //[PJSC v1.01]
        setPjsc2Duty();              //[PJSC v1.01]
        setPjsc3Duty();              //[PJSC v1.01]
        setPjsc4Duty();              //[PJSC v1.01]
      }
      else
      {
        closeInjector1and4();
        closeInjector2and3();
        DISABLE_TIMER_FUEL1TO4();    //[PJSC v1.01]
      }
    }
    else
    { 
      // Disable timer channel
      DISABLE_TIMER_FUEL1TO4();      //[PJSC v1.01]
    }
  }                                  //[PJSC v1.01]
}
//[PJSC]

//***************************************************************************************************************
//[PJSC v1.01] For injector test mode
//PWM and Pulse output setting for test mode

//*************** Toggle Injector1 output ***************
void pjsc1Toggle()
{
  if (pjsc_pwm_state[CH_INJ1])
  {
    closeInjector1();
    FUEL1_COMPARE = FUEL1_COUNTER + (pjsc_pwm_max_count[CH_INJ1] - pjsc_pwm_cur_value[CH_INJ1]);
    pjsc_pwm_state[CH_INJ1] = false;
  }
  else
  {
    openInjector1();
    FUEL1_COMPARE = FUEL1_COUNTER + pjsc_pwm_target_value[CH_INJ1];
    pjsc_pwm_cur_value[CH_INJ1] = pjsc_pwm_target_value[CH_INJ1];
    pjsc_pwm_state[CH_INJ1] = true;
  }
}

//*************** Toggle Injector2 output ***************
void pjsc2Toggle()
{
  if (pjsc_pwm_state[CH_INJ2])
  {
    closeInjector2();
    FUEL2_COMPARE = FUEL2_COUNTER + (pjsc_pwm_max_count[CH_INJ2] - pjsc_pwm_cur_value[CH_INJ2]);
    pjsc_pwm_state[CH_INJ2] = false;
  }
  else
  {
    openInjector2();
    FUEL2_COMPARE = FUEL2_COUNTER + pjsc_pwm_target_value[CH_INJ2];
    pjsc_pwm_cur_value[CH_INJ2] = pjsc_pwm_target_value[CH_INJ2];
    pjsc_pwm_state[CH_INJ2] = true;
  }
}

//*************** Toggle Injector3 output ***************
void pjsc3Toggle()
{
  if (pjsc_pwm_state[CH_INJ3])
  {
    closeInjector3();
    FUEL3_COMPARE = FUEL3_COUNTER + (pjsc_pwm_max_count[CH_INJ3] - pjsc_pwm_cur_value[CH_INJ3]);
    pjsc_pwm_state[CH_INJ3] = false;
  }
  else
  {
    openInjector3();
    FUEL3_COMPARE = FUEL3_COUNTER + pjsc_pwm_target_value[CH_INJ3];
    pjsc_pwm_cur_value[CH_INJ3] = pjsc_pwm_target_value[CH_INJ3];
    pjsc_pwm_state[CH_INJ3] = true;
  }
}

//*************** Toggle Injector4 output ***************
void pjsc4Toggle()
{
  if (pjsc_pwm_state[CH_INJ4])
  {
    closeInjector4();
    FUEL4_COMPARE = FUEL4_COUNTER + (pjsc_pwm_max_count[CH_INJ4] - pjsc_pwm_cur_value[CH_INJ4]);
    pjsc_pwm_state[CH_INJ4] = false;
  }
  else
  {
    openInjector4();
    FUEL4_COMPARE = FUEL4_COUNTER + pjsc_pwm_target_value[CH_INJ4];
    pjsc_pwm_cur_value[CH_INJ4] = pjsc_pwm_target_value[CH_INJ4];
    pjsc_pwm_state[CH_INJ4] = true;
  }
}

//*************** Set PJSC Duty ratio for Injector1 output ***************
void setPjsc1Duty()
{
  if(pjscDuty[CH_INJ1] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeInjector1();
    FUEL1_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_INJ1] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openInjector1();
    FUEL1_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_INJ1] = percentage(pjscDuty[CH_INJ1], pjsc_pwm_max_count[CH_INJ1]);
    FUEL1_TIMER_ENABLE();
  }
}

//*************** Set PJSC Duty ratio for Injector2 output ***************
void setPjsc2Duty()
{
  if(pjscDuty[CH_INJ2] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeInjector2();
    FUEL2_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_INJ2] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openInjector2();
    FUEL2_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_INJ2] = percentage(pjscDuty[CH_INJ2], pjsc_pwm_max_count[CH_INJ2]);
    FUEL2_TIMER_ENABLE();
  }
}

//*************** Set PJSC Duty ratio for Injector3 output ***************
void setPjsc3Duty()
{
  if(pjscDuty[CH_INJ3] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeInjector3();
    FUEL3_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_INJ3] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openInjector3();
    FUEL3_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_INJ3] = percentage(pjscDuty[CH_INJ3], pjsc_pwm_max_count[CH_INJ3]);
    FUEL3_TIMER_ENABLE();
  }
}

//*************** Set PJSC Duty ratio for Injector4 output ***************
void setPjsc4Duty()
{
  if(pjscDuty[CH_INJ4] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeInjector4();
    FUEL4_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_INJ4] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openInjector4();
    FUEL4_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_INJ4] = percentage(pjscDuty[CH_INJ4], pjsc_pwm_max_count[CH_INJ4]);
    FUEL4_TIMER_ENABLE();
  }
}

//************************** Toggle MUX1 output **************************
void mux1Toggle()
{
  if (pjsc_pwm_state[CH_MUX1])
  {
    closeMux1();
    MUX1_COMPARE = MUX1_COUNTER + (pjsc_pwm_max_count[CH_MUX1] - pjsc_pwm_cur_value[CH_MUX1]);
    pjsc_pwm_state[CH_MUX1] = false;
  }
  else
  {
    openMux1();
    MUX1_COMPARE = MUX1_COUNTER + pjsc_pwm_target_value[CH_MUX1];
    pjsc_pwm_cur_value[CH_MUX1] = pjsc_pwm_target_value[CH_MUX1];
    pjsc_pwm_state[CH_MUX1] = true;
  }
}

//************************** Toggle MUX2 output **************************
void mux2Toggle()
{
  if (pjsc_pwm_state[CH_MUX2])
  {
    closeMux2();
    MUX2_COMPARE = MUX2_COUNTER + (pjsc_pwm_max_count[CH_MUX2] - pjsc_pwm_cur_value[CH_MUX2]);
    pjsc_pwm_state[CH_MUX2] = false;
  }
  else
  {
    openMux2();
    MUX2_COMPARE = MUX2_COUNTER + pjsc_pwm_target_value[CH_MUX2];
    pjsc_pwm_cur_value[CH_MUX2] = pjsc_pwm_target_value[CH_MUX2];
    pjsc_pwm_state[CH_MUX2] = true;
  }
}

//************************** Toggle MUX3 output **************************
void mux3Toggle()
{
  if (pjsc_pwm_state[CH_MUX3])
  {
    closeMux3();
    MUX3_COMPARE = MUX3_COUNTER + (pjsc_pwm_max_count[CH_MUX3] - pjsc_pwm_cur_value[CH_MUX3]);
    pjsc_pwm_state[CH_MUX3] = false;
  }
  else
  {
    openMux3();
    MUX3_COMPARE = MUX3_COUNTER + pjsc_pwm_target_value[CH_MUX3];
    pjsc_pwm_cur_value[CH_MUX3] = pjsc_pwm_target_value[CH_MUX3];
    pjsc_pwm_state[CH_MUX3] = true;
  }
}

//************************** Toggle MUX4 output **************************
void mux4Toggle()
{
  if (pjsc_pwm_state[CH_MUX4])
  {
    closeMux4();
    MUX4_COMPARE = MUX4_COUNTER + (pjsc_pwm_max_count[CH_MUX4] - pjsc_pwm_cur_value[CH_MUX4]);
    pjsc_pwm_state[CH_MUX4] = false;
  }
  else
  {
    openMux4();
    MUX4_COMPARE = MUX4_COUNTER + pjsc_pwm_target_value[CH_MUX4];
    pjsc_pwm_cur_value[CH_MUX4] = pjsc_pwm_target_value[CH_MUX4];
    pjsc_pwm_state[CH_MUX4] = true;
  }
}

//****************** Set PWM Duty ratio for MUX1 output ******************
void setMux1Duty()
{
  if(pjscDuty[CH_MUX1] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeMux1();
    MUX1_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_MUX1] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openMux1();
    MUX1_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_MUX1] = percentage(pjscDuty[CH_MUX1], pjsc_pwm_max_count[CH_MUX1]);
    MUX1_TIMER_ENABLE();
  }
}

//****************** Set PWM Duty ratio for MUX2 output ******************
void setMux2Duty()
{
  if(pjscDuty[CH_MUX2] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeMux2();
    MUX2_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_MUX2] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openMux2();
    MUX2_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_MUX2] = percentage(pjscDuty[CH_MUX2], pjsc_pwm_max_count[CH_MUX2]);
    MUX2_TIMER_ENABLE();
  }
}

//****************** Set PWM Duty ratio for MUX3 output ******************
void setMux3Duty()
{
  if(pjscDuty[CH_MUX3] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeMux3();
    MUX3_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_MUX3] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openMux3();
    MUX3_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_MUX3] = percentage(pjscDuty[CH_MUX3], pjsc_pwm_max_count[CH_MUX3]);
    MUX3_TIMER_ENABLE();
  }
}

//****************** Set PWM Duty ratio for MUX4 output ******************
void setMux4Duty()
{
  if(pjscDuty[CH_MUX4] == 0)
  {
    //Make sure solenoid is off (0% duty)
    closeMux4();
    MUX4_TIMER_DISABLE();
  }
  else if (pjscDuty[CH_MUX4] >= 100)
  {
    //Make sure solenoid is on (100% duty)
    openMux4();
    MUX4_TIMER_DISABLE();
  }
  else
  {
    pjsc_pwm_target_value[CH_MUX4] = percentage(pjscDuty[CH_MUX4], pjsc_pwm_max_count[CH_MUX4]);
    MUX4_TIMER_ENABLE();
  }
}

//*************** Injector test mode contorol for PWM output ***************
void hardWareTstControlPWM(byte injCh)
{
  pjsc_pwm_max_count[injCh] = 1000000L / (16 * configPage2.dutyFreqTst[injCh] * 2);  //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
  pjscDuty[injCh] = configPage2.dutyRatioTst[injCh];

  switch (injCh) {
    case CH_INJ1:
      setPjsc1Duty();
      break;
    
    case CH_INJ2:
      setPjsc2Duty();
      break;
      
    case CH_INJ3:
      setPjsc3Duty();
      break;
      
    case CH_INJ4:
      setPjsc4Duty();
      break;
      
    case CH_IGN1:
      setCoil1Duty();
      break;
      
    case CH_IGN2:
      setCoil2Duty();
      break;
     
    case CH_IGN3:
      setCoil3Duty();
      break;
      
    case CH_IGN4:
      setCoil4Duty();
      break;
      
    case CH_MUX1:
      setMux1Duty();
      break;
      
    case CH_MUX2:
      setMux2Duty();
      break;
      
    case CH_MUX3:
      setMux3Duty();
      break;
      
    case CH_MUX4:
      setMux4Duty();
      break;
      
    default:
      break;
  }
}

//*************** Toggle Injector1 Pulse output ***************
void injTstPulseToggle1()
{
  if (fuelSchedule1.Status == PENDING)
  {
    openInjector1();
    fuelSchedule1.Status = RUNNING;
    FUEL1_COMPARE = FUEL1_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule1.duration); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule1.Status == RUNNING)
  {
    closeInjector1();
    fuelSchedule1.Status = PENDING;
    FUEL1_COMPARE = FUEL1_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule1.startTime); //Doing this here prevents a potential overflow on restarts

    currentStatus.testCnt++;
    if( (currentStatus.testCnt >= configPage2.testinjcnt) && (configPage2.testinjcnt != 0) ) { FUEL1_TIMER_DISABLE(); }
  }
}

//*************** Toggle Injector2 Pulse output ***************
void injTstPulseToggle2()
{
  if (fuelSchedule2.Status == PENDING)
  {
    openInjector2();
    fuelSchedule2.Status = RUNNING;
    FUEL2_COMPARE = FUEL2_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule2.duration); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule2.Status == RUNNING)
  {
    closeInjector2();
    fuelSchedule2.Status = PENDING;
    FUEL2_COMPARE = FUEL2_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule2.startTime); //Doing this here prevents a potential overflow on restarts

    currentStatus.testCnt++;
    if( (currentStatus.testCnt >= configPage2.testinjcnt) && (configPage2.testinjcnt != 0) ) { FUEL2_TIMER_DISABLE(); }
  }
}

//*************** Toggle Injector3 Pulse output ***************
void injTstPulseToggle3()
{
  if (fuelSchedule3.Status == PENDING)
  {
    openInjector3();
    fuelSchedule3.Status = RUNNING;
    FUEL3_COMPARE = FUEL3_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule3.duration); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule3.Status == RUNNING)
  {
    closeInjector3();
    fuelSchedule3.Status = PENDING;
    FUEL3_COMPARE = FUEL3_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule3.startTime); //Doing this here prevents a potential overflow on restarts

    currentStatus.testCnt++;
    if( (currentStatus.testCnt >= configPage2.testinjcnt) && (configPage2.testinjcnt != 0) ) { FUEL3_TIMER_DISABLE(); }
  }
}

//*************** Toggle Injector4 Pulse output ***************
void injTstPulseToggle4()
{
  if (fuelSchedule4.Status == PENDING)
  {
    openInjector4();
    fuelSchedule4.Status = RUNNING;
    FUEL4_COMPARE = FUEL4_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule4.duration); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule4.Status == RUNNING)
  {
    closeInjector4();
    fuelSchedule4.Status = PENDING;
    FUEL4_COMPARE = FUEL4_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule4.startTime); //Doing this here prevents a potential overflow on restarts

    currentStatus.testCnt++;
    if( (currentStatus.testCnt >= configPage2.testinjcnt) && (configPage2.testinjcnt != 0) ) { FUEL4_TIMER_DISABLE(); }
  }
}

//*************** Injector test mode contorol for Pulse output ***************
void injTstControlPulse(byte injCh)
{
  if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
  {
    currentStatus.testCnt = 0;

    switch (injCh) {
      case CH_INJ1:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ1);
        fuelSchedule1.Status = PENDING;
        fuelSchedule1.startTime = (unsigned long)((configPage2.testint - configPage2.testpw) * 10);
        fuelSchedule1.duration = (unsigned long)(configPage2.testpw * 10);
        FUEL1_COMPARE = FUEL1_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule1.duration);
        interrupts();
        FUEL1_TIMER_ENABLE();
        break;
        
      case CH_INJ2:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ2);
        fuelSchedule2.Status = PENDING;
        fuelSchedule2.startTime = (unsigned long)((configPage2.testint - configPage2.testpw) * 10);
        fuelSchedule2.duration = (unsigned long)(configPage2.testpw * 10);
        FUEL2_COMPARE = FUEL2_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule2.duration);
        interrupts();
        FUEL2_TIMER_ENABLE();
        break;
        
      case CH_INJ3:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ3);
        fuelSchedule3.Status = PENDING;
        fuelSchedule3.startTime = (unsigned long)((configPage2.testint - configPage2.testpw) * 10);
        fuelSchedule3.duration = (unsigned long)(configPage2.testpw * 10);
        FUEL3_COMPARE = FUEL3_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule3.duration);
        interrupts();
        FUEL3_TIMER_ENABLE();
        break;
        
      case CH_INJ4:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ4);
        fuelSchedule4.Status = PENDING;
        fuelSchedule4.startTime = (unsigned long)((configPage2.testint - configPage2.testpw) * 10);
        fuelSchedule4.duration = (unsigned long)(configPage2.testpw * 10);
        FUEL4_COMPARE = FUEL4_COUNTER + uS_TO_TIMER_COMPARE_SLOW(fuelSchedule4.duration);
        interrupts();
        FUEL4_TIMER_ENABLE();
        break;
        
      default:
        break;
    }
  }
  else
  {
    switch (injCh) {
      case CH_INJ1:
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ1);
        closeInjector1();
        FUEL1_TIMER_DISABLE();
        break;
      
      case CH_INJ2:
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ2);
        closeInjector2();
        FUEL2_TIMER_DISABLE();
        break;
        
      case CH_INJ3:
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ3);
        closeInjector3();
        FUEL3_TIMER_DISABLE();
        break;
        
      case CH_INJ4:
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ4);
        closeInjector4();
        FUEL4_TIMER_DISABLE();
        break;
        
      default:
        break;
    }
  }
}
//[PJSC v1.01]
//***************************************************************************************************************

#if defined(CORE_TEENSY)
void ftm0_isr(void)
{
  //Use separate variables for each test to ensure conversion to bool
  bool interrupt1 = (FTM0_C0SC & FTM_CSC_CHF);
  bool interrupt2 = (FTM0_C1SC & FTM_CSC_CHF);
  bool interrupt3 = (FTM0_C2SC & FTM_CSC_CHF);
  bool interrupt4 = (FTM0_C3SC & FTM_CSC_CHF);
  bool interrupt5 = (FTM0_C4SC & FTM_CSC_CHF);
  bool interrupt6 = (FTM0_C5SC & FTM_CSC_CHF);
  bool interrupt7 = (FTM0_C6SC & FTM_CSC_CHF);
  bool interrupt8 = (FTM0_C7SC & FTM_CSC_CHF);

  if(interrupt1) { FTM0_C0SC &= ~FTM_CSC_CHF; fuelSchedule1Interrupt(); }
  else if(interrupt2) { FTM0_C1SC &= ~FTM_CSC_CHF; fuelSchedule2Interrupt(); }
  else if(interrupt3) { FTM0_C2SC &= ~FTM_CSC_CHF; fuelSchedule3Interrupt(); }
  else if(interrupt4) { FTM0_C3SC &= ~FTM_CSC_CHF; fuelSchedule4Interrupt(); }
  else if(interrupt5) { FTM0_C4SC &= ~FTM_CSC_CHF; ignitionSchedule1Interrupt(); }
  else if(interrupt6) { FTM0_C5SC &= ~FTM_CSC_CHF; ignitionSchedule2Interrupt(); }
  else if(interrupt7) { FTM0_C6SC &= ~FTM_CSC_CHF; ignitionSchedule3Interrupt(); }
  else if(interrupt8) { FTM0_C7SC &= ~FTM_CSC_CHF; ignitionSchedule4Interrupt(); }

}
void ftm3_isr(void)
{

#if (INJ_CHANNELS >= 5)
  bool interrupt1 = (FTM3_C0SC & FTM_CSC_CHF);
  if(interrupt1) { FTM3_C0SC &= ~FTM_CSC_CHF; fuelSchedule5Interrupt(); }
#endif
#if (INJ_CHANNELS >= 6)
  bool interrupt2 = (FTM3_C1SC & FTM_CSC_CHF);
  else if(interrupt2) { FTM3_C1SC &= ~FTM_CSC_CHF; fuelSchedule6Interrupt(); }
#endif
#if (INJ_CHANNELS >= 7)
  bool interrupt3 = (FTM3_C2SC & FTM_CSC_CHF);
  else if(interrupt3) { FTM3_C2SC &= ~FTM_CSC_CHF; fuelSchedule7Interrupt(); }
#endif
#if (INJ_CHANNELS >= 8)
  bool interrupt4 = (FTM3_C3SC & FTM_CSC_CHF);
  else if(interrupt4) { FTM3_C3SC &= ~FTM_CSC_CHF; fuelSchedule8Interrupt(); }
#endif
#if (IGN_CHANNELS >= 5)
  bool interrupt5 = (FTM3_C4SC & FTM_CSC_CHF);
  if(interrupt5) { FTM3_C4SC &= ~FTM_CSC_CHF; ignitionSchedule5Interrupt(); }
#endif
#if (IGN_CHANNELS >= 6)
  bool interrupt6 = (FTM3_C5SC & FTM_CSC_CHF);
  else if(interrupt6) { FTM3_C5SC &= ~FTM_CSC_CHF; ignitionSchedule6Interrupt(); }
#endif
#if (IGN_CHANNELS >= 7)
  bool interrupt7 = (FTM3_C6SC & FTM_CSC_CHF);
  else if(interrupt7) { FTM3_C6SC &= ~FTM_CSC_CHF; ignitionSchedule7Interrupt(); }
#endif
#if (IGN_CHANNELS >= 8)
  bool interrupt8 = (FTM3_C7SC & FTM_CSC_CHF);
  else if(interrupt8) { FTM3_C7SC &= ~FTM_CSC_CHF; ignitionSchedule8Interrupt(); }
#endif

}
#endif
