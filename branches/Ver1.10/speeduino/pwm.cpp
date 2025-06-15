/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
#include "globals.h"
#include "maths.h"
#include "scheduledIO.h"
#include "timers.h"
#include "auxiliaries.h"
#include "pwm.h"

//******************** [PJSC v1.10] ********************
byte pjscDuty[13];
unsigned int pjsc_pwm_max_count[13];
volatile unsigned int pjsc_pwm_cur_value[13];
long pjsc_pwm_target_value[13];
volatile bool pjsc_pwm_state[13];
//******************** [PJSC v1.10] ********************

//*****************************************************************************************************************************************
//[PJSC]
//The control function to set PWM output to injector output channel.
void initialisePjsc(void)
{
  pjsc_pwm_max_count[CH_INJ1] = 1000000L / (4 * configPage15.pjscFreq);
  pjsc_pwm_max_count[CH_INJ2] = 1000000L / (4 * configPage15.pjscFreq);
  pjsc_pwm_max_count[CH_INJ3] = 1000000L / (4 * configPage15.pjscFreq);
  pjsc_pwm_max_count[CH_INJ4] = 1000000L / (4 * configPage15.pjscFreq);

  /*
  if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) )
  {
    closeInjector1and4();
    closeInjector2and3();
    DISABLE_TIMER_FUEL1TO4();
  }
  */
}

void pjscControl(byte injCh)
{
  if( (configPage15.squirtDeviceTypeCh1 == 1) || (configPage15.squirtDeviceTypeCh2 == 1) || (configPage15.squirtDeviceTypeCh3 == 1) || (configPage15.squirtDeviceTypeCh4 == 1) )
  {
    if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) && !BIT_CHECK(currentStatus.testMode, BIT_TEST_PWM) && !BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      pjsc_pwm_max_count[injCh] = 1000000L / (4 * configPage15.pjscFreq);

      switch(injCh)
      {
        case CH_INJ1:
          pjscDuty[CH_INJ1] = (unsigned long)currentStatus.inj1VE * (unsigned long)currentStatus.corrections / 100;
          if(pjscDuty[CH_INJ1] == 0)         { FUEL1_TIMER_DISABLE(); closeInjector1(); }
          else if (pjscDuty[CH_INJ1] >= 100) { FUEL1_TIMER_DISABLE(); openInjector1(); }
          else { pjsc_pwm_target_value[CH_INJ1] = percentage(pjscDuty[CH_INJ1], pjsc_pwm_max_count[CH_INJ1]); FUEL1_TIMER_ENABLE(); }
          break;

        case CH_INJ2:
          pjscDuty[CH_INJ2] = (unsigned long)currentStatus.inj2VE * (unsigned long)currentStatus.corrections / 100;
          if(pjscDuty[CH_INJ2] == 0)         { FUEL2_TIMER_DISABLE(); closeInjector2(); }
          else if (pjscDuty[CH_INJ2] >= 100) { FUEL2_TIMER_DISABLE(); openInjector2(); }
          else { pjsc_pwm_target_value[CH_INJ2] = percentage(pjscDuty[CH_INJ2], pjsc_pwm_max_count[CH_INJ2]); FUEL2_TIMER_ENABLE(); }
          break;

        case CH_INJ3:
          pjscDuty[CH_INJ3] = (unsigned long)currentStatus.inj3VE * (unsigned long)currentStatus.corrections / 100;
          if(pjscDuty[CH_INJ3] == 0)         { FUEL3_TIMER_DISABLE(); closeInjector3(); }
          else if (pjscDuty[CH_INJ3] >= 100) { FUEL3_TIMER_DISABLE(); openInjector3(); }
          else { pjsc_pwm_target_value[CH_INJ3] = percentage(pjscDuty[CH_INJ3], pjsc_pwm_max_count[CH_INJ3]); FUEL3_TIMER_ENABLE(); }
          break;

        case CH_INJ4:
          pjscDuty[CH_INJ4] = (unsigned long)currentStatus.inj4VE * (unsigned long)currentStatus.corrections / 100;
          if(pjscDuty[CH_INJ4] == 0)         { FUEL4_TIMER_DISABLE(); closeInjector4(); }
          else if (pjscDuty[CH_INJ4] >= 100) { FUEL4_TIMER_DISABLE(); openInjector4(); }
          else { pjsc_pwm_target_value[CH_INJ4] = percentage(pjscDuty[CH_INJ4], pjsc_pwm_max_count[CH_INJ4]); FUEL4_TIMER_ENABLE(); }
          break;

        default:
          break;
      }
    }
  }
}

//***************************************************************************************************************
//[PJSC v1.01] For injector test mode
//PWM and Pulse output setting for test mode

//*************** Toggle Injector1 output ***************
void pjsc1Toggle(void)
{
  if (pjsc_pwm_state[CH_INJ1])
  {
    if( configPage15.solenoidValveDirectionCh1 == 1 ) { openInjector1(); }
    else { closeInjector1(); }
    SET_COMPARE(FUEL1_COMPARE, FUEL1_COUNTER + (pjsc_pwm_max_count[CH_INJ1] - pjsc_pwm_cur_value[CH_INJ1]));
    pjsc_pwm_state[CH_INJ1] = false;

    currentStatus.testCnt++;
    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) ) { FUEL1_TIMER_DISABLE(); }
    }
  }
  else
  {
    if( configPage15.solenoidValveDirectionCh1 == 1 ) { closeInjector1(); }
    else { openInjector1(); }
    SET_COMPARE(FUEL1_COMPARE, FUEL1_COUNTER + pjsc_pwm_target_value[CH_INJ1]);
    pjsc_pwm_cur_value[CH_INJ1] = pjsc_pwm_target_value[CH_INJ1];
    pjsc_pwm_state[CH_INJ1] = true;
  }
}

//*************** Toggle Injector2 output ***************
void pjsc2Toggle(void)
{
  if (pjsc_pwm_state[CH_INJ2])
  {
    if( configPage15.solenoidValveDirectionCh2 == 1 ) { openInjector2(); }
    else { closeInjector2(); }
    SET_COMPARE(FUEL2_COMPARE, FUEL2_COUNTER + (pjsc_pwm_max_count[CH_INJ2] - pjsc_pwm_cur_value[CH_INJ2]));
    pjsc_pwm_state[CH_INJ2] = false;

    currentStatus.testCnt++;
    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) ) { FUEL2_TIMER_DISABLE(); }
    }
  }
  else
  {
    if( configPage15.solenoidValveDirectionCh2 == 1 ) { closeInjector2(); }
    else { openInjector2(); }
    SET_COMPARE(FUEL2_COMPARE, FUEL2_COUNTER + pjsc_pwm_target_value[CH_INJ2]);
    pjsc_pwm_cur_value[CH_INJ2] = pjsc_pwm_target_value[CH_INJ2];
    pjsc_pwm_state[CH_INJ2] = true;
  }
}

//*************** Toggle Injector3 output ***************
void pjsc3Toggle(void)
{
  if (pjsc_pwm_state[CH_INJ3])
  {
    if( configPage15.solenoidValveDirectionCh3 == 1 ) { openInjector3(); }
    else { closeInjector3(); }
    SET_COMPARE(FUEL3_COMPARE, FUEL3_COUNTER + (pjsc_pwm_max_count[CH_INJ3] - pjsc_pwm_cur_value[CH_INJ3]));
    pjsc_pwm_state[CH_INJ3] = false;

    currentStatus.testCnt++;
    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) ) { FUEL3_TIMER_DISABLE(); }
    }
  }
  else
  {
    if( configPage15.solenoidValveDirectionCh3 == 1 ) { closeInjector3(); }
    else { openInjector3(); }
    SET_COMPARE(FUEL3_COMPARE, FUEL3_COUNTER + pjsc_pwm_target_value[CH_INJ3]);
    pjsc_pwm_cur_value[CH_INJ3] = pjsc_pwm_target_value[CH_INJ3];
    pjsc_pwm_state[CH_INJ3] = true;
  }
}

//*************** Toggle Injector4 output ***************
void pjsc4Toggle(void)
{
  if (pjsc_pwm_state[CH_INJ4])
  {
    if( configPage15.solenoidValveDirectionCh4 == 1 ) { openInjector4(); }
    else { closeInjector4(); }
    SET_COMPARE(FUEL4_COMPARE, FUEL4_COUNTER + (pjsc_pwm_max_count[CH_INJ4] - pjsc_pwm_cur_value[CH_INJ4]));
    pjsc_pwm_state[CH_INJ4] = false;

    currentStatus.testCnt++;
    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) ) { FUEL4_TIMER_DISABLE(); }
    }
  }
  else
  {
    if( configPage15.solenoidValveDirectionCh4 == 1 ) { closeInjector4(); }
    else { openInjector4(); }
    SET_COMPARE(FUEL4_COMPARE, FUEL4_COUNTER + pjsc_pwm_target_value[CH_INJ4]);
    pjsc_pwm_cur_value[CH_INJ4] = pjsc_pwm_target_value[CH_INJ4];
    pjsc_pwm_state[CH_INJ4] = true;
  }
}

//*************** Toggle Ignition1 output ***************
void ign1Toggle(void)
{
  if (pjsc_pwm_state[CH_IGN1])
  {
    endCoil1Charge();
    SET_COMPARE(IGN1_COMPARE, IGN1_COUNTER + (pjsc_pwm_max_count[CH_IGN1] - pjsc_pwm_cur_value[CH_IGN1]));
    pjsc_pwm_state[CH_IGN1] = false;

    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      currentStatus.testCnt++;
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) )
      {
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN1);
        IGN1_TIMER_DISABLE();
      }
    }
  }
  else
  {
    beginCoil1Charge();
    SET_COMPARE(IGN1_COMPARE, IGN1_COUNTER + pjsc_pwm_target_value[CH_IGN1]);
    pjsc_pwm_cur_value[CH_IGN1] = pjsc_pwm_target_value[CH_IGN1];
    pjsc_pwm_state[CH_IGN1] = true;
  }
}

//*************** Toggle Ignition2 output ***************
void ign2Toggle(void)
{
  if (pjsc_pwm_state[CH_IGN2])
  {
    endCoil2Charge();
    SET_COMPARE(IGN2_COMPARE, IGN2_COUNTER + (pjsc_pwm_max_count[CH_IGN2] - pjsc_pwm_cur_value[CH_IGN2]));
    pjsc_pwm_state[CH_IGN2] = false;

    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      currentStatus.testCnt++;
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) )
      {
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN2);
        IGN2_TIMER_DISABLE();
      }
    }
  }
  else
  {
    beginCoil2Charge();
    SET_COMPARE(IGN2_COMPARE, IGN2_COUNTER + pjsc_pwm_target_value[CH_IGN2]);
    pjsc_pwm_cur_value[CH_IGN2] = pjsc_pwm_target_value[CH_IGN2];
    pjsc_pwm_state[CH_IGN2] = true;
 }
}

//*************** Toggle Ignition3 output ***************
void ign3Toggle(void)
{
  if (pjsc_pwm_state[CH_IGN3])
  {
    endCoil3Charge();
    SET_COMPARE(IGN3_COMPARE, IGN3_COUNTER + (pjsc_pwm_max_count[CH_IGN3] - pjsc_pwm_cur_value[CH_IGN3]));
    pjsc_pwm_state[CH_IGN3] = false;

    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      currentStatus.testCnt++;
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) )
      {
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN3);
        IGN3_TIMER_DISABLE();
      }
    }
  }
  else
  {
    beginCoil3Charge();
    SET_COMPARE(IGN3_COMPARE, IGN3_COUNTER + pjsc_pwm_target_value[CH_IGN3]);
    pjsc_pwm_cur_value[CH_IGN3] = pjsc_pwm_target_value[CH_IGN3];
    pjsc_pwm_state[CH_IGN3] = true;
  }
}

//*************** Toggle Ignition4 output ***************
void ign4Toggle(void)
{
  if (pjsc_pwm_state[CH_IGN4])
  {
    endCoil4Charge();
    SET_COMPARE(IGN4_COMPARE, IGN4_COUNTER + (pjsc_pwm_max_count[CH_IGN4] - pjsc_pwm_cur_value[CH_IGN4]));
    pjsc_pwm_state[CH_IGN4] = false;

    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      currentStatus.testCnt++;
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) )
      {
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN4);
        IGN4_TIMER_DISABLE();
      }
    }
  }
  else
  {
    beginCoil4Charge();
    SET_COMPARE(IGN4_COMPARE, IGN4_COUNTER + pjsc_pwm_target_value[CH_IGN4]);
    pjsc_pwm_cur_value[CH_IGN4] = pjsc_pwm_target_value[CH_IGN4];
    pjsc_pwm_state[CH_IGN4] = true;
  }
}

//*************** Injector test mode contorol for PWM output ***************
void hardWareTstControlPWM(byte injCh)
{
  pjsc_pwm_max_count[injCh] = 1000000L / (4 * configPage15.dutyFreqTst[injCh]);  //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
  pjscDuty[injCh] = configPage15.dutyRatioTst[injCh];

  switch (injCh) {
    case CH_INJ1:
      if(pjscDuty[injCh] == 0)         { FUEL1_TIMER_DISABLE(); closeInjector1(); }
      else if (pjscDuty[injCh] >= 100) { FUEL1_TIMER_DISABLE(); openInjector1(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); FUEL1_TIMER_ENABLE(); }
      break;
    
    case CH_INJ2:
      if(pjscDuty[injCh] == 0)         { FUEL1_TIMER_DISABLE(); closeInjector2(); }
      else if (pjscDuty[injCh] >= 100) { FUEL1_TIMER_DISABLE(); openInjector2(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); FUEL2_TIMER_ENABLE(); }
      break;
      
    case CH_INJ3:
      if(pjscDuty[injCh] == 0)         { FUEL1_TIMER_DISABLE(); closeInjector3(); }
      else if (pjscDuty[injCh] >= 100) { FUEL1_TIMER_DISABLE(); openInjector3(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); FUEL3_TIMER_ENABLE(); }
      break;
      
    case CH_INJ4:
      if(pjscDuty[injCh] == 0)         { FUEL1_TIMER_DISABLE(); closeInjector4(); }
      else if (pjscDuty[injCh] >= 100) { FUEL1_TIMER_DISABLE(); openInjector4(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); FUEL4_TIMER_ENABLE(); }
      break;
      
    case CH_IGN1:
      if(pjscDuty[injCh] == 0)         { IGN1_TIMER_DISABLE(); endCoil1Charge(); }
      else if (pjscDuty[injCh] >= 100) { IGN1_TIMER_DISABLE(); beginCoil1Charge(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); IGN1_TIMER_ENABLE(); }
      break;
      
    case CH_IGN2:
      if(pjscDuty[injCh] == 0)         { IGN2_TIMER_DISABLE(); endCoil2Charge(); }
      else if (pjscDuty[injCh] >= 100) { IGN2_TIMER_DISABLE(); beginCoil2Charge(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); IGN2_TIMER_ENABLE(); }
      break;
     
    case CH_IGN3:
      if(pjscDuty[injCh] == 0)         { IGN3_TIMER_DISABLE(); endCoil3Charge(); }
      else if (pjscDuty[injCh] >= 100) { IGN3_TIMER_DISABLE(); beginCoil3Charge(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); IGN3_TIMER_ENABLE(); }
      break;
      
    case CH_IGN4:
      if(pjscDuty[injCh] == 0)         { IGN4_TIMER_DISABLE(); endCoil4Charge(); }
      else if (pjscDuty[injCh] >= 100) { IGN4_TIMER_DISABLE(); beginCoil4Charge(); }
      else { pjsc_pwm_target_value[injCh] = percentage(pjscDuty[injCh], pjsc_pwm_max_count[injCh]); IGN4_TIMER_ENABLE(); }
      break;
      
    default:
      break;
  }
}

//*************** Injector test mode contorol for Pulse output ***************
void injTstControlPulse(byte injCh)
{
  if(configPage15.testop_inj == TEST_OP_ALL) { injCh = CH_INJ1; }

  if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
  {
    noInterrupts();
    pjsc_pwm_max_count[injCh] = 1000000L / (4 * 100000 / configPage15.testint);           //Converts the pulse interval in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
    pjsc_pwm_target_value[injCh] = 1000000L / (4 * 100000 / configPage15.testpw);         //Converts the pulse width in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
    pjsc_pwm_state[injCh] = true;
    interrupts();

    currentStatus.testCnt = 0;

    switch (injCh) {
      case CH_INJ1:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ1);
        FUEL1_TIMER_ENABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
        
      case CH_INJ2:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ2);
        FUEL2_TIMER_ENABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
        
      case CH_INJ3:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ3);
        FUEL3_TIMER_ENABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
        
      case CH_INJ4:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ4);
        FUEL4_TIMER_ENABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
        
      default:
        break;
    }
  }
  else
  {
    switch (injCh) {
      case CH_INJ1:
        closeInjector1();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ1);
        FUEL1_TIMER_DISABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
      
      case CH_INJ2:
        closeInjector2();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ2);
        FUEL2_TIMER_DISABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
        
      case CH_INJ3:
        closeInjector3();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ3);
        FUEL3_TIMER_DISABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
        
      case CH_INJ4:
        closeInjector4();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ4);
        FUEL4_TIMER_DISABLE();
        if(configPage15.testop_inj == TEST_OP_ONE) { break; }
      
      default:
        break;
    }
  }
}

//*************** Ignition test mode contorol for Pulse output ***************
void ignTstControlPulse(byte ignCh)
{
  uint16_t tmp_testint;
  uint16_t tmp_testpw;
  bool allChOutput = false;

  pjsc_pwm_state[ignCh] = false;

  if( BIT_CHECK(currentStatus.testOutputs, 1) )
  {
    tmp_testint = configPage15.testint;
    tmp_testpw = configPage15.testpw;
    if(configPage15.testop_coil == TEST_OP_ALL) { ignCh = CH_IGN1; allChOutput = true; }
    currentStatus.testCnt = 0;
  }

  if( BIT_CHECK(currentStatus.testOutputs, 1) && !BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )  //Ignition output off
  {
    switch (ignCh) {
      case CH_IGN1:
        endCoil1Charge();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN1);
        IGN1_TIMER_DISABLE();
        if(allChOutput == false) { break; }
      
      case CH_IGN2:
        endCoil2Charge();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN2);
        IGN2_TIMER_DISABLE();
        if(allChOutput == false) { break; }
      
      case CH_IGN3:
        endCoil3Charge();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN3);
        IGN3_TIMER_DISABLE();
        if(allChOutput == false) { break; }
      
      case CH_IGN4:
        endCoil4Charge();
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN4);
        IGN4_TIMER_DISABLE();
        if(allChOutput == false) { break; }
      
      default:
        break;
    }
  }
  else
  {
    pjsc_pwm_max_count[ignCh] = 1000000L * tmp_testint / 400000;           //Converts the pulse interval in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
    pjsc_pwm_target_value[ignCh] = 1000000L * tmp_testpw / 400000;         //Converts the pulse width in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle

    switch (ignCh) {
      case CH_IGN1:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN1);
        IGN1_TIMER_ENABLE();
        if(allChOutput == false) { break; }
        
      case CH_IGN2:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN2);
        IGN2_TIMER_ENABLE();
        if(allChOutput == false) { break; }
        
      case CH_IGN3:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN3);
        IGN3_TIMER_ENABLE();
        if(allChOutput == false) { break; }
        
      case CH_IGN4:
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN4);
        IGN4_TIMER_ENABLE();
        if(allChOutput == false) { break; }

      default:
        break;
    }
  }
}

//************************** [PJSC v1.10] Toggle MUXHC output **************************
void muxHCToggle(void)
{
  if (pjsc_pwm_state[CH_MUXHC])
  {
    closeMuxHC();
    SET_COMPARE(IDLE_COMPARE, IDLE_COUNTER + (pjsc_pwm_max_count[CH_MUXHC] - pjsc_pwm_cur_value[CH_MUXHC]) );
    pjsc_pwm_state[CH_MUXHC] = false;

    if( BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )
    {
      currentStatus.testCnt++;
      if( (currentStatus.testCnt >= configPage15.testinjcnt) && (configPage15.testinjcnt != 0) )
      {
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUX3);
        IDLE_TIMER_DISABLE();
      }
    }
  }
  else
  {
    openMuxHC();
    SET_COMPARE(IDLE_COMPARE, IDLE_COUNTER + pjsc_pwm_target_value[CH_MUXHC] );
    pjsc_pwm_cur_value[CH_MUXHC] = pjsc_pwm_target_value[CH_MUXHC];
    pjsc_pwm_state[CH_MUXHC] = true;
  }
}

//*************** [PJSC v1.10] MUX test mode contorol for PWM output ***************
void muxTstControlPWM(byte muxCh)
{
  pjsc_pwm_max_count[muxCh] = 1000000L / (16 * configPage15.dutyFreqTst[muxCh]);  //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
  pjscDuty[muxCh] = configPage15.dutyRatioTst[muxCh];

  switch (muxCh) {
    case CH_MUX1:
      if(pjscDuty[muxCh] == 0)         { DISABLE_BOOST_TIMER(); closeMux1(); }
      else if (pjscDuty[muxCh] >= 100) { DISABLE_BOOST_TIMER(); openMux1(); }
      else { pjsc_pwm_target_value[muxCh] = percentage(pjscDuty[muxCh], pjsc_pwm_max_count[muxCh]); ENABLE_BOOST_TIMER(); }
      break;
      
    case CH_MUX2:
      if(pjscDuty[muxCh] == 0)         { DISABLE_VVT_TIMER(); closeMux2(); }
      else if (pjscDuty[muxCh] >= 100) { DISABLE_VVT_TIMER(); openMux2(); }
      else { pjsc_pwm_target_value[muxCh] = percentage(pjscDuty[muxCh], pjsc_pwm_max_count[muxCh]); ENABLE_VVT_TIMER(); }
      break;
      
    case CH_MUX3:
      if(pjscDuty[muxCh] == 0)         { closeMux3(); }
      else if (pjscDuty[muxCh] >= 100) { openMux3(); }
      break;
      
    case CH_MUX4:
      if(pjscDuty[muxCh] == 0)         { closeMux4(); }
      else if (pjscDuty[muxCh] >= 100) { openMux4(); }
      break;
      
    case CH_MUXHC:
      if(pjscDuty[muxCh] == 0)         { IDLE_TIMER_DISABLE(); closeMuxHC(); }
      else if (pjscDuty[muxCh] >= 100) { IDLE_TIMER_DISABLE(); openMuxHC(); }
      else { pjsc_pwm_target_value[muxCh] = percentage(pjscDuty[muxCh], pjsc_pwm_max_count[muxCh]); IDLE_TIMER_ENABLE(); }
      break;

    default:
      break;
  }
}

//*************** [PJSC v1.10] MUX test mode contorol for Pulse output ***************
void muxPulseOutputControl(byte muxCh)
{
  uint16_t tmp_testint;
  uint16_t tmp_testpw;
  bool allChOutput = false;

  pjsc_pwm_state[muxCh] = false;

  if( BIT_CHECK(currentStatus.testOutputs, 1) )
  {
    tmp_testint = configPage15.testint;
    tmp_testpw = configPage15.testpw;
    if(configPage15.testop_mux == TEST_OP_ALL){ muxCh = CH_MUX1; allChOutput = true; }
    currentStatus.testCnt = 0;
  }

  if( BIT_CHECK(currentStatus.testOutputs, 1) && BIT_CHECK(currentStatus.testMode, BIT_TEST_PULSE) )  //MUX output off
  {
    pjsc_pwm_max_count[muxCh] = 1000000L / (16 * 100000 / tmp_testint);           //Converts the pulse interval in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
    pjsc_pwm_target_value[muxCh] = 1000000L / (16 * 100000 / tmp_testpw);         //Converts the pulse width in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle

    switch (muxCh) {
      case CH_MUX1:
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX1);
        ENABLE_BOOST_TIMER();
        if(allChOutput == false) { break; }

      case CH_MUX2:
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX2);
        ENABLE_VVT_TIMER();
        if(allChOutput == false) { break; }

      case CH_MUXHC:
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUXHC);
        IDLE_TIMER_ENABLE();
        if(allChOutput == false) { break; }

      default:
        break;
    }
  }
  else
  {
    switch (muxCh) {
      case CH_MUX1:
        closeMux1();
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUX1);
        DISABLE_BOOST_TIMER();
        if(allChOutput == false) { break; }

      case CH_MUX2:
        closeMux2();
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUX2);
        DISABLE_VVT_TIMER();
        if(allChOutput == false) { break; }

      case CH_MUXHC:
        closeMuxHC();
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUXHC);
        IDLE_TIMER_DISABLE();
        if(allChOutput == false) { break; }
      
      default:
        break;
    }
  }
}

//[PJSC v1.01]
//***************************************************************************************************************
