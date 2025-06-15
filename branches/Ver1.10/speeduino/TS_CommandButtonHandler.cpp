
/** \file
 * Header file for the TunerStudio command handler
 * The command handler manages all the inputs FROM TS which are issued when a command button is clicked by the user
 */

#include "globals.h"
#include "TS_CommandButtonHandler.h"
#include "utilities.h"
#include "scheduledIO.h"
#include "sensors.h"
#include "storage.h"
#include "SD_logger.h"
#include "pages.h"
#include "pwm.h"                 //[PJSC v1.10]
#include "auxiliaries.h"         //[PJSC v1.10]
#ifdef USE_MC33810
  #include "acc_mc33810.h"
#endif

static bool commandRequiresStoppedEngine(uint16_t buttonCommand)
{
  return ((buttonCommand >= TS_CMD_INJ1_ON) && (buttonCommand <= TS_CMD_IGN8_PULSED)) 
      || ((buttonCommand == TS_CMD_TEST_ENBL) || (buttonCommand == TS_CMD_TEST_DSBL));
}

/**
 * @brief 
 * 
 * @param buttonCommand The command number of the button that was clicked. See TS_CommendButtonHandler.h for a list of button IDs
 */
bool TS_CommandButtonsHandler(uint16_t buttonCommand)
{
  if (commandRequiresStoppedEngine(buttonCommand) && currentStatus.RPM != 0)
  {
    return false;
  }
  
  switch (buttonCommand)
  {
    case TS_CMD_TEST_DSBL: // cmd is stop
      BIT_CLEAR(currentStatus.testOutputs, 1);
      BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       //[PJSC v1.10tmp]
      endCoil1Charge();
      endCoil2Charge();
      endCoil3Charge();
      endCoil4Charge();
      #if IGN_CHANNELS >= 5
      endCoil5Charge();
      #endif
      #if IGN_CHANNELS >= 6
      endCoil6Charge();
      #endif
      #if IGN_CHANNELS >= 7
      endCoil7Charge();
      #endif
      #if IGN_CHANNELS >= 8
      endCoil8Charge();
      #endif


      closeInjector1();
      closeInjector2();
      closeInjector3();
      closeInjector4();
      #if INJ_CHANNELS >= 5
      closeInjector5();
      #endif
      #if INJ_CHANNELS >= 6
      closeInjector6();
      #endif
      #if INJ_CHANNELS >= 7
      closeInjector7();
      #endif
      #if INJ_CHANNELS >= 8
      closeInjector8();
      #endif
      //********** [PJSC v1.10] **********
      FUEL1_TIMER_DISABLE();
      FUEL2_TIMER_DISABLE();
      FUEL3_TIMER_DISABLE();
      FUEL4_TIMER_DISABLE();
      IGN1_TIMER_DISABLE();
      IGN2_TIMER_DISABLE();
      IGN3_TIMER_DISABLE();
      IGN4_TIMER_DISABLE();
      DISABLE_BOOST_TIMER();
      DISABLE_VVT_TIMER();
      currentStatus.testMode = 0;
      currentStatus.testModeActive = 0;
      //********** [PJSC v1.10] **********

      HWTest_INJ_Pulsed = 0;
      HWTest_IGN_Pulsed = 0;
      break;

    case TS_CMD_TEST_ENBL: // cmd is enable
      // currentStatus.testactive = 1;
      BIT_SET(currentStatus.testOutputs, 1);
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10tmp]
      break;

    case TS_CMD_INJ1_ON: // cmd group is for injector1 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.01]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ1);                   //[PJSC v1.01]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector1(); }
      break;

    case TS_CMD_INJ1_OFF: // cmd group is for injector1 off actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector1(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ1_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        closeInjector1();                                                     // |
        FUEL1_TIMER_DISABLE();                                                // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ1);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_INJ1_PULSED: // cmd group is for injector1 50% dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ1_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ1_CMD_BIT)) { closeInjector1(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ1);                 // |
        hardWareTstControlPWM(CH_INJ1);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_INJ2_ON: // cmd group is for injector2 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ2);                   //[PJSC v1.10]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector2(); }
      break;

    case TS_CMD_INJ2_OFF: // cmd group is for injector2 off actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector2(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ2_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        closeInjector2();                                                     // |
        FUEL2_TIMER_DISABLE();                                                // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ2);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_INJ2_PULSED: // cmd group is for injector2 50%dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ2_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ2_CMD_BIT)) { closeInjector2(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ2);                 // |
        hardWareTstControlPWM(CH_INJ2);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_INJ3_ON: // cmd group is for injector3 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ3);                   //[PJSC v1.10]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector3(); }
      break;

    case TS_CMD_INJ3_OFF: // cmd group is for injector3 off actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector3(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ3_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        closeInjector3();                                                     // |
        FUEL3_TIMER_DISABLE();                                                // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ3);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_INJ3_PULSED: // cmd group is for injector3 50%dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ3_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ3_CMD_BIT)) { closeInjector3(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ3);                 // |
        hardWareTstControlPWM(CH_INJ3);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_INJ4_ON: // cmd group is for injector4 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ4);                   //[PJSC v1.10]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector4(); }
      break;

    case TS_CMD_INJ4_OFF: // cmd group is for injector4 off actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector4(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ4_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        closeInjector4();                                                     // |
        FUEL4_TIMER_DISABLE();                                                // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_INJ4);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_INJ4_PULSED: // cmd group is for injector4 50% dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ4_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ4_CMD_BIT)) { closeInjector4(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_INJ4);                 // |
        hardWareTstControlPWM(CH_INJ4);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

#if INJ_CHANNELS >= 5              //[PJSC v1.10]
    case TS_CMD_INJ5_ON: // cmd group is for injector5 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector5(); }
      break;

    case TS_CMD_INJ5_OFF: // cmd group is for injector5 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector5(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ5_CMD_BIT); }
      break;

    case TS_CMD_INJ5_PULSED: // cmd group is for injector5 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ5_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ5_CMD_BIT)) { closeInjector5(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ6_ON: // cmd group is for injector6 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector6(); }
      break;

    case TS_CMD_INJ6_OFF: // cmd group is for injector6 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector6(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ6_CMD_BIT); }
      break;

    case TS_CMD_INJ6_PULSED: // cmd group is for injector6 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ6_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ6_CMD_BIT)) { closeInjector6(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ7_ON: // cmd group is for injector7 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector7(); }
      break;

    case TS_CMD_INJ7_OFF: // cmd group is for injector7 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector7(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ7_CMD_BIT); }
      break;

    case TS_CMD_INJ7_PULSED: // cmd group is for injector7 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ7_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ7_CMD_BIT)) { closeInjector7(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ8_ON: // cmd group is for injector8 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector8(); }
      break;

    case TS_CMD_INJ8_OFF: // cmd group is for injector8 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector8(); BIT_CLEAR(HWTest_INJ_Pulsed, INJ8_CMD_BIT); }
      break;

    case TS_CMD_INJ8_PULSED: // cmd group is for injector8 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_INJ_Pulsed, INJ8_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_Pulsed, INJ8_CMD_BIT)) { closeInjector8(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;
#endif                             //[PJSC v1.10]

    //************************* [PJSC v1.10] *************************
    case TS_CMD_FP_ON: // cmd group is for fuel pump on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
        BIT_SET(currentStatus.testModeActive, BIT_TEST_FP);
        FUEL_PUMP_ON(); currentStatus.fuelPumpOn = true;
      }
      break;
    case TS_CMD_FP_OFF: // cmd group is for fuel pump off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_FP);
        FUEL_PUMP_OFF(); currentStatus.fuelPumpOn = false; 
      }
      break;
    case TS_CMD_INJ_PULSE_ON: // cmd group is for injector Pulse output start actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(currentStatus.testMode, BIT_TEST_PULSE); injTstControlPulse(configPage15.testsel_inj); }
      break;
    case TS_CMD_INJ_PULSE_OFF: // cmd group is for injector Pulse output stop actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_CLEAR(currentStatus.testMode, BIT_TEST_PULSE); injTstControlPulse(configPage15.testsel_inj); }
      break;
    case TS_CMD_IGN_PULSE_ON: // cmd group is for ignition Pulse output start actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(currentStatus.testMode, BIT_TEST_PULSE); ignTstControlPulse((configPage15.testsel_coil + CH_IGN1)); }
      break;
    case TS_CMD_IGN_PULSE_OFF: // cmd group is for ignition Pulse output stop actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_CLEAR(currentStatus.testMode, BIT_TEST_PULSE); ignTstControlPulse((configPage15.testsel_coil + CH_IGN1)); }
      break;
    case TS_CMD_MUX_PULSE_ON: // cmd group is for MUX Pulse output start actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(currentStatus.testMode, BIT_TEST_PULSE); muxPulseOutputControl((configPage15.testsel_mux + CH_MUX1)); }
      break;
    case TS_CMD_MUX_PULSE_OFF: // cmd group is for MUX Pulse output stop actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_CLEAR(currentStatus.testMode, BIT_TEST_PULSE); muxPulseOutputControl((configPage15.testsel_mux + CH_MUX1)); }
      break;
    //************************* [PJSC v1.10] *************************

    case TS_CMD_IGN1_ON: // cmd group is for spark1 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN1);                   //[PJSC v1.10]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ beginCoil1Charge(); }
      break;

    case TS_CMD_IGN1_OFF: // cmd group is for spark1 off actions
//[PJSC v1.10]        if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil1Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN1_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        endCoil1Charge();                                                     // |
        IGN1_TIMER_DISABLE();                                                 // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN1);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_IGN1_PULSED: // cmd group is for spark1 50%dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN1_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN1_CMD_BIT)) { endCoil1Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN1);                 // |
        hardWareTstControlPWM(CH_IGN1);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_IGN2_ON: // cmd group is for spark2 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN2);                   //[PJSC v1.10]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil2Charge(); }
      break;

    case TS_CMD_IGN2_OFF: // cmd group is for spark2 off actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil2Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN2_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        endCoil2Charge();                                                     // |
        IGN2_TIMER_DISABLE();                                                 // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN2);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_IGN2_PULSED: // cmd group is for spark2 50%dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN2_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN2_CMD_BIT)) { endCoil2Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN2);                 // |
        hardWareTstControlPWM(CH_IGN2);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_IGN3_ON: // cmd group is for spark3 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN3);                   //[PJSC v1.10]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil3Charge(); }
      break;

    case TS_CMD_IGN3_OFF: // cmd group is for spark3 off actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil3Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN3_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        endCoil3Charge();                                                     // |
        IGN3_TIMER_DISABLE();                                                 // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN3);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_IGN3_PULSED: // cmd group is for spark3 50%dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN3_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN3_CMD_BIT)) { endCoil3Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN3);                 // |
        hardWareTstControlPWM(CH_IGN3);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_IGN4_ON: // cmd group is for spark4 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);                           //[PJSC v1.10]
      BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN4);                   //[PJSC v1.10]
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil4Charge(); }
      break;

    case TS_CMD_IGN4_OFF: // cmd group is for spark4 off actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil4Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN4_CMD_BIT); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10]
      {                                                                       // |
        endCoil4Charge();                                                     // |
        IGN4_TIMER_DISABLE();                                                 // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);                       // |
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);                      // |
        BIT_CLEAR(currentStatus.testModeActive, BIT_TEST_IGN4);               // V
      }                                                                       //[PJSC v1.10]
      break;

    case TS_CMD_IGN4_PULSED: // cmd group is for spark4 50%dc actions
//[PJSC v1.10]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN4_CMD_BIT); }
//[PJSC v1.10]      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN4_CMD_BIT)) { endCoil4Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                           //[PJSC v1.10] For PWM output by Test mode
      {                                                                       // |
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);                        // |
        BIT_SET(currentStatus.testModeActive, BIT_TEST_IGN4);                 // |
        hardWareTstControlPWM(CH_IGN4);                                       // V
      }                                                                       //[PJSC v1.10]
      break;

#if IGN_CHANNELS >= 5              //[PJSC v1.10]
    case TS_CMD_IGN5_ON: // cmd group is for spark5 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil5Charge(); }
      break;

    case TS_CMD_IGN5_OFF: // cmd group is for spark5 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil5Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN5_CMD_BIT); }
      break;

    case TS_CMD_IGN5_PULSED: // cmd group is for spark4 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN5_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN5_CMD_BIT)) { endCoil5Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN6_ON: // cmd group is for spark6 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil6Charge(); }
      break;

    case TS_CMD_IGN6_OFF: // cmd group is for spark6 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil6Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN6_CMD_BIT); }
      break;

    case TS_CMD_IGN6_PULSED: // cmd group is for spark6 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN6_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN6_CMD_BIT)) { endCoil6Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN7_ON: // cmd group is for spark7 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil7Charge(); }
      break;

    case TS_CMD_IGN7_OFF: // cmd group is for spark7 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil7Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN7_CMD_BIT); }
      break;

    case TS_CMD_IGN7_PULSED: // cmd group is for spark7 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN7_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN7_CMD_BIT)) { endCoil7Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN8_ON: // cmd group is for spark8 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil8Charge(); }
      break;

    case TS_CMD_IGN8_OFF: // cmd group is for spark8 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil8Charge(); BIT_CLEAR(HWTest_IGN_Pulsed, IGN8_CMD_BIT); }
      break;

    case TS_CMD_IGN8_PULSED: // cmd group is for spark8 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_SET(HWTest_IGN_Pulsed, IGN8_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_Pulsed, IGN8_CMD_BIT)) { endCoil8Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;
#endif                             //[PJSC v1.10]

    //************************* [PJSC v1.10] *************************
    case TS_CMD_MUX1_ON: // cmd group is for MUX1 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);
//[PJSC v1.10tmp]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { digitalWrite(pinMuxout1, HIGH); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                                                //[PJSC v1.10tmp]
      {                                                                                            //[PJSC v1.10tmp]
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX1);                                   //[PJSC v1.10tmp]
        openMux1();                                                                                //[PJSC v1.10tmp]
      }                                                                                            //[PJSC v1.10tmp]
      break;

    case TS_CMD_MUX1_OFF: // cmd group is for MUX1 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {                               
//[PJSC v1.10tmp]        digitalWrite(pinMuxout1, LOW);
        closeMux1();                                                                               //[PJSC v1.10tmp]
        DISABLE_BOOST_TIMER();
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON); 
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUX1);
      }     
      break;

    case TS_CMD_MUX1_50PC: // cmd group is for MUX1 PWM actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX1);
        muxTstControlPWM(CH_MUX1);
      }
      break;

    case TS_CMD_MUX2_ON: // cmd group is for MUX2 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);
//[PJSC v1.10tmp]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { digitalWrite(pinMuxout2, HIGH); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                                                //[PJSC v1.10tmp]
      {                                                                                            //[PJSC v1.10tmp]
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX2);                                   //[PJSC v1.10tmp]
        openMux2();                                                                                //[PJSC v1.10tmp]
      }                                                                                            //[PJSC v1.10tmp]
      break;

    case TS_CMD_MUX2_OFF: // cmd group is for MUX2 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
//[PJSC v1.10tmp]        digitalWrite(pinMuxout2, LOW);
        closeMux2();                                                                               //[PJSC v1.10tmp]
        DISABLE_VVT_TIMER();
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUX2);
      }
      break;

    case TS_CMD_MUX2_50PC: // cmd group is for MUX2 PWM actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX2);
        muxTstControlPWM(CH_MUX2);
      }
      break;

    case TS_CMD_MUX3_ON: // cmd group is for MUX3 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);
//[PJSC v1.10tmp]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { digitalWrite(pinMuxout3, HIGH); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                                                //[PJSC v1.10tmp]
      {                                                                                            //[PJSC v1.10tmp]
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX3);                                   //[PJSC v1.10tmp]
        openMux3();                                                                                //[PJSC v1.10tmp]
      }                                                                                            //[PJSC v1.10tmp]
      break;

    case TS_CMD_MUX3_OFF: // cmd group is for MUX3 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
//[PJSC v1.10tmp]        digitalWrite(pinMuxout3, LOW);
        closeMux3();                                                                               //[PJSC v1.10tmp]
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON); 
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUX3);
      }
      break;

    case TS_CMD_MUX3_50PC: // cmd group is for MUX3 PWM actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX3);
        muxTstControlPWM(CH_MUX3);
      }
      break;

    case TS_CMD_MUX4_ON: // cmd group is for MUX4 on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);
//[PJSC v1.10tmp]      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { digitalWrite(pinMuxout4, HIGH); }
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                                                //[PJSC v1.10tmp]
      {                                                                                            //[PJSC v1.10tmp]
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX4);                                   //[PJSC v1.10tmp]
        openMux4();                                                                                //[PJSC v1.10tmp]
      }                                                                                            //[PJSC v1.10tmp]
      break;

    case TS_CMD_MUX4_OFF: // cmd group is for MUX4 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
//[PJSC v1.10tmp]        digitalWrite(pinMuxout4, LOW);
        closeMux4();                                                                               //[PJSC v1.10tmp]
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON);
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUX4);
      }
      break;

    case TS_CMD_MUX4_50PC: // cmd group is for MUX4 PWM actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUX4);
        muxTstControlPWM(CH_MUX4);
      }
      break;

    case TS_CMD_MUXHC_ON: // cmd group is for MUX HC on actions
      BIT_SET(currentStatus.testMode, BIT_TEST_ON);
      if( BIT_CHECK(currentStatus.testOutputs, 1) )                                                //[PJSC v1.10tmp]
      {                                                                                            //[PJSC v1.10tmp]
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUXHC);                                  //[PJSC v1.10tmp]
        openMuxHC();                                                                               //[PJSC v1.10tmp]
      }                                                                                            //[PJSC v1.10tmp]
      break;

    case TS_CMD_MUXHC_OFF: // cmd group is for MUX HC off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {                               
        closeMuxHC();                                                                              //[PJSC v1.10tmp]
        DISABLE_BOOST_TIMER();
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_ON); 
        BIT_CLEAR(currentStatus.testMode, BIT_TEST_PWM);
        BIT_CLEAR(currentStatus.muxTestModeActive, BIT_TEST_MUXHC);
      }     
      break;

    case TS_CMD_MUXHC_50PC: // cmd group is for MUX HC PWM actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) )
      {
        BIT_SET(currentStatus.testMode, BIT_TEST_PWM);
        BIT_SET(currentStatus.muxTestModeActive, BIT_TEST_MUXHC);
        muxTstControlPWM(CH_MUXHC);
      }
      break;
    //************************* [PJSC v1.10] *************************

    //**************** [PJSC v1.10tmp] For PV control *****************
    case TS_CMD_PV_ROTATE:   // cmd group is for PV control
      pv_target_position_adc = configPage15.PVPosTarget;
      break;

    case TS_CMD_PV_DIR_HIGH:   // cmd group is for PV control
      PV_DIR_HIGH();
      break;

    case TS_CMD_PV_DIR_LOW:    // cmd group is for PV control
      PV_DIR_LOW();
      break;

    case TS_CMD_PV_PWM_HIGH:   // cmd group is for PV control
      PV_PWM_HIGH();
      break;

    case TS_CMD_PV_PWM_LOW:    // cmd group is for PV control
      PV_PWM_LOW();
      break;

    case TS_CMD_PV_DIS_HIGH:     // cmd group is for PV control
      PV_DIS_HIGH();
      break;

    case TS_CMD_PV_DIS_LOW:      // cmd group is for PV control
      PV_DIS_LOW();
      break;
    //**************** [PJSC v1.10tmp] For PV control *****************

    //VSS Calibration routines
    case TS_CMD_VSS_60KMH:
      {
        if(configPage2.vssMode == 1)
        {
          //Calculate the ratio of VSS reading from Aux/CAN input and actual VSS (assuming that actual VSS is really 60km/h).
          configPage2.vssPulsesPerKm = (currentStatus.canin[configPage2.vssAuxCh] / 60);
          writeConfig(veSetPage); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
          BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
        }
        else
        {
          //Calibrate the actual pulses per distance
          uint32_t calibrationGap = vssGetPulseGap(0);
          if( calibrationGap > 0 )
          {
            configPage2.vssPulsesPerKm = MICROS_PER_MIN / calibrationGap;
            writeConfig(veSetPage); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
            BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
          }
        }
      }
      break;

    //Calculate the RPM to speed ratio for each gear
    case TS_CMD_VSS_RATIO1:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio1 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO2:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio2 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO3:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio3 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO4: 
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio4 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO5:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio5 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO6:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio6 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    //STM32 Commands
    case TS_CMD_STM32_REBOOT: //
      doSystemReset();
      break;

    case TS_CMD_STM32_BOOTLOADER: //
      jumpToBootloader();
      break;

#ifdef SD_LOGGING
    case TS_CMD_SD_FORMAT: //Format SD card
      formatExFat();
      break;
#endif

    default:
      return false;
      break;
  }

  return true;
}
