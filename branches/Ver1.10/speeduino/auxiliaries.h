#ifndef AUX_H
#define AUX_H

#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

#include <SimplyAtomic.h>

void initialiseAuxPWM(void);
void boostControl(void);
void boostDisable(void);
void boostByGear(void);
void vvtControl(void);
void initialiseFan(void);
void initialiseAirCon(void);
void nitrousControl(void);
void fanControl(void);
/* [PJSC v1.10] Omitto Ari conditoner controll
void airConControl(void);
bool READ_AIRCON_REQUEST(void);
[PJSC v1.10] Omitto Ari conditoner controll */ 
void wmiControl(void);
//******************** [PJSC v1.10] ********************
void initialiseStarter(void);
void starterControl(void);
void mux1Toggle(void);
void mux2Toggle(void);
void oilSolenoidControl(void);
void initialisePvControl(void);
void PvControl(void);
void PvPercentToADC(void);
void PvTest(void);
bool PvHoldCheck(void);
//******************** [PJSC v1.10] ********************

#define SIMPLE_BOOST_P  1
#define SIMPLE_BOOST_I  1
#define SIMPLE_BOOST_D  1

#if(defined(CORE_TEENSY) || defined(CORE_STM32))
#define BOOST_PIN_LOW()         (digitalWrite(pinBoost, LOW))
#define BOOST_PIN_HIGH()        (digitalWrite(pinBoost, HIGH))
#define VVT1_PIN_LOW()          (digitalWrite(pinVVT_1, LOW))
#define VVT1_PIN_HIGH()         (digitalWrite(pinVVT_1, HIGH))
#define VVT2_PIN_LOW()          (digitalWrite(pinVVT_2, LOW))
#define VVT2_PIN_HIGH()         (digitalWrite(pinVVT_2, HIGH))
#define FAN_PIN_LOW()           (digitalWrite(pinFan, LOW))
#define FAN_PIN_HIGH()          (digitalWrite(pinFan, HIGH))
#define N2O_STAGE1_PIN_LOW()    (digitalWrite(configPage10.n2o_stage1_pin, LOW))
#define N2O_STAGE1_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage1_pin, HIGH))
#define N2O_STAGE2_PIN_LOW()    (digitalWrite(configPage10.n2o_stage2_pin, LOW))
#define N2O_STAGE2_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage2_pin, HIGH))
/* [PJSC v1.10] Omitto Ari conditoner controll
#define AIRCON_PIN_LOW()        (digitalWrite(pinAirConComp, LOW))
#define AIRCON_PIN_HIGH()       (digitalWrite(pinAirConComp, HIGH))
#define AIRCON_FAN_PIN_LOW()    (digitalWrite(pinAirConFan, LOW))
#define AIRCON_FAN_PIN_HIGH()   (digitalWrite(pinAirConFan, HIGH))
[PJSC v1.10] Omitto Ari conditoner controll */ 
#define FUEL_PUMP_ON()          (digitalWrite(pinFuelPump, HIGH))
#define FUEL_PUMP_OFF()         (digitalWrite(pinFuelPump, LOW))
#define PV_DIR_LOW()            (digitalWrite(pinPvDIR, LOW))            //[PJSC v1.10] For PV control
#define PV_DIR_HIGH()           (digitalWrite(pinPvDIR, HIGH))           //[PJSC v1.10] For PV control
#define PV_PWM_LOW()            (digitalWrite(pinPvPWM, LOW))            //[PJSC v1.10] For PV control
#define PV_PWM_HIGH()           (digitalWrite(pinPvPWM, HIGH))           //[PJSC v1.10] For PV control
#define PV_DIS_LOW()            (digitalWrite(pinPvDIS, LOW))            //[PJSC v1.10] For PV control
#define PV_DIS_HIGH()           (digitalWrite(pinPvDIS, HIGH))           //[PJSC v1.10] For PV control
#define STARTER_PIN_LOW()       (digitalWrite(pinStarter, LOW))          //[PJSC v1.10]
#define STARTER_PIN_HIGH()      (digitalWrite(pinStarter, HIGH))         //[PJSC v1.10]

#else

#define BOOST_PIN_LOW()         ATOMIC() { *boost_pin_port &= ~(boost_pin_mask); }
#define BOOST_PIN_HIGH()        ATOMIC() { *boost_pin_port |= (boost_pin_mask);  }
#define VVT1_PIN_LOW()          ATOMIC() { *vvt1_pin_port &= ~(vvt1_pin_mask);   }
#define VVT1_PIN_HIGH()         ATOMIC() { *vvt1_pin_port |= (vvt1_pin_mask);    }
#define VVT2_PIN_LOW()          ATOMIC() { *vvt2_pin_port &= ~(vvt2_pin_mask);   }
#define VVT2_PIN_HIGH()         ATOMIC() { *vvt2_pin_port |= (vvt2_pin_mask);    }
#define N2O_STAGE1_PIN_LOW()    ATOMIC() { *n2o_stage1_pin_port &= ~(n2o_stage1_pin_mask);  }
#define N2O_STAGE1_PIN_HIGH()   ATOMIC() { *n2o_stage1_pin_port |= (n2o_stage1_pin_mask);   }
#define N2O_STAGE2_PIN_LOW()    ATOMIC() { *n2o_stage2_pin_port &= ~(n2o_stage2_pin_mask);  }
#define N2O_STAGE2_PIN_HIGH()   ATOMIC() { *n2o_stage2_pin_port |= (n2o_stage2_pin_mask);   }
#define FUEL_PUMP_ON()          ATOMIC() { *pump_pin_port |= (pump_pin_mask);     }
#define FUEL_PUMP_OFF()         ATOMIC() { *pump_pin_port &= ~(pump_pin_mask);    }
#define PV_DIR_LOW()            ATOMIC() { *pvdir_pin_port &= ~(pvdir_pin_mask); }          //[PJSC v1.10] For PV control
#define PV_DIR_HIGH()           ATOMIC() { *pvdir_pin_port |= (pvdir_pin_mask);  }          //[PJSC v1.10] For PV control
#define PV_PWM_LOW()            ATOMIC() { *pvpwm_pin_port &= ~(pvpwm_pin_mask); }          //[PJSC v1.10] For PV control
#define PV_PWM_HIGH()           ATOMIC() { *pvpwm_pin_port |= (pvpwm_pin_mask);  }          //[PJSC v1.10] For PV control
#define PV_DIS_LOW()            ATOMIC() { *pvdis_pin_port &= ~(pvdis_pin_mask); }          //[PJSC v1.10] For PV control
#define PV_DIS_HIGH()           ATOMIC() { *pvdis_pin_port |= (pvdis_pin_mask);  }          //[PJSC v1.10] For PV control
#define STARTER_PIN_LOW()       *starter_pin_port &= ~(starter_pin_mask)                    //[PJSC v1.10]
#define STARTER_PIN_HIGH()      *starter_pin_port |= (starter_pin_mask)                     //[PJSC v1.10]

//Note the below macros cannot use ATOMIC() as they are called from within ternary operators. The ATOMIC is instead placed around the ternary call below
#define FAN_PIN_LOW()           *fan_pin_port &= ~(fan_pin_mask)
#define FAN_PIN_HIGH()          *fan_pin_port |= (fan_pin_mask)
/* [PJSC v1.10] Omitto Ari conditoner controll
#define AIRCON_PIN_LOW()        *aircon_comp_pin_port &= ~(aircon_comp_pin_mask)
#define AIRCON_PIN_HIGH()       *aircon_comp_pin_port |= (aircon_comp_pin_mask)
#define AIRCON_FAN_PIN_LOW()    *aircon_fan_pin_port &= ~(aircon_fan_pin_mask)
#define AIRCON_FAN_PIN_HIGH()   *aircon_fan_pin_port |= (aircon_fan_pin_mask)
[PJSC v1.10] Omitto Ari conditoner controll */ 

#endif

/* [PJSC v1.10] Omitto Ari conditoner controll
#define AIRCON_ON()             ATOMIC() { ((((configPage15.airConCompPol)==1)) ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_OFF()            ATOMIC() { ((((configPage15.airConCompPol)==1)) ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_FAN_ON()         ATOMIC() { ((((configPage15.airConFanPol)==1)) ? AIRCON_FAN_PIN_LOW() : AIRCON_FAN_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_FAN); }
#define AIRCON_FAN_OFF()        ATOMIC() { ((((configPage15.airConFanPol)==1)) ? AIRCON_FAN_PIN_HIGH() : AIRCON_FAN_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_FAN); }
[PJSC v1.10] Omitto Ari conditoner controll */ 

#define FAN_ON()                ATOMIC() { ((configPage6.fanInv) ? FAN_PIN_LOW() : FAN_PIN_HIGH()); }
#define FAN_OFF()               ATOMIC() { ((configPage6.fanInv) ? FAN_PIN_HIGH() : FAN_PIN_LOW()); }

#define READ_N2O_ARM_PIN()    ((*n2o_arming_pin_port & n2o_arming_pin_mask) ? true : false)

#define VVT1_PIN_ON()     VVT1_PIN_HIGH();
#define VVT1_PIN_OFF()    VVT1_PIN_LOW();
#define VVT2_PIN_ON()     VVT2_PIN_HIGH();
#define VVT2_PIN_OFF()    VVT2_PIN_LOW();
#define VVT_TIME_DELAY_MULTIPLIER  50

#define WMI_TANK_IS_EMPTY() ((configPage10.wmiEmptyEnabled) ? ((configPage10.wmiEmptyPolarity) ? digitalRead(pinWMIEmpty) : !digitalRead(pinWMIEmpty)) : 1)

//******************** [PJSC v1.10] ********************
#define openMux1() *mux1_pin_port |= (mux1_pin_mask);
#define closeMux1() *mux1_pin_port &= ~(mux1_pin_mask);
#define openMux2() *mux2_pin_port |= (mux2_pin_mask);
#define closeMux2() *mux2_pin_port &= ~(mux2_pin_mask);
#define openMux3() *mux3_pin_port |= (mux3_pin_mask);
#define closeMux3() *mux3_pin_port &= ~(mux3_pin_mask);
#define openMux4() *mux4_pin_port |= (mux4_pin_mask);
#define closeMux4() *mux4_pin_port &= ~(mux4_pin_mask);
#define openMuxHC() *muxHC_pin_port |= (muxHC_pin_mask);
#define closeMuxHC() *muxHC_pin_port &= ~(muxHC_pin_mask);

#define STARTER_ENABLE()        { STARTER_PIN_HIGH(); }
#define STARTER_DISABLE()       { STARTER_PIN_LOW(); }

#define PV_STOP()               { PV_DIS_HIGH(); PV_DIR_LOW();  PV_PWM_LOW();  }
#define PV_FORWARD()            { PV_DIS_LOW();  PV_DIR_HIGH(); PV_PWM_HIGH(); }
#define PV_BACKWARD()           { PV_DIS_LOW();  PV_DIR_LOW();  PV_PWM_HIGH(); }
#define PV_FORWARD_BRAKE()      { PV_DIS_LOW();  PV_DIR_HIGH(); PV_PWM_LOW();  }
#define PV_BACKWARD_BRAKE()     { PV_DIS_LOW();  PV_DIR_LOW();  PV_PWM_LOW();  }

#define PV_HISTERYSIS           2
#define PV_STAY_COUNT_MAX      45
#define PV_STUCK_THRESHOLD    100

#define PV_OPE_STOP            0
#define PV_OPE_FORWARD          1
#define PV_OPE_BACKWARD         2
#define PV_OPE_FORWARD_BRAKE    3
#define PV_OPE_BACKWARD_BRAKE   4

#define PV_STATE_DEACTIVE       0
#define PV_STATE_INIT_CLOSE     1
#define PV_STATE_INIT_OPEN      2
#define PV_STATE_ACTIVE         3
#define PV_STATE_STUCK          4

#define BIT_PV_TEST_CLOSE_COMP  0
#define BIT_PV_TEST_OPEN_COMP   1
#define BIT_PV_TEST_COMP        2

extern volatile PORT_TYPE *mux1_pin_port;
extern volatile byte mux1_pin_mask;
extern volatile PORT_TYPE *mux2_pin_port;
extern volatile byte mux2_pin_mask;
extern volatile PORT_TYPE *mux3_pin_port;
extern volatile byte mux3_pin_mask;
extern volatile PORT_TYPE *mux4_pin_port;
extern volatile byte mux4_pin_mask;
extern volatile PORT_TYPE *muxHC_pin_port;
extern volatile byte muxHC_pin_mask;
extern volatile PORT_TYPE *pvdir_pin_port;
extern volatile PINMASK_TYPE pvdir_pin_mask;
extern volatile PORT_TYPE *pvpwm_pin_port;
extern volatile PINMASK_TYPE pvpwm_pin_mask;
extern volatile PORT_TYPE *pvdis_pin_port;
extern volatile PINMASK_TYPE pvdis_pin_mask;
extern volatile PORT_TYPE *starter_pin_port;
extern volatile PINMASK_TYPE starter_pin_mask;
extern volatile uint8_t pv_flag;
extern byte pv_operation;
extern byte pv_operation_prev;
extern byte pv_state;
extern byte pv_state_prev;
extern int pv_stay_count;
extern int pv_stuck_check_count;
extern unsigned int pv_target_position_adc;
extern unsigned int pv_pid_last_position_adc;
//extern unsigned int pv_pid_current_position_adc;
//******************** [PJSC v1.10] ********************

extern volatile PORT_TYPE *vvt1_pin_port;
extern volatile PINMASK_TYPE vvt1_pin_mask;
extern volatile PORT_TYPE *vvt2_pin_port;
extern volatile PINMASK_TYPE vvt2_pin_mask;
extern volatile PORT_TYPE *fan_pin_port;
extern volatile PINMASK_TYPE fan_pin_mask;

#if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
extern uint16_t fan_pwm_max_count; //Used for variable PWM frequency
void fanInterrupt(void);
#endif

extern uint16_t vvt_pwm_max_count; //Used for variable PWM frequency
extern uint16_t boost_pwm_max_count; //Used for variable PWM frequency

void boostInterrupt(void);
void vvtInterrupt(void);

#endif