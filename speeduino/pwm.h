#ifndef PWM_H
#define PWM_H

#include "globals.h"

//******************** [PJSC v1.10] ********************
void initialisePjsc(void);
void pjscControl(byte injCh);
void pjsc1Toggle(void);
void pjsc2Toggle(void);
void pjsc3Toggle(void);
void pjsc4Toggle(void);
void ign1Toggle(void);
void ign2Toggle(void);
void ign3Toggle(void);
void ign4Toggle(void);
void hardWareTstControlPWM(byte injCh);
void injTstControlPulse(byte injCh);
void ignTstControlPulse(byte ignCh);
void muxHCToggle(void);
void muxPulseOutputControl(byte muxCh);
void muxTstControlPWM(byte muxCh);

extern byte pjscDuty[4];
extern byte pjscDuty_inj;
extern byte pjscDuty_spark;
extern byte pjscDuty_mux;
extern unsigned int pjsc_pwm_max_count[4];
extern unsigned int pjsc_pwm_max_count_spark;
extern unsigned int pjsc_pwm_max_count_mux1;
extern unsigned int pjsc_pwm_max_count_mux2;
extern unsigned int pjsc_pwm_max_count_muxHC;
extern volatile unsigned int pjsc_pwm_cur_value[13];
extern long pjsc_pwm_target_value[13];
extern volatile bool pjsc_pwm_state[13];

#define DISABLE_TIMER_FUEL1TO4(void) FUEL1_TIMER_DISABLE(void); FUEL2_TIMER_DISABLE(void); FUEL3_TIMER_DISABLE(void); FUEL4_TIMER_DISABLE(void)
//******************** [PJSC v1.10] ********************

#endif //PWM_H
