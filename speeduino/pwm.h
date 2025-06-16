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

extern byte pjscDuty[13];
extern unsigned int pjsc_pwm_max_count[13];
extern volatile unsigned int pjsc_pwm_cur_value[13];
extern long pjsc_pwm_target_value[13];
extern volatile bool pjsc_pwm_state[13];

#define DISABLE_TIMER_FUEL1TO4(void) FUEL1_TIMER_DISABLE(void); FUEL2_TIMER_DISABLE(void); FUEL3_TIMER_DISABLE(void); FUEL4_TIMER_DISABLE(void)
//******************** [PJSC v1.10] ********************

#endif //PWM_H
