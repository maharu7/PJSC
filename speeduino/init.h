#ifndef INIT_H
#define INIT_H

void initialiseAll();
void initialiseTriggers();
void setPinMapping(byte);
void initialiseExternalTrigger();      //[PJSC] For External Trigger Interruot
void changeMapSelectSw();              //[PJSC v1.01] For MAP switch
void initialiseCaptureDutyPulse();     //[PJSC] For capturing duty pulse
void initialiseCaptureDutyPulse2();    //[PJSC] For capturing duty pulse

#endif