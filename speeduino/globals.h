#ifndef GLOBALS_H
#define GLOBALS_H
#include <Arduino.h>
#include "table.h"

//These are configuration options for changing around the outputs that are used. THese are just the defaults and may be changed in the sections below based on the hardware in use. 
#define INJ_CHANNELS 4
#define IGN_CHANNELS 5

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define BOARD_DIGITAL_GPIO_PINS 54
  #define BOARD_NR_GPIO_PINS 62
  #define LED_BUILTIN 13
  #define CORE_AVR
  #define BOARD_H "board_avr2560.h"

  //#define TIMER5_MICROS

#elif defined(CORE_TEENSY)
  #define BOARD_H "board_teensy35.h"

#elif defined(STM32_MCU_SERIES) || defined(ARDUINO_ARCH_STM32) || defined(__STM32F1__) || defined(STM32F4) || defined(STM32)
  #define CORE_STM32
  #define BOARD_H "board_stm32.h"
  #ifndef word
    #define word(h, l) ((h << 8) | l) //word() function not defined for this platform in the main library
  #endif
  #if defined (STM32F1) || defined(__STM32F1__)
    #define BOARD_DIGITAL_GPIO_PINS 34
    #define BOARD_NR_GPIO_PINS 34
    #ifndef LED_BUILTIN
      #define LED_BUILTIN PB1 //Maple Mini
    #endif
  #elif defined(ARDUINO_BLACK_F407VE) || defined(STM32F4)
    #define BOARD_DIGITAL_GPIO_PINS 80
    #define BOARD_NR_GPIO_PINS 80
    #define LED_BUILTIN PA7

    //These boards always make 8/8 channels available
    #undef INJ_CHANNELS
    #undef IGN_CHANNELS
    #define INJ_CHANNELS 8
    #define IGN_CHANNELS 8
  #endif

  //Specific mode for Bluepill due to its small flash size. This disables a number of strings from being compiled into the flash
  #if defined(MCU_STM32F103C8) || defined(MCU_STM32F103CB)
    #define SMALL_FLASH_MODE
  #endif

  extern "C" char* sbrk(int incr); //Used to freeRam
  #if defined(ARDUINO_ARCH_STM32) // STM32GENERIC core
    inline unsigned char  digitalPinToInterrupt(unsigned char Interrupt_pin) { return Interrupt_pin; } //This isn't included in the stm32duino libs (yet)
    #define portOutputRegister(port) (volatile byte *)( &(port->ODR) )
    #define portInputRegister(port) (volatile byte *)( &(port->IDR) )
  #else //libmaple core aka STM32DUINO
    //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
    #define portOutputRegister(port) (volatile byte *)( &(port->regs->ODR) )
    #define portInputRegister(port) (volatile byte *)( &(port->regs->IDR) )
  #endif
#elif defined(__SAMD21G18A__)
  #define BOARD_H "board_samd21.h"
  #define CORE_SAMD21
#else
  #error Incorrect board selected. Please select the correct board (Usually Mega 2560) and upload again
#endif

//This can only be included after the above section
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1<<(pos)))

#define interruptSafe(c)  noInterrupts(); c interrupts(); //Wraps any code between nointerrupt and interrupt calls

#define MS_IN_MINUTE 60000
#define US_IN_MINUTE 60000000

//Define the load algorithm
#define LOAD_SOURCE_MAP         0
#define LOAD_SOURCE_TPS         1
#define LOAD_SOURCE_IMAPEMAP    2

//Define bit positions within engine virable
#define BIT_ENGINE_RUN      0   // Engine running
#define BIT_ENGINE_CRANK    1   // Engine cranking
#define BIT_ENGINE_ASE      2   // after start enrichment (ASE)
#define BIT_ENGINE_WARMUP   3   // Engine in warmup
#define BIT_ENGINE_ACC      4   // in acceleration mode (TPS accel)
#define BIT_ENGINE_DCC      5   // in deceleration mode
#define BIT_ENGINE_MAPACC   6   // MAP acceleration mode
#define BIT_ENGINE_MAPDCC   7   // MAP decelleration mode

//Define masks for Status1
#define BIT_STATUS1_INJ1           0  //inj1
#define BIT_STATUS1_INJ2           1  //inj2
#define BIT_STATUS1_INJ3           2  //inj3
#define BIT_STATUS1_INJ4           3  //inj4
#define BIT_STATUS1_DFCO           4  //Decelleration fuel cutoff
#define BIT_STATUS1_BOOSTCUT       5  //Fuel component of MAP based boost cut out
#define BIT_STATUS1_TOOTHLOG1READY 6  //Used to flag if tooth log 1 is ready
#define BIT_STATUS1_TOOTHLOG2READY 7  //Used to flag if tooth log 2 is ready (Log is not currently used)

//Define masks for spark variable
#define BIT_SPARK_HLAUNCH         0  //Hard Launch indicator
#define BIT_SPARK_SLAUNCH         1  //Soft Launch indicator
#define BIT_SPARK_HRDLIM          2  //Hard limiter indicator
#define BIT_SPARK_SFTLIM          3  //Soft limiter indicator
#define BIT_SPARK_BOOSTCUT        4  //Spark component of MAP based boost cut out
#define BIT_SPARK_ERROR           5  // Error is detected
#define BIT_SPARK_IDLE            6  // idle on
#define BIT_SPARK_SYNC            7  // Whether engine has sync or not

#define BIT_SPARK2_FLATSH         0  //Flat shift hard cut
#define BIT_SPARK2_FLATSS         1  //Flat shift soft cut
#define BIT_SPARK2_UNUSED3        2
#define BIT_SPARK2_UNUSED4        3
#define BIT_SPARK2_UNUSED5        4
#define BIT_SPARK2_UNUSED6        5
#define BIT_SPARK2_UNUSED7        6
#define BIT_SPARK2_UNUSED8        7

#define BIT_TIMER_1HZ             0
#define BIT_TIMER_4HZ             1
#define BIT_TIMER_10HZ            2
#define BIT_TIMER_15HZ            3
#define BIT_TIMER_30HZ            4

#define BIT_STATUS3_RESET_PREVENT 0 //Indicates whether reset prevention is enabled
#define BIT_STATUS3_NITROUS       1
#define BIT_STATUS3_UNUSED2       2
#define BIT_STATUS3_UNUSED3       3
#define BIT_STATUS3_UNUSED4       4
#define BIT_STATUS3_NSQUIRTS1     5
#define BIT_STATUS3_NSQUIRTS2     6
#define BIT_STATUS3_NSQUIRTS3     7

//[PJSC v1.01] Define masks for Test mode
#define BIT_TEST_ON               0  //ON/OFF
#define BIT_TEST_PWM              1  //Outputs PWM signal
#define BIT_TEST_PULSE            2  //Outputs Pulse

//[PJSC v1.01] Define masks for Test mode active
#define BIT_TEST_INJ1             0  //inj1
#define BIT_TEST_INJ2             1  //inj2
#define BIT_TEST_INJ3             2  //inj3
#define BIT_TEST_INJ4             3  //inj4
#define BIT_TEST_IGN1             4  //coil1
#define BIT_TEST_IGN2             5  //coil2
#define BIT_TEST_IGN3             6  //coil3
#define BIT_TEST_IGN4             7  //coil4

//[PJSC v1.01] Define argument for Injector channel
#define CH_INJ1                   0  //inj1
#define CH_INJ2                   1  //inj2
#define CH_INJ3                   2  //inj3
#define CH_INJ4                   3  //inj4
#define CH_IGN1                   4  //coil1
#define CH_IGN2                   5  //coil2
#define CH_IGN3                   6  //coli3
#define CH_IGN4                   7  //coil4
#define CH_MUX1                   8  //MUX output1
#define CH_MUX2                   9  //MUX output2
#define CH_MUX3                   10 //MUX output3
#define CH_MUX4                   11 //MUX output4

#define VALID_MAP_MAX 1022 //The largest ADC value that is valid for the MAP sensor
#define VALID_MAP_MIN 2 //The smallest ADC value that is valid for the MAP sensor

#define TOOTH_LOG_SIZE      64
#define TOOTH_LOG_BUFFER    128 //256

#define COMPOSITE_LOG_PRI   0
#define COMPOSITE_LOG_SEC   1
#define COMPOSITE_LOG_TRIG  2
#define COMPOSITE_LOG_SYNC  3

#define INJ_PAIRED          0
#define INJ_SEMISEQUENTIAL  1
#define INJ_BANKED          2
#define INJ_SEQUENTIAL      3

#define IGN_MODE_WASTED     0
#define IGN_MODE_SINGLE     1
#define IGN_MODE_WASTEDCOP  2
#define IGN_MODE_SEQUENTIAL 3
#define IGN_MODE_ROTARY     4

#define SEC_TRIGGER_SINGLE  0
#define SEC_TRIGGER_4_1     1

#define ROTARY_IGN_FC       0
#define ROTARY_IGN_FD       1
#define ROTARY_IGN_RX8      2

#define BOOST_MODE_SIMPLE   0
#define BOOST_MODE_FULL     1

#define HARD_CUT_FULL       0
#define HARD_CUT_ROLLING    1

#define SIZE_BYTE           8
#define SIZE_INT            16

#define EVEN_FIRE           0
#define ODD_FIRE            1

#define EGO_ALGORITHM_SIMPLE  0
#define EGO_ALGORITHM_PID     2

#define STAGING_MODE_TABLE  0
#define STAGING_MODE_AUTO   1

#define NITROUS_OFF         0
#define NITROUS_STAGE1      1
#define NITROUS_STAGE2      2

#define KNOCK_MODE_OFF      0
#define KNOCK_MODE_DIGITAL  1
#define KNOCK_MODE_ANALOG   2

#define RESET_CONTROL_DISABLED             0
#define RESET_CONTROL_PREVENT_WHEN_RUNNING 1
#define RESET_CONTROL_PREVENT_ALWAYS       2
#define RESET_CONTROL_SERIAL_COMMAND       3

#define OPEN_LOOP_BOOST     0
#define CLOSED_LOOP_BOOST   1

#define FOUR_STROKE         0
#define TWO_STROKE          1

#define MAX_RPM 18000 //This is the maximum rpm that the ECU will attempt to run at. It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be allowed to run. Lower number gives better performance
#define engineSquirtsPerCycle 2 //Would be 1 for a 2 stroke

//Table sizes
#define CALIBRATION_TABLE_SIZE 512
#define CALIBRATION_TEMPERATURE_OFFSET 40 // All temperature measurements are stored offset by 40 degrees. This is so we can use an unsigned byte (0-255) to represent temperature ranges from -40 to 215
#define OFFSET_FUELTRIM 127 //The fuel trim tables are offset by 128 to allow for -128 to +128 values
#define OFFSET_IGNITION 40 //Ignition values from the main spark table are offset 40 degrees downards to allow for negative spark timing

#define SERIAL_BUFFER_THRESHOLD 32 // When the serial buffer is filled to greater than this threshold value, the serial processing operations will be performed more urgently in order to avoid it overflowing. Serial buffer is 64 bytes long, so the threshold is set at half this as a reasonable figure

#define FUEL_PUMP_ON() *pump_pin_port |= (pump_pin_mask)
#define FUEL_PUMP_OFF() *pump_pin_port &= ~(pump_pin_mask)

const char TSfirmwareVersion[] PROGMEM = "Speeduino";

const byte data_structure_version = 2; //This identifies the data structure when reading / writing.
//const byte page_size = 64;
//[PJSC]const int16_t npage_size[11] PROGMEM = {0,288,128,288,128,288,128,240,192,192,192};
const int16_t npage_size[12] PROGMEM = {0,288,128,288,128,288,128,240,192,192,288,288};             //[PJSC v1.01]
//const byte page11_size = 128;
#define MAP_PAGE_SIZE 288

#define NUM_SQUIRT_DEVICE        4    //[PJSC]
#define SELECT_VE1               0    //[PJSC]
#define SELECT_VE2               1    //[PJSC]
#define SELECT_VE3               2    //[PJSC v1.01]
#define SELECT_VE4               3    //[PJSC v1.01]
#define MULTI_VE_COUNT           4    //[PJSC v1.01]
#define SINGLE_VE_COUNT          1    //[PJSC v1.01]

#define EXTRIG_SPARK_DISABLE     0    //[PJSC v1.01] For MAP switching
#define EXTRIG_SPARK_CAPTURE     1    // V
#define EXTRIG_MAP_SELECT        2    //[PJSC v1.01] For MAP switching
#define EXTRIG_MISFIRE_DETECTION 3    //[PJSC v1.03] For Misfire detection
#define EXTRIG_VIECLE_SPEED      4    //[PJSC v1.03] For capturing viecle speed

#define DIGITAL_INPUT2_DISABLE   0    //[PJSC v1.03]
#define DIGITAL_INPUT2_PWM_CAPT  1    //[PJSC v1.03]
#define DIGITAL_INPUT2_SHIFT     2    //[PJSC v1.03]

#define MUXOUT_OFF               0    //[PJSC v1.01] MUX output selection
#define MUXOUT_IDLE              1    //[PJSC v1.01] MUX output selection
#define MUXOUT_FAN               2    //[PJSC v1.01] MUX output selection
#define MUXOUT_LAUNCH            3    //[PJSC v1.01] MUX output selection
#define MUXOUT_FUELPUMP          4    //[PJSC v1.01] MUX output selection
#define MUXOUT_BOOST             5    //[PJSC v1.01] MUX output selection
#define MUXOUT_VVT               6    //[PJSC v1.01] MUX output selection
#define MUXOUT_TACH              7    //[PJSC v1.01] MUX output selection

#define ANALOG_INPUT_OFF         0    //[PJSC v1.02] Analog input selection
#define ANALOG_EXVALVE           1    //[PJSC v1.02] Analog input selection
#define ANALOG_O2_SEC            2    //[PJSC v1.02] Analog input selection
#define ANALOG_BARO              1    //[PJSC v1.03] Analog input selection2
#define ANALOG_EGT               2    //[PJSC v1.03] Analog input selection2

struct table3D fuelTable; //16x16 fuel map
struct table3D fuelTable2; //16x16 fuel map2 [PJSC]
struct table3D fuelTable3; //16x16 fuel map3 [PJSC v1.01]
struct table3D ignitionTable; //16x16 ignition map
struct table3D afrTable; //16x16 afr target map
struct table3D stagingTable; //8x8 fuel staging table
struct table3D boostTable; //8x8 boost map
struct table3D vvtTable; //8x8 vvt map
//[PJSC v1.01]struct table3D trim1Table; //6x6 Fuel trim 1 map
//[PJSC v1.01]struct table3D trim2Table; //6x6 Fuel trim 2 map
//[PJSC v1.01]struct table3D trim3Table; //6x6 Fuel trim 3 map
//[PJSC v1.01]struct table3D trim4Table; //6x6 Fuel trim 4 map
struct table2D taeTable; //4 bin TPS Acceleration Enrichment map (2D)
struct table2D WUETable; //10 bin Warm Up Enrichment map (2D)
struct table2D crankingEnrichTable; //4 bin cranking Enrichment map (2D)
struct table2D dwellVCorrectionTable; //6 bin dwell voltage correction (2D)
struct table2D injectorVCorrectionTable; //6 bin injector voltage correction (2D)
struct table2D IATDensityCorrectionTable; //9 bin inlet air temperature density correction (2D)
struct table2D IATRetardTable; //6 bin ignition adjustment based on inlet air temperature  (2D)
struct table2D rotarySplitTable; //8 bin ignition split curve for rotary leading/trailing  (2D)
struct table2D flexFuelTable;  //6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D flexAdvTable;   //6 bin flex fuel correction table for timing advance (2D)
struct table2D flexBoostTable; //6 bin flex fuel correction table for boost adjustments (2D)
struct table2D knockWindowStartTable;
struct table2D knockWindowDurationTable;
struct table2D barometricCorrectionTable; //[PJSC v1.03]9 bin barometric pressure correction (2D)

//These are for the direct port manipulation of the injectors, coils and aux outputs
volatile PORT_TYPE *inj1_pin_port;
volatile byte inj1_pin_mask;
volatile PORT_TYPE *inj2_pin_port;
volatile byte inj2_pin_mask;
volatile PORT_TYPE *inj3_pin_port;
volatile byte inj3_pin_mask;
volatile PORT_TYPE *inj4_pin_port;
volatile byte inj4_pin_mask;
volatile PORT_TYPE *inj5_pin_port;
volatile byte inj5_pin_mask;
volatile PORT_TYPE *inj6_pin_port;
volatile byte inj6_pin_mask;
volatile PORT_TYPE *inj7_pin_port;
volatile byte inj7_pin_mask;
volatile PORT_TYPE *inj8_pin_port;
volatile byte inj8_pin_mask;

volatile PORT_TYPE *ign1_pin_port;
volatile byte ign1_pin_mask;
volatile PORT_TYPE *ign2_pin_port;
volatile byte ign2_pin_mask;
volatile PORT_TYPE *ign3_pin_port;
volatile byte ign3_pin_mask;
volatile PORT_TYPE *ign4_pin_port;
volatile byte ign4_pin_mask;
volatile PORT_TYPE *ign5_pin_port;
volatile byte ign5_pin_mask;
volatile PORT_TYPE *ign6_pin_port;
volatile byte ign6_pin_mask;
volatile PORT_TYPE *ign7_pin_port;
volatile byte ign7_pin_mask;
volatile PORT_TYPE *ign8_pin_port;
volatile byte ign8_pin_mask;

volatile PORT_TYPE *tach_pin_port;
volatile byte tach_pin_mask;
volatile PORT_TYPE *pump_pin_port;
volatile byte pump_pin_mask;

volatile PORT_TYPE *triggerPri_pin_port;
volatile byte triggerPri_pin_mask;
volatile PORT_TYPE *triggerSec_pin_port;
volatile byte triggerSec_pin_mask;

volatile byte captureDutyPulseInterrupt;     //[PJSC] For capture duty pulse
volatile byte captureDutyPulseInterrupt2;    //[PJSC] For capture duty pulse
volatile byte tempVEvalue[4];                //[PJSC v1.01] Multi VE Map support
volatile unsigned int dualFuelLoadVE;        //[PJSC v1.01] Dual Fuel Load support

//These need to be here as they are used in both speeduino.ino and scheduler.ino
bool channel1InjEnabled = true;
bool channel2InjEnabled = false;
bool channel3InjEnabled = false;
bool channel4InjEnabled = false;
bool channel5InjEnabled = false;
bool channel6InjEnabled = false;
bool channel7InjEnabled = false;
bool channel8InjEnabled = false;

int ignition1EndAngle = 0;
int ignition2EndAngle = 0;
int ignition3EndAngle = 0;
int ignition4EndAngle = 0;
int ignition5EndAngle = 0;

//These are variables used across multiple files
bool initialisationComplete = false; //Tracks whether the setup() function has run completely
volatile uint16_t mainLoopCount;
unsigned long revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
volatile unsigned long timer5_overflow_count = 0; //Increments every time counter 5 overflows. Used for the fast version of micros()
volatile unsigned long ms_counter = 0; //A counter that increments once per ms
uint16_t fixedCrankingOverride = 0;
bool clutchTrigger;
bool previousClutchTrigger;
volatile uint32_t toothHistory[TOOTH_LOG_BUFFER];
volatile uint8_t compositeLogHistory[TOOTH_LOG_BUFFER];
volatile bool fpPrimed = false; //Tracks whether or not the fuel pump priming has been completed yet
volatile unsigned int toothHistoryIndex = 0;
volatile byte toothHistorySerialIndex = 0;
byte primaryTriggerEdge;
byte secondaryTriggerEdge;
int CRANK_ANGLE_MAX = 720;
int CRANK_ANGLE_MAX_IGN = 360;
int CRANK_ANGLE_MAX_INJ = 360; //The number of crank degrees that the system track over. 360 for wasted / timed batch and 720 for sequential


//This needs to be here because using the config page directly can prevent burning the setting
byte resetControl = RESET_CONTROL_DISABLED;

volatile byte TIMER_mask;
volatile byte LOOP_TIMER;

//The status struct contains the current values for all 'live' variables
//In current version this is 64 bytes
struct statuses {
  volatile bool hasSync;
  uint16_t RPM;
  long longRPM;
  int mapADC;
  int baroADC;
  long MAP; //Has to be a long for PID calcs (Boost control)
  int16_t EMAP;
  int16_t EMAPADC;
//[PJSC v1.01]  byte baro; //Barometric pressure is simply the inital MAP reading, taken before the engine is running. Alternatively, can be taken from an external sensor
  int baro; //[PJSC v1.01] Barometric pressure is simply the inital MAP reading, taken before the engine is running. Alternatively, can be taken from an external sensor
  byte TPS; //The current TPS reading (0% - 100%)
  byte TPSlast; //The previous TPS reading
  unsigned long TPS_time; //The time the TPS sample was taken
  unsigned long TPSlast_time; //The time the previous TPS sample was taken
  byte tpsADC; //0-255 byte representation of the TPS
//[PJSC v1.03]  byte tpsDOT;
  int tpsDOT;  //[PJSC v1.03]
  volatile int rpmDOT;
  byte VE;
  byte VE2;    //[PJSC]
  byte VE3;    //[PJSC v1.01]
  byte VE4;    //[PJSC v1.01]
  byte O2;
  byte O2_2;
  int coolant;
  int cltADC;
  int IAT;
  int iatADC;
  int batADC;
  int O2ADC;
  int O2_2ADC;
  int dwell;
  byte dwellCorrection; //The amount of correction being applied to the dwell time.
  byte battery10; //The current BRV in volts (multiplied by 10. Eg 12.5V = 125)
  int8_t advance; //Signed 8 bit as advance can now go negative (ATDC)
//[PJSC v1.03]  byte corrections;
  uint16_t corrections;      //[PJSC v1.03]
  int16_t TAEamount; //The amount of accleration enrichment currently being applied
  byte egoCorrection; //The amount of closed loop AFR enrichment currently being applied
  byte wueCorrection; //The amount of warmup enrichment currently being applied
  byte batCorrection; //The amount of battery voltage enrichment currently being applied
  byte iatCorrection; //The amount of inlet air temperature adjustment currently being applied
  byte launchCorrection; //The amount of correction being applied if launch control is active
  byte flexCorrection; //Amount of correction being applied to compensate for ethanol content
  int8_t flexIgnCorrection; //Amount of additional advance being applied based on flex. Note the type as this allows for negative values
  byte baroCorrection; //[PJSC v1.03]The amount of barometric pressure adjustment currently being applied
  byte afrTarget;
  byte idleDuty;
  bool idleUpActive;
  bool fanOn; //Whether or not the fan is turned on
  volatile byte ethanolPct; //Ethanol reading (if enabled). 0 = No ethanol, 100 = pure ethanol. Eg E85 = 85.
  unsigned long TAEEndTime; //The target end time used whenever TAE is turned on
  volatile byte status1;
  volatile byte spark;
  volatile byte spark2;
  byte engine;
  unsigned int PW1; //In uS
  unsigned int PW2; //In uS
  unsigned int PW3; //In uS
  unsigned int PW4; //In uS
  unsigned int PW5; //In uS
  unsigned int PW6; //In uS
  unsigned int PW7; //In uS
  unsigned int PW8; //In uS
  volatile byte runSecs; //Counter of seconds since cranking commenced (overflows at 255 obviously)
  volatile byte secl; //Continous
  volatile unsigned int loopsPerSecond;
  boolean launchingSoft; //True when in launch control soft limit mode
  boolean launchingHard; //True when in launch control hard limit mode
  uint16_t freeRAM;
  unsigned int clutchEngagedRPM;
  bool flatShiftingHard;
  volatile uint32_t startRevolutions; //A counter for how many revolutions have been completed since sync was achieved.
  uint16_t boostTarget;
  byte testOutputs;
  bool testActive;
  uint16_t boostDuty; //Percentage value * 100 to give 2 points of precision
  byte idleLoad; //Either the current steps or current duty cycle for the idle control.
  uint16_t canin[16];   //16bit raw value of selected canin data for channel 0-15
  uint8_t current_caninchannel = 0; //start off at channel 0
  uint16_t crankRPM = 400; //The actual cranking RPM limit. Saves us multiplying it everytime from the config page
  volatile byte status3;
  int16_t flexBoostCorrection; //Amount of boost added based on flex
  byte nitrous_status;
  byte nSquirts;
  byte nChannels; //Number of fuel and ignition channels
  int16_t fuelLoad;
  int16_t fuelLoad2;               //[PJSC v1.01] For Secondary Fuel Load
  int16_t fuelLoad3;               //[PJSC v1.01] For x4 VE table support
  int16_t fuelLoad4;               //[PJSC v1.01] For x4 VE table support
  int16_t ignLoad;
  bool fuelPumpOn; //The current status of the fuel pump
  byte syncLossCounter;
  byte knockRetard;
  bool knockActive;
  bool toothLogEnabled;
  bool compositeLogEnabled;
  byte exValvePosition;                //[PJSC] For External Trigger
  byte exValvePositionADC;             //[PJSC] For External Trigger
  int extTriggerAngle;                 //[PJSC] For External Trigger
  int extTriggerAngle_last;            //[PJSC] For External Trigger
  uint16_t extTriggerRPM;              //[PJSC v1.03] For External Trigger
  uint16_t extTriggerLoad;             //[PJSC v1.03] For External Trigger
  byte dutyCaptureCount;               //[PJSC] For capturing duty pulse
  byte dutyCaptureCount2;              //[PJSC] For capturing duty pulse
  int dutyFreq;                        //[PJSC] For capturing duty pulse
  int dutyFreq2;                       //[PJSC] For capturing duty pulse
  int dutyRatio;                       //[PJSC] For capturing duty pulse
  int dutyRatio2;                      //[PJSC] For capturing duty pulse
  unsigned long dutyON_time = 0;       //[PJSC] For capturing duty pulse
  unsigned long dutyONlast_time = 0;   //[PJSC] For capturing duty pulse
  unsigned long dutyOFF_time = 0;      //[PJSC] For capturing duty pulse
  unsigned long dutyOFFlast_time = 0;  //[PJSC] For capturing duty pulse
  unsigned long cycle_t;               //[PJSC] For capturing duty pulse
  unsigned long on_t;                  //[PJSC] For capturing duty pulse
  unsigned long dutyON_time2 = 0;      //[PJSC] For capturing duty pulse
  unsigned long dutyONlast_time2 = 0;  //[PJSC] For capturing duty pulse
  unsigned long dutyOFF_time2 = 0;     //[PJSC] For capturing duty pulse
  unsigned long dutyOFFlast_time2 = 0; //[PJSC] For capturing duty pulse
  unsigned long cycle_t2;              //[PJSC] For capturing duty pulse
  unsigned long on_t2;                 //[PJSC] For capturing duty pulse
  unsigned long ignGap = 0;            //[PJSC v1.03] For misfire detection
  byte testMode;                       //[PJSC v1.01] For test mode
  byte testModeActive;                 //[PJSC v1.01] For test mode
  int16_t testCnt;                     //[PJSC v1.01] For test mode
  boolean mapSelectSw;                 //[PJSC v1.01] For MAP switching
  byte veMapSelectionSw1Pri[4];        //[PJSC v1.01] For x4 Fuel table support
  byte veMapSelectionSw1Sec[4];        //[PJSC v1.01]  |
  byte veMapSelectionSw2Pri[4];        //[PJSC v1.01]  V
  byte veMapSelectionSw2Sec[4];        //[PJSC v1.01] For x4 Fuel table support
  byte afr_analyze1;                   //[PJSC v1.02] For AFR sensor selection
  byte afr_analyze2;                   // |
  byte afr_analyze3;                   // V
  byte afr_analyze4;                   //[PJSC v1.02] For AFR sensor selection
  byte dualVE1;                        //[PJSC v1.03] For Dual Fuel Load
  byte dualVE2;                        //[PJSC v1.03] For Dual Fuel Load
  byte dualVE3;                        //[PJSC v1.03] For Dual Fuel Load
  byte dualVE4;                        //[PJSC v1.03] For Dual Fuel Load
  int EGTADC;                          //[PJSC v1.03] For Exhaust Gas Temperature input
  uint16_t sparkRPM;                   //[PJSC v1.03] For misfire detection
  uint16_t viecleSpeed;                //[PJSC v1.03] For capturing viecle speed

  //Helpful bitwise operations:
  //Useful reference: http://playground.arduino.cc/Code/BitMath
  // y = (x >> n) & 1;    // n=0..15.  stores nth bit of x in y.  y becomes 0 or 1.
  // x &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
  // x |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.

};
struct statuses currentStatus; //The global status object

//Page 1 of the config - See the ini file for further reference
//This mostly covers off variables that are required for fuel
struct config2 {

  byte unused2_1;
  byte unused2_2;
  byte asePct;  //Afterstart enrichment (%)
  byte aseCount; //Afterstart enrichment cycles. This is the number of ignition cycles that the afterstart enrichment % lasts for
  byte wueValues[10]; //Warm up enrichment array (10 bytes)
  byte crankingPct; //Cranking enrichment
  byte pinMapping; // The board / ping mapping to be used
  byte tachoPin : 6; //Custom pin setting for tacho output
  byte tachoDiv : 2; //Whether to change the tacho speed
  byte unused2_17;
  byte unused2_18;
  byte tpsThresh;
  byte taeTime;

  //Display config bits
  byte displayType : 3; //21
  byte display1 : 3;
  byte display2 : 2;

  byte display3 : 3;    //22
  byte display4 : 2;
  byte display5 : 3;

  byte displayB1 : 4;   //23
  byte displayB2 : 4;

  byte reqFuel;       //24
  byte divider;
  byte injTiming : 1;
  byte multiplyMAP : 1;
  byte includeAFR : 1;
  byte hardCutType : 1;
  byte ignAlgorithm : 3;
  byte indInjAng : 1;
  byte injOpen; //Injector opening time (ms * 10)
  uint16_t inj1Ang;
  uint16_t inj2Ang;
  uint16_t inj3Ang;
  uint16_t inj4Ang;

  //config1 in ini
  byte mapSample : 2;
  byte strokes : 1;
  byte injType : 1;
  byte nCylinders : 4; //Number of cylinders

  //config2 in ini
  byte fuelAlgorithm : 3;
  byte fixAngEnable : 1; //Whether fixed/locked timing is enabled
  byte nInjectors : 4; //Number of injectors


  //config3 in ini
  byte engineType : 1;
  byte flexEnabled : 1;
  byte unused2_38c : 1; //"Speed Density", "Alpha-N"
  byte baroCorr : 1;
  byte injLayout : 2;
  byte perToothIgn : 1;
  byte dfcoEnabled : 1; //Whether or not DFCO is turned on

  byte primePulse;
  byte dutyLim;
  byte flexFreqLow; //Lowest valid frequency reading from the flex sensor
  byte flexFreqHigh; //Highest valid frequency reading from the flex sensor

  byte boostMaxDuty;
  byte tpsMin;
  byte tpsMax;
  int8_t mapMin; //Must be signed
  uint16_t mapMax;
  byte fpPrime; //Time (In seconds) that the fuel pump should be primed for on power up
  byte stoich;
  uint16_t oddfire2; //The ATDC angle of channel 2 for oddfire
  uint16_t oddfire3; //The ATDC angle of channel 3 for oddfire
  uint16_t oddfire4; //The ATDC angle of channel 4 for oddfire

  byte idleUpPin : 6;
  byte idleUpPolarity : 1;
  byte idleUpEnabled : 1;

  byte idleUpAdder;
  byte taeTaperMin;
  byte taeTaperMax;

  byte iacCLminDuty;
  byte iacCLmaxDuty;
  byte boostMinDuty;

  int8_t baroMin; //Must be signed
  uint16_t baroMax;

  int8_t EMAPMin; //Must be signed
  uint16_t EMAPMax;

  byte fanWhenOff : 1;               // Only run fan when engine is running
  byte dfcoTPSdotEnabled : 1;        //[PJSC v1.03] For TPSdotDFCO
  byte swIATcorrection: 1;           //[PJSCv1.03]
  byte swTAE: 1;                     // |
  byte swWUE: 1;                     // V
  byte swCrankingEnrichment: 1;      //[PJSCv1.03]
  byte fanUnused : 2;                //[PJSC v1.03] For TPSdotDFCO
//[PJSC v1.03]  byte fanUnused : 7;

  //[PJSC]  byte unused1_70[57];
  byte pjscFreq;                     //[PJSC]    Offset 71
  byte exValvePosMin;                // |
  byte exValvePosMax;                // |
  byte unused2_74;                   // |
  byte squirtDeviceType : 1;         // |
  byte multiVEmapEnabled: 1;         // |
  byte mapSeparationEnabled: 1;      // |
  byte mapSwitchingEnabled: 1;       // |
  byte dualFuelEnabled: 1;           // |
  byte secondaryFuelUsage: 1;        // |
//[PJSC v1.03]  byte fuelCorrectionEnabled: 1;     // |[PJSC v1.01]
//[PJSC v1.03]  byte unused2_75: 1;                // |
  byte swBatVCorrection: 1;          // |[PJSC v1.03]
  byte vvtSamplingRate: 1;           // |[PJSC v1.03]
//[PJSC v1.03]  byte exTrigModeSelect : 2;         // | For External Trigger
  byte exTrigModeSelect : 3;         // | [PJSC v1.03] For capturing viecle speed
  byte externalTrigEdge: 1;          // |  0: Rising, 1: Falling
//[PJSC v1.02]  byte exValveCaptureEnabled: 1;     // | For capturing Exhaust valve position
//[PJSC v1.03]  byte exValveCalibrationMode: 1;    // | For support Exhaust Valve calibrationmode
//[PJSC v1.02]  byte unused2_76: 3;                // |
  byte analogInputPortSelection: 2;  // |[PJSC v1.02] For Analog input port selection
  byte analogInputPortSelection2: 2; // |[PJSC v1.03]
  byte dutyPulseCaptureEnabled: 2;   // | For capturing duty pulse ch1
  byte dutyPulseCaptureEnabled2: 2;  // | For capturing duty pulse ch2
  byte dutyPulseOnLevel: 1;          // | For capturing duty pulse ch1, 0: high, 1: low
  byte dutyPulseOnLevel2: 1;         // | For capturing duty pulse ch2, 0: high, 1: low
  byte unused2_77: 2;                // V
  byte veMapSelectionInj1Pri: 4;     //[PJSCv1.01] For x4 Fuel table support
  byte veMapSelectionInj1Sec: 4;     //[PJSCv1.01]  |
  byte veMapSelectionInj2Pri: 4;     //[PJSCv1.01]  |
  byte veMapSelectionInj2Sec: 4;     //[PJSCv1.01]  |
  byte veMapSelectionInj3Pri: 4;     //[PJSCv1.01]  |
  byte veMapSelectionInj3Sec: 4;     //[PJSCv1.01]  |
  byte veMapSelectionInj4Pri: 4;     //[PJSCv1.01]  |
  byte veMapSelectionInj4Sec: 4;     //[PJSCv1.01]  |
  byte veMapSelectionInj1_2Pri: 4;   //[PJSCv1.01]  |
  byte veMapSelectionInj1_2Sec: 4;   //[PJSCv1.01]  |
  byte veMapSelectionInj2_2Pri: 4;   //[PJSCv1.01]  |
  byte veMapSelectionInj2_2Sec: 4;   //[PJSCv1.01]  |
  byte veMapSelectionInj3_2Pri: 4;   //[PJSCv1.01]  |
  byte veMapSelectionInj3_2Sec: 4;   //[PJSCv1.01]  |
  byte veMapSelectionInj4_2Pri: 4;   //[PJSCv1.01]  V
  byte veMapSelectionInj4_2Sec: 4;   //[PJSCv1.01] For x4 Fuel table support
//[PJSC v1.01]  byte dutyFreqTst[4];               //[PJSC v1.01] For test mode
//[PJSC v1.01]  byte dutyRatioTst[4];              // |
  byte dutyFreqTst[12];              //[PJSC v1.01] For test mode
  byte dutyRatioTst[12];             // |
  byte testop_fp: 1;                 // |
  byte testop_inj: 2;                // |
  byte testop_coil: 2;               // |
  byte testsel_inj: 2;               // |
  byte testsel_coil: 1;              // |
  uint16_t testint;                  // |
  uint16_t testpw;                   // |
  uint16_t testinjcnt;               // |
  byte muxout1Selection: 4;          // | MUX output1 selection
  byte muxout2Selection: 4;          // | MUX output2 selection
  byte muxout3Selection: 4;          // |[PJSC v1.01] MUX output3 selection
  byte muxout4Selection: 4;          // |[PJSC v1.01] MUX output4 selection
  byte fuelAlgorithm2: 3;            // |[PJSC v1.01] For Secondary Fuel Algorithm
  byte fuelAlgorithm3: 3;            // |
//[PJSC v1.03]  byte unused2_118: 2;               // |
  byte swASE: 1;                     // |[PJSC v1.03]
  byte exValveCalibrationMode: 1;    // |[PJSC v1.03] For capturing viecle speed
//[PJSC v1.03]  byte unused2_118: 1;               // |[PJSC v1.03]
  byte table4Usage: 1;               // | For switching usage of 3rd table Ignition/Fuel
  byte useMAPasSync: 1;              // |[PJSC v1.03]
  byte isolateNumTooth: 1;           // |[PJSC v1.03] For trigger wheels with different number of triggers and number of cylinders
  byte triggerWheelSelectable: 1;    // |[PJSC v1.03]
  byte unused2_119: 4;               // |
  byte inj1SquirtStartEnd: 1;        // |
  byte inj2SquirtStartEnd: 1;        // |
  byte inj3SquirtStartEnd: 1;        // V
  byte inj4SquirtStartEnd: 1;        //[PJSC v1.01]
  byte afr_sensor_selection1: 1;     //[PJSC v1.02]
  byte afr_sensor_selection2: 1;     //[PJSC v1.02]
  byte afr_sensor_selection3: 1;     //[PJSC v1.02]
  byte afr_sensor_selection4: 1;     //[PJSC v1.02]
//[PJSC v1.03]  byte unused2_122[6];               //[PJSC v1.01]
  byte squirtDeviceTypeCh1 : 1;       //[PJSC v1.03]
  byte squirtDeviceTypeCh2 : 1;       // |
  byte squirtDeviceTypeCh3 : 1;       // |
  byte squirtDeviceTypeCh4 : 1;       // |
  byte solenoidValveDirectionCh1 : 1; // |
  byte solenoidValveDirectionCh2 : 1; // |
  byte solenoidValveDirectionCh3 : 1; // |
  byte solenoidValveDirectionCh4 : 1; // |
  byte EGTvoltage1;                   // |
  byte EGTvoltage2;                   // |
  int8_t EGTtemperature1;             // V
  uint16_t EGTtemperature2;           //[PJSC v1.03]

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bi systems require all structs to be fully packed
#endif

//Page 4 of the config - See the ini file for further reference
//This mostly covers off variables that are required for ignition
struct config4 {

  int16_t triggerAngle;
  int8_t FixAng; //Negative values allowed
  byte CrankAng;
  byte TrigAngMul; //Multiplier for non evenly divisible tooth counts.

  byte TrigEdge : 1;
  byte TrigSpeed : 1;
  byte IgInv : 1;
  byte TrigPattern : 5;

  byte TrigEdgeSec : 1;
  byte fuelPumpPin : 6;
  byte useResync : 1;

  byte sparkDur; //Spark duration in ms * 10
  byte trigPatternSec; //Mode for Missing tooth secondary trigger.  Either single tooth cam wheel or 4-1
  uint8_t bootloaderCaps; //Capabilities of the bootloader over stock. e.g., 0=Stock, 1=Reset protection, etc.

  byte resetControl : 2; //Which method of reset control to use (0=None, 1=Prevent When Running, 2=Prevent Always, 3=Serial Command)
  byte resetControlPin : 6;

  byte StgCycles; //The number of initial cycles before the ignition should fire when first cranking

  byte boostType : 1; //Open or closed loop boost control
  byte useDwellLim : 1; //Whether the dwell limiter is off or on
  byte sparkMode : 3; //Spark output mode (Eg Wasted spark, single channel or Wasted COP)
  byte triggerFilter : 2; //The mode of trigger filter being used (0=Off, 1=Light (Not currently used), 2=Normal, 3=Aggressive)
  byte ignCranklock : 1; //Whether or not the ignition timing during cranking is locked to a CAS pulse. Only currently valid for Basic distributor and 4G63.

  byte dwellCrank; //Dwell time whilst cranking
  byte dwellRun; //Dwell time whilst running
  byte triggerTeeth; //The full count of teeth on the trigger wheel if there were no gaps
  byte triggerMissingTeeth; //The size of the tooth gap (ie number of missing teeth)
  byte crankRPM; //RPM below which the engine is considered to be cranking
  byte floodClear; //TPS value that triggers flood clear mode (No fuel whilst cranking)
  byte SoftRevLim; //Soft rev limit (RPM/100)
  byte SoftLimRetard; //Amount soft limit retards (degrees)
  byte SoftLimMax; //Time the soft limit can run
  byte HardRevLim; //Hard rev limit (RPM/100)
  byte taeBins[4]; //TPS based acceleration enrichment bins (%/s)
  byte taeValues[4]; //TPS based acceleration enrichment rates (% to add)
  byte wueBins[10]; //Warmup Enrichment bins (Values are in configTable1)
  byte dwellLimit;
  byte dwellCorrectionValues[6]; //Correction table for dwell vs battery voltage
  byte iatRetBins[6]; // Inlet Air Temp timing retard curve bins
  byte iatRetValues[6]; // Inlet Air Temp timing retard curve values
  byte dfcoRPM; //RPM at which DFCO turns off/on at
  byte dfcoHyster; //Hysteris RPM for DFCO
  byte dfcoTPSThresh; //TPS must be below this figure for DFCO to engage

  byte ignBypassEnabled : 1; //Whether or not the ignition bypass is enabled
  byte ignBypassPin : 6; //Pin the ignition bypass is activated on
  byte ignBypassHiLo : 1; //Whether this should be active high or low.

  byte ADCFILTER_TPS;
  byte ADCFILTER_CLT;
  byte ADCFILTER_IAT;
  byte ADCFILTER_O2;
  byte ADCFILTER_BAT;
  byte ADCFILTER_MAP; //This is only used on Instantaneous MAP readings and is intentionally very weak to allow for faster response
  byte ADCFILTER_BARO;

//[PJSC v1.01]  byte unused2_64[57];
  byte baroDenBins[9];               //[PJSC v1.01] For Barometric extend correnction
  byte baroDenRates[9];              //[PJSC v1.01] For Barometric extend correnction
//[PJSC v1.02]  byte unused2_82[39];
  byte crankingFilter : 1;           //[PJSC v1.02]
  byte unused2_89 : 7;               //[PJSC v1.02]
  byte numSparkPerRev;               //[PJSC v1.03] For misfire detection
  byte misfireDetectThresh;          //[PJSC v1.03] For misfire detection
  byte numSpeedPulsePerRev;          //[PJSC v1.03] For capturing viecle speed
  uint16_t tireCircumference;        //[PJSC v1.03] For capturing viecle speed

  int16_t dfcoTPSdotThresh;          //[PJSC v1.03] For TPSdot DFCO
  byte dfcoTPSdotMulti;              //[PJSC v1.03] For TPSdot DFCO
  byte dfcoTPSdotDuration;           //[PJSC v1.03] For TPSdot DFCO
  byte dfcoTPSdotRPM;                //[PJSC v1.03] For TPSdot DFCO
  byte dfcoTPSdotTPSThresh;          //[PJSC v1.03] For TPSdot DFCO
  byte unused2_101[26];              //[PJSC v1.03] For TPSdot DFCO

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bi systems require all structs to be fully packed
#endif

//Page 6 of the config - See the ini file for further reference
//This mostly covers off variables that are required for AFR targets and closed loop
struct config6 {

  byte egoAlgorithm : 2;
  byte egoType : 2;
  byte boostEnabled : 1;
  byte vvtEnabled : 1;
  byte boostCutType : 2;

  byte egoKP;
  byte egoKI;
  byte egoKD;
  byte egoTemp; //The temperature above which closed loop functions
  byte egoCount; //The number of ignition cylces per step
  byte unused6_6;
  byte egoLimit; //Maximum amount the closed loop will vary the fueling
  byte ego_min; //AFR must be above this for closed loop to function
  byte ego_max; //AFR must be below this for closed loop to function
  byte ego_sdelay; //Time in seconds after engine starts that closed loop becomes available
  byte egoRPM; //RPM must be above this for closed loop to function
  byte egoTPSMax; //TPS must be below this for closed loop to function
  byte vvtPin : 6;
  byte useExtBaro : 1;
  byte boostMode : 1; //Simple of full boost control
  byte boostPin : 6;
  byte VVTasOnOff : 1; //Whether or not to use the VVT table as an on/off map
  byte useEMAP : 1;
  byte voltageCorrectionBins[6]; //X axis bins for voltage correction tables
  byte injVoltageCorrectionValues[6]; //Correction table for injector PW vs battery voltage
  byte airDenBins[9];
  byte airDenRates[9];
  byte boostFreq; //Frequency of the boost PWM valve
  byte vvtFreq; //Frequency of the vvt PWM valve
  byte idleFreq;

  byte launchPin : 6;
  byte launchEnabled : 1;
  byte launchHiLo : 1;

  byte lnchSoftLim;
  int8_t lnchRetard; //Allow for negative advance value (ATDC)
  byte lnchHardLim;
  byte lnchFuelAdd;

  //PID values for idle needed to go here as out of room in the idle page
  byte idleKP;
  byte idleKI;
  byte idleKD;

  byte boostLimit; //Is divided by 2, allowing kPa values up to 511
  byte boostKP;
  byte boostKI;
  byte boostKD;

  byte lnchPullRes : 2;
  byte fuelTrimEnabled : 1;
  byte flatSEnable : 1;
  byte baroPin : 4;
  byte flatSSoftWin;
  byte flatSRetard;
  byte flatSArm;

  byte iacCLValues[10]; //Closed loop target RPM value
  byte iacOLStepVal[10]; //Open loop step values for stepper motors
  byte iacOLPWMVal[10]; //Open loop duty values for PMWM valves
  byte iacBins[10]; //Temperature Bins for the above 3 curves
  byte iacCrankSteps[4]; //Steps to use when cranking (Stepper motor)
  byte iacCrankDuty[4]; //Duty cycle to use on PWM valves when cranking
  byte iacCrankBins[4]; //Temperature Bins for the above 2 curves

  byte iacAlgorithm : 3; //Valid values are: "None", "On/Off", "PWM", "PWM Closed Loop", "Stepper", "Stepper Closed Loop"
  byte iacStepTime : 3; //How long to pulse the stepper for to ensure the step completes (ms)
  byte iacChannels : 1; //How many outputs to use in PWM mode (0 = 1 channel, 1 = 2 channels)
  byte iacPWMdir : 1; //Direction of the PWM valve. 0 = Normal = Higher RPM with more duty. 1 = Reverse = Lower RPM with more duty

  byte iacFastTemp; //Fast idle temp when using a simple on/off valve

  byte iacStepHome; //When using a stepper motor, the number of steps to be taken on startup to home the motor
  byte iacStepHyster; //Hysteresis temperature (*10). Eg 2.2C = 22

  byte fanInv : 1;        // Fan output inversion bit
  byte fanEnable : 1;     // Fan enable bit. 0=Off, 1=On/Off
  byte fanPin : 6;
  byte fanSP;             // Cooling fan start temperature
  byte fanHyster;         // Fan hysteresis
  byte fanFreq;           // Fan PWM frequency
  byte fanPWMBins[4];     //Temperature Bins for the PWM fan control

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

//Page 9 of the config mostly deals with CANBUS control
//See ini file for further info (Config Page 10 in the ini)
struct config9 {
  byte enable_secondarySerial:1;            //enable secondary serial
  byte intcan_available:1;                     //enable internal can module
  byte enable_intcan:1;
  byte caninput_sel[16];                    //bit status on/Can/analog_local/digtal_local if input is enabled
  uint16_t caninput_source_can_address[16];        //u16 [15] array holding can address of input
  uint8_t caninput_source_start_byte[16];     //u08 [15] array holds the start byte number(value of 0-7)
  uint16_t caninput_source_num_bytes;     //u16 bit status of the number of bytes length 1 or 2
  byte unused10_67;
  byte unused10_68;
  byte enable_candata_out : 1;
  byte canoutput_sel[8];
  uint16_t canoutput_param_group[8];
  uint8_t canoutput_param_start_byte[8];
  byte canoutput_param_num_bytes[8];

  byte unused10_110;
  byte unused10_111;
  byte unused10_112;
  byte unused10_113;
  byte speeduino_tsCanId:4;         //speeduino TS canid (0-14)
  uint16_t true_address;            //speeduino 11bit can address
  uint16_t realtime_base_address;   //speeduino 11 bit realtime base address
  uint16_t obd_address;             //speeduino OBD diagnostic address
  uint8_t Auxinpina[16];            //analog  pin number when internal aux in use
  uint8_t Auxinpinb[16];            // digital pin number when internal aux in use

  byte iacStepperInv : 1;  //stepper direction of travel to allow reversing. 0=normal, 1=inverted.

  byte unused10_153;
  byte unused10_154;
  byte unused10_155;
  byte unused10_157;
  byte unused10_158;
  byte unused10_159;
  byte unused10_160;
  byte unused10_161;
  byte unused10_162;
  byte unused10_163;
  byte unused10_164;
  byte unused10_165;
  byte unused10_166;
  byte unused10_167;
  byte unused10_168;
  byte unused10_169;
  byte unused10_170;
  byte unused10_171;
  byte unused10_172;
  byte unused10_173;
  byte unused10_174;
  byte unused10_175;
  byte unused10_176;
  byte unused10_177;
  byte unused10_178;
  byte unused10_179;
  byte unused10_180;
  byte unused10_181;
  byte unused10_182;
  byte unused10_183;
  byte unused10_184;
  byte unused10_185;
  byte unused10_186;
  byte unused10_187;
  byte unused10_188;
  byte unused10_189;
  byte unused10_190;
  byte unused10_191;

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

/*
Page 10 - No specific purpose. Created initially for the cranking enrich curve
192 bytes long
See ini file for further info (Config Page 11 in the ini)
*/
struct config10 {
  byte crankingEnrichBins[4];
  byte crankingEnrichValues[4];

  byte rotaryType : 2;
  byte stagingEnabled : 1;
  byte stagingMode : 1;
  byte EMAPPin : 4;

  byte rotarySplitValues[8];
  byte rotarySplitBins[8];

  uint16_t boostSens;
  byte boostIntv;
  uint16_t stagedInjSizePri;
  uint16_t stagedInjSizeSec;
  byte lnchCtrlTPS;

  uint8_t flexBoostBins[6];
  int16_t flexBoostAdj[6];  //kPa to be added to the boost target @ current ethanol (negative values allowed)
  uint8_t flexFuelBins[6];
  uint8_t flexFuelAdj[6];   //Fuel % @ current ethanol (typically 100% @ 0%, 163% @ 100%)
  uint8_t flexAdvBins[6];
  uint8_t  flexAdvAdj[6];    //Additional advance (in degrees) @ current ethanol (typically 0 @ 0%, 10-20 @ 100%). NOTE: THIS IS A SIGNED VALUE!
                            //And another three corn rows die.

  byte n2o_enable : 2;
  byte n2o_arming_pin : 6;
  byte n2o_minCLT;
  byte n2o_maxMAP;
  byte n2o_minTPS;
  byte n2o_maxAFR;

  byte n2o_stage1_pin : 6;
  byte n2o_pin_polarity : 1;
  byte n2o_stage1_unused : 1;
  byte n2o_stage1_minRPM;
  byte n2o_stage1_maxRPM;
  byte n2o_stage1_adderMin;
  byte n2o_stage1_adderMax;
  byte n2o_stage1_retard;

  byte n2o_stage2_pin : 6;
  byte n2o_stage2_unused : 2;
  byte n2o_stage2_minRPM;
  byte n2o_stage2_maxRPM;
  byte n2o_stage2_adderMin;
  byte n2o_stage2_adderMax;
  byte n2o_stage2_retard;

  byte knock_mode : 2;
  byte knock_pin : 6;

  byte knock_trigger : 1;
  byte knock_pullup : 1;
  byte knock_limiterDisable : 1;
  byte knock_unused : 2;
  byte knock_count : 3;

  byte knock_threshold;
  byte knock_maxMAP;
  byte knock_maxRPM;
  byte knock_window_rpms[6];
  byte knock_window_angle[6];
  byte knock_window_dur[6];

  byte knock_maxRetard;
  byte knock_firstStep;
  byte knock_stepSize;
  byte knock_stepTime;
        
  byte knock_duration; //Time after knock retard starts that it should start recovering
  byte knock_recoveryStepTime;
  byte knock_recoveryStep;

  byte unused11_122_191[69];

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

byte pinInjector1; //Output pin injector 1
byte pinInjector2; //Output pin injector 2
byte pinInjector3; //Output pin injector 3 is on
byte pinInjector4; //Output pin injector 4 is on
byte pinInjector5; //Output pin injector 5 NOT USED YET
byte pinInjector6; //Placeholder only - NOT USED
byte pinInjector7; //Placeholder only - NOT USED
byte pinInjector8; //Placeholder only - NOT USED
byte pinCoil1; //Pin for coil 1
byte pinCoil2; //Pin for coil 2
byte pinCoil3; //Pin for coil 3
byte pinCoil4; //Pin for coil 4
byte pinCoil5; //Pin for coil 5
byte pinCoil6; //Pin for coil 6
byte pinCoil7; //Pin for coil 7
byte pinCoil8; //Pin for coil 8
byte pinTrigger; //The CAS pin
byte pinTrigger2; //The Cam Sensor pin
byte pinTrigger3;	//the 2nd cam sensor pin
byte pinTPS;//TPS input pin
byte pinMAP; //MAP sensor pin
byte pinEMAP; //EMAP sensor pin
byte pinMAP2; //2nd MAP sensor (Currently unused)
byte pinIAT; //IAT sensor pin
byte pinCLT; //CLS sensor pin
byte pinO2; //O2 Sensor pin
byte pinO2_2; //second O2 pin
byte pinBat; //Battery voltage pin
byte pinDisplayReset; // OLED reset pin
byte pinTachOut; //Tacho output
byte pinFuelPump; //Fuel pump on/off
byte pinIdle1; //Single wire idle control
byte pinIdle2; //2 wire idle control (Not currently used)
byte pinIdleUp; //Input for triggering Idle Up
byte pinSpareTemp1; // Future use only
byte pinSpareTemp2; // Future use only
byte pinSpareOut1; //Generic output
byte pinSpareOut2; //Generic output
byte pinSpareOut3; //Generic output
byte pinSpareOut4; //Generic output
byte pinSpareOut5; //Generic output
byte pinSpareOut6; //Generic output
byte pinSpareHOut1; //spare high current output
byte pinSpareHOut2; // spare high current output
byte pinSpareLOut1; // spare low current output
byte pinSpareLOut2; // spare low current output
byte pinSpareLOut3;
byte pinSpareLOut4;
byte pinSpareLOut5;
byte pinBoost;
byte pinVVT_1;		// vvt output 1
byte pinVVt_2;		// vvt output 2
byte pinFan;       // Cooling fan output
byte pinStepperDir; //Direction pin for the stepper motor driver
byte pinStepperStep; //Step pin for the stepper motor driver
byte pinStepperEnable; //Turning the DRV8825 driver on/off
byte pinLaunch;
byte pinIgnBypass; //The pin used for an ignition bypass (Optional)
byte pinFlex; //Pin with the flex sensor attached
byte pinBaro; //Pin that an external barometric pressure sensor is attached to (If used)
byte pinEGT;          //[PJSC v1.03] For Exhaust Gas Temperature Sensor input
byte pinResetControl; // Output pin used control resetting the Arduino
byte pinExtTrigger;   //[PJSC] External Trigger input pin
byte pinExValve;      //[PJSC] Exhaust valve position input pin
byte pinCaptureDuty1; //[PJSC] For capturing duty pulse
byte pinCaptureDuty2; //[PJSC] For capturing duty pulse
byte pinMuxout1;      //[PJSC v1.01] For MUX output setting
byte pinMuxout2;      //[PJSC v1.01] For MUX output setting
byte pinMuxout3;      //[PJSC v1.01] For MUX output setting
byte pinMuxout4;      //[PJSC v1.01] For MUX output setting
byte pinAnalogInput1; //[PJSC v1.02] For Analog input selection
byte pinAnalogInput2; //[PJSC v1.03] For Analog input2 selection

// global variables // from speeduino.ino
extern struct statuses currentStatus; // from speeduino.ino
extern struct table3D fuelTable; //16x16 fuel map
extern struct table3D fuelTable2; //16x16 fuel map2 [PJSC]
extern struct table3D fuelTable3; //16x16 fuel map2 [PJSC v1.01]
extern struct table3D ignitionTable; //16x16 ignition map
extern struct table3D afrTable; //16x16 afr target map
extern struct table3D stagingTable; //8x8 afr target map
extern struct table2D taeTable; //4 bin TPS Acceleration Enrichment map (2D)
extern struct table2D WUETable; //10 bin Warm Up Enrichment map (2D)
extern struct table2D crankingEnrichTable; //4 bin cranking Enrichment map (2D)
extern struct config2 configPage2;
extern struct config4 configPage4;
extern struct config6 configPage6;
extern struct config9 configPage9;
extern struct config10 configPage10;
extern unsigned long currentLoopTime; //The time the current loop started (uS)
extern unsigned long previousLoopTime; //The time the previous loop started (uS)
volatile uint16_t ignitionCount; //The count of ignition events that have taken place since the engine started
extern byte cltCalibrationTable[CALIBRATION_TABLE_SIZE];
extern byte iatCalibrationTable[CALIBRATION_TABLE_SIZE];
extern byte o2CalibrationTable[CALIBRATION_TABLE_SIZE];

#endif // GLOBALS_H
