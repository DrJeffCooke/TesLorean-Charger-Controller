#ifndef __CONFIG_H__
#define __CONFIG_H__

// PIN Mapping for Microcontroller
#define CHARGER1_ENABLE     3
#define CHARGER2_ENABLE     4
#define CHARGER3_ENABLE     5
#define CHARGER1_ACTIVATE   62
#define CHARGER2_ACTIVATE   63
#define CHARGER3_ACTIVATE   64
#define DIG_OUT_1            48
#define DIG_OUT_2            49
#define DIG_OUT_3            50
#define DIG_OUT_4            51
#define DIG_IN_1            6
#define DIG_IN_2            7
#define EVSE_ACTIVATE       44
#define EVSE_PILOT          2
#define EVSE_PROX           A0

// Change this value if you want to load a new set of parameters into EEPROM
// If left the same, the parameter values from EEPROM are used
#define EEPROM_VERSION      13

// Data structure to hold the EEPROM values
typedef struct
{
    uint8_t  version; //eeprom version stored
    uint8_t  enabledChargers;
    uint8_t  mainsRelay; //which output is used to control the AC relay
    uint8_t  autoEnableCharger;
    uint8_t  canControl;
    uint8_t  type;
    uint8_t  phaseconfig;
    uint16_t voltSet;
    uint16_t currReq;
    uint32_t can0Speed;
    uint32_t can1Speed;
    uint16_t dcdcsetpoint;
}   ChargerParams;

#endif
