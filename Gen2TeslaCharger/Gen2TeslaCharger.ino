/*
  Tesla Gen 2 Charger Control Program
  2017-2018
  T de Bree
  D.Maguire
  Additional work by C. Kidder
  Runs on OpenSource Logic board V2 in Gen2 charger. Commands all modules.

  June 2018 - Added customizations for the TesLorean controller environment DrJeffCooke
*/

#include <can_common.h>
#include <due_can.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include <DueTimer.h>
#include "config.h"

#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) {
  obj.print(arg);
  return obj;
}

//********* DEBUG FLAGS ********************
int debugevse = 1;  // 1 = show Proximity status and Pilot current limmits
int debug = 1;      // 1 = show phase module CAN feedback
int candebug = 1;   // show CAN updates and timeouts
int debugmsgfreq = 500; // number of ms between general debug outputs

//*********GENERAL VARIABLE   DATA ******************
uint16_t curset = 0;  // ?BUG? - variable not found in the code
int  setting = 1;     // Flag to indicate if a entry made to change a setting (setting changed = 1, not changed = 0)
int incomingByte = 0; // Temporary variable for last byte received via serial port
int state;            // Current charger state (0 = turn chargers off, 1 = turn chargers on, 2 = enable chargers, 3 = waiting for chargers to enable)
int reqdstate;        // Request to change to this state
bool bChargerEnabled; // Flag to indicate if chargers are enabled or not (state = 1)(false = not enabled, true = enabled)
// tcan => time at last incoming CAN message for Elcon or slave charging
// tboot => time at instruction to startup the modules
// tlast => time at last status message
// slavetimeout => time at last update of the slave message
unsigned long slavetimeout, tlast, tcan, tboot = 0;

//*********EVSE VARIABLE   DATA ******************
uint16_t ACvoltIN = 240; // AC input voltage 240VAC for EU/UK and 110VAC for US
// Proximity EVSE variables
byte Proximity = 0;   // Proximity line enumeration (see below) : voltage set by EVSE, but depends on Type 1 or Type 2
//proximity status values for type 1
#define Unconnected 0 // 3.3V
#define Buttonpress 1 // 2.3V
#define Connected 2 // 1.35V
uint16_t cablelim = 0;    // Power connection (Type 2 only) cable limit as indicated by Proximity
volatile uint32_t pilotgap = 1000000; // microseconds (1000 micros = 1 millisec) If Pilot response gap is longer than 1.0 seconds
// Pilot EVSE variables
volatile uint32_t pilottimer = 0;         // Used to calculate the duty cycle on the Pilot line, duty cycle indicates AC current limit
volatile uint16_t timehigh, duration = 0; // 'timehigh' is an unused variable, 'duration' is used in Pilot duty cycle calculation
volatile uint16_t accurlim = 0;           // AC current limit calculated from the Pilot line
volatile int dutycycle = 0;               // Calculated duty cycle of the Pilot line
bool bPilotTimeout;                       // Flag set if the pilot squarewave times out
uint16_t enablechargerswait = 2000;       // How many ms to wait from time of enabling chargers to activating

//*********Single or Three Phase Config VARIABLE   DATA ******************
// Power Source configuration
#define Singlephase 0 // all parallel on one phase Type 1
#define Threephase 1  // one module per phase Type 2

//*********Charger Control VARIABLE   DATA ******************
bool Vlimmode = true;                 // ?UNUSED? - Set charges to voltage limit mode
// modulelimcur is the mA (scaled * 1.5) requested for charging for each single module
// dcaclim is the max mA limit (per module) that the ACcurrent should be at to limit the DCcurrent to the max
uint16_t modulelimcur, dcaclim = 0;
// set maximum AC current in mA (only used to initially set 'dcaclim' in setup) Modules max mA is 16,000
uint16_t maxaccur = 16000;            
// Used to set the maxACcurrent = depends on DCvoltage, ACvoltage and maxACcurrent per module
// maxdccur = (16000 (maxDCcurrentpermodule) * 3 (three modules) * ACvoltage / DCvoltage) - 400
// Suspect the '400' is a safety factor as the DCvolts vary (ACvolts should be consistent)
uint16_t maxdccur = 28000;            
// activemodules = count of modules that are enabled and active
// slavechargerenable = 0 slave charger not recruited, 1 slave charger recruited (if charge request is > 15A), says nothing about slave being present
int activemodules, slavechargerenable = 0;
// For validation checks
uint16_t maxhiaccur = maxaccur;            // maximum AC current in mA (only iused to initially set 'dcaclim' in setup)
uint16_t maxhidccur = maxdccur;            // maximum limit DC current output in mA
uint16_t maxhivolts = 40000;               // set to 405v; maximum limit Voltage in 0.01V (hundredths of a volt)

//*********Feedback from charge VARIABLE   DATA ******************
uint16_t dcvolt[3] = {0, 0, 0};       //DC Voltage, 1 = 1V
uint16_t dccur[3] = {0, 0, 0};        //DC Current
uint16_t totdccur = 0;                //Total DC Current, 1 = 0.005Amp
uint16_t acvolt[3] = {0, 0, 0};       //AC Voltage, 1 = 1V
uint16_t accur[3] = {0, 0, 0};        //AC Current, 1 = 0.06666 Amp
long acpower = 0;                     // ?UNUSED?
byte inlettarg [3] = {0, 0, 0};       //inlet target temperatures, should be used to command cooling.
byte curtemplim [3] = {0, 0, 0};      //current limit due to temperature
byte templeg[2][3] = {{0, 0, 0}, {0, 0, 0}}; //temperatures reported back
bool ACpres [3] = {0, 0, 0};          //AC present detection on the modules {true, false}
bool ModEn [3] = {0, 0, 0};           //Module enable feedback on the modules {true, false}
bool ModFlt [3] = {0, 0, 0};          //module fault feedback {true, false}
byte ModStat [3] = {0, 0, 0};         //Module Status {binary}
int newframe = 0;                     // bit0=1 if temp reported/detail message, bit1=1 if  DC feedback reported, value is never used

// Record variable used to hold the EEPROM variable values
ChargerParams parameters;

//*********DCDC Messages VARIABLE   DATA ******************
bool dcdcenable = 1; // 1=ON, 0=OFF, CAN messages for the DCDC.(Tesla DCDC specific), voltage specified by parameters.dcdcsetpoint

//*********Charger Messages VARIABLE   DATA ******************
int ControlID = 0x300;      // Internal charger frame ID for control
int StatusID = 0x410;       // Internal charger frame ID for status

//********** ELCON specific - not used in TesLorean ************
unsigned long ElconID = 0x18FF50E5;
unsigned long ElconControlID = 0x1806E5F4;

//********** DCDC CAN ***************
int DCDCConverterControlID = 0x3D8;       // Frame ID for CAN instructions to the Tesla DCDC converter

//********* Charger Outgoing Status and Control CAN **************
int TesLoreanChargerID = 0x2B0;     // Internal charger frame ID for controller comms with TesLorean
int FastCycleCountLimit = 9;        // How many 100ms cycles
int TesLoreanFastCycleCount = 0;    // Counts in 100ms gaps, in a 0-9 cycle, so rolls over every 1 sec
int SlowCycleCountLimit = 99;        // How many 100ms cycles
int TesLoreanSlowCycleCount = 0;    // Counts in 100ms gaps, in a 0-99 cycle, so rolls over every 10 secs
int VSlowCycleCountLimit = 299;        // How many 100ms cycles
int TesLoreanVSlowCycleCount = 0;    // Counts in 100ms gaps, in a 0-2999 cycle, so rolls over every 30 secs

// SETUP TASKS
// 1. Initialize the USB port (takes commands and outputs debug and status messages)
// 2. Set up the ChargerMessages to output ever 100ms
// 2. Set up the Pilot interrupt (pilot changes signal EVSE voltage allowance via Duty Cyle calcs)
// 3. Read parameters from the EEPROM, if a new version is provided update the EEPROM from values in code
// 4. Initialize CAN0 (internal to charger) and CAN1 (external to charger) ports
// 5. Set up the microcontroller's pin mapping to the charge modules and to the external data ports
// 6. Initialize the charger status flag
void setup()
{
  // Start the USB port communciations
  Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

  // Output a Charger 'keep alive' message 10 times a second
  Timer3.attachInterrupt(Charger_msgs).start(90000); // charger messages every 100ms (probably 90ms to allow for run time)

  // Run the Pilotread() function if the PILOT line value changes
  attachInterrupt(EVSE_PILOT, Pilotread , CHANGE);

  Wire.begin();

  // Read the EEPROM and populate the 'parameters' record
  EEPROM.read(0, parameters);
  if (parameters.version != EEPROM_VERSION)   // Check that EEPROM data is a different version from the data in Config.H
  {
    parameters.version = EEPROM_VERSION;  // Increment the EEPROM_VERSION constant in the Config.H file to force a reloading of the EEPROM stored values
    parameters.can0Speed = 500000;        // CAN0 connects to the internal charger modules
    parameters.can1Speed = 500000;        // CAN1 is exposed externally at the port
    parameters.enabledChargers = 123;     // enable per phase - 123 is all phases - 3 is just phase 3
    parameters.mainsRelay = 48;           // ?BUG? variable is not referenced in the code. It may refer to the line on the microcontroller that outputs to the main s relay
    
    // TesLorean Configuration
    parameters.currReq = 15000;            // max current input limit per module, note: 1500 = 1A, 7500 = 5A, 15000 = 10A
    parameters.voltSet = maxhivolts;      // 1 = 0.01V
    parameters.phaseconfig = Singlephase; //AC input configuration (US=Singlephase, EU=Threephase)
    parameters.type = 1;                  // Socket type1 or 2. Note Type 1 is J1772 USA
    parameters.autoEnableCharger = 0;     // 1 = enabled, 0 = disabled auto start, with proximity and pilot control
    parameters.canControl = 0;            // 0 = disabled can control, or in the following modes 1 = master, 2 = Elcon master, 3 = slave
                                          // Master mode also accommodates a Slave charger (i.e. sends Slave CAN instructions every 100ms)
    parameters.dcdcsetpoint = 14000;      // voltage setpoint for DCDC Converter in mv (sent via external CAN to DCDC)

    // Write out the replacement values
    EEPROM.write(0, parameters);          // Write the values out to the EEPROM
  }

  // ///////////// Initialize CAN ports ////////////////////////////
  if (Can1.begin(parameters.can1Speed, 255)) //can1 external bus
  {
    Serial.println("Using CAN1 - initialization completed.\n");
  }
  else Serial.println("CAN1 initialization (sync) ERROR\n");

  // Initialize CAN0
  if (Can0.begin(parameters.can0Speed, 255)) //can0 charger modules
  {
    Serial.println("Using CAN0 - initialization completed.\n");
  }
  else Serial.println("CAN0 initialization (sync) ERROR\n");

  // Clear (by setting blank) all the filters on the CAN data, i.e. no frames get summarily rejected
  int filter;
  //extended
  for (filter = 0; filter < 3; filter++)
  {
    Can0.setRXFilter(filter, 0, 0, true);
    Can1.setRXFilter(filter, 0, 0, true);
  }
  //standard
  for (int filter = 3; filter < 7; filter++)
  {
    Can0.setRXFilter(filter, 0, 0, false);
    Can1.setRXFilter(filter, 0, 0, false);
  }

  ///////////////////CHARGER ENABLE AND ACTIVATE LINES///////////////////////////////////
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CHARGER1_ENABLE, OUTPUT); //CHG1 ENABLE
  pinMode(CHARGER2_ENABLE, OUTPUT);  //CHG2 ENABLE
  pinMode(CHARGER3_ENABLE, OUTPUT); //CHG3 ENABLE
  pinMode(CHARGER1_ACTIVATE, OUTPUT); //CHG1 ACTIVATE
  pinMode(CHARGER2_ACTIVATE, OUTPUT);  //CHG2 ACTIVATE
  pinMode(CHARGER3_ACTIVATE, OUTPUT); //CHG3 ACTIVATE

  //////////////////////////////////////////////////////////////////////////////////////
  pinMode(DIG_IN_1, INPUT); //IP1
  pinMode(DIG_IN_2, INPUT); //IP2

  //////////////DIGITAL OUTPUTS MAPPED TO X046. 10 PIN CONNECTOR ON LEFT////////////////
  pinMode(DIG_OUT_1, OUTPUT); //OP1 - X046 PIN 6
  pinMode(DIG_OUT_2, OUTPUT); //OP2
  pinMode(DIG_OUT_3, OUTPUT); //OP2
  pinMode(DIG_OUT_4, OUTPUT); //OP3
  pinMode(EVSE_ACTIVATE, OUTPUT); //pull Pilot to 6V
  ///////////////////////////////////////////////////////////////////////////////////////

  // Set the maximum AC current
  dcaclim = maxaccur;

  bChargerEnabled = false;  // ?FIXED? are we supposed to command the charger to charge?
  bPilotTimeout = false;    // Flag to indicate if pilot error detected during interrupt service 
}

// MAIN LOOP TASKS
// 1. Check for internal CAN data frame and process
// 2. Check for external CAN data frame and process
// 3. Check for Serial data, adjust data and flag that a change occurred
// 4. If setting was changed (most likely via serial port) output a detailed reflection
// 5. Check if settings changed, the use STATUS as indicator of the mode the charge modules should be in
// ... status 0 => shutdown the charging, status 2 => enable chargers, status 3 => wait defined time, status 1 => activate chargers
// 6. Output debug and status information (on a regular time interval)
// 7. Check the DC Current is within limits, if not modify
// 8. Check the AC Current is within limits, if not modify
// 9. Check the Proximity setting and update the connection status as necessary
// 10. Handle AutoCharging (if cable plugged in, the start sequence to start up chargers)
void loop()
{
  CAN_FRAME incoming;

  // Check the internal CAN bus for data from the charger modules
  if (Can0.available())   // ?PENDING? - May need to be moved to an interrupt to match data flows
  {
    Can0.read(incoming);
    candecode(incoming);
  }

  // Check the external CAN bus for data from other controllers
  if (Can1.available())   // ?PENDING? - May need to be moved to an interrupt to match data flows
  {
    Can1.read(incoming);
    canextdecode(incoming);
  }

  // Check for data incoming on the controllers USB port (should occur very infrequently)
  if (Serial.available())
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 'a':   //a for auto enable
        if (Serial.available() > 0)
        {
          parameters.autoEnableCharger = Serial.parseInt();   // returns 0 if no int found
          if (parameters.autoEnableCharger > 1)
          {
            parameters.autoEnableCharger = 0;
            Serial.println("[SERIAL] Error : Autostart selections are 'a' followed by 0 or 1.");
          }
          setting = 1;
          // Confirmation message
          if (parameters.autoEnableCharger == 0){Serial.println("[SERIAL] Autostart disabled.");}
          else {Serial.println("[SERIAL] Autostart enabled.");}
        }
        break;

      case 'p':   //p for phases setting
        if (Serial.available() > 0)
        {
          //parameters.phaseconfig = Serial.parseInt() - 1;   // ?BUG? Why subtract 1?
          parameters.phaseconfig = Serial.parseInt();   // returns 0 if no int found
          // valid entries are 3 and 1
          if ( parameters.phaseconfig == 3)
          {
            parameters.phaseconfig = Threephase;
            setting = 1;
          }
          if (parameters.phaseconfig == 1)
          {
            parameters.phaseconfig = Singlephase;
            setting = 1;
          }
          // If the parameter is not 1 or 3, just leave it unchanged
          if (setting == 0)
          {
            Serial.println("[SERIAL] Error : Phase entry must be p1 or p3");
          }
          else
          {
            // Confirmation message
            if ( parameters.phaseconfig == 3){Serial.println("Info : Phase set to 'Three Phase'");}
            else {Serial.println("[SERIAL] Phase set to 'Single Phase'");}
          }
        }
        break;

      case 't':   //t for type
        if (Serial.available() > 0)
        {
          parameters.type = Serial.parseInt();    // returns 0 if no int found
          // Values other than 1 are set to 2, only 1 and 2 are valid values
          if (parameters.type == 2 )
          {
            setting = 1;
          }
          if (parameters.type == 1)
          {
            setting = 1;
          }
          // If the parameter is not 1 or 2, just leave it unchanged
          if (setting == 0)
          {
            Serial.println("[SERIAL] Error : Type entry must be t1 or t2");
          }
          else
          {
            // Confirmation message
            if (parameters.type == 2 ){Serial.println("Info : Set as Type 2.");}
            else {Serial.println("Error : Set as Type 1.");}
          }
        }
        break;

      case 'x':   //x for can control enable
        if (Serial.available() > 0)
        {
          parameters.canControl = Serial.parseInt();    // returns 0 if no int found
          if (parameters.canControl > 3)
          {
            Serial.println("[SERIAL] Error : CAN control selections are 'x' followed by 0,1,2, or 3.");
            parameters.canControl = 0;
          }
          setting = 1;
          // Confirmation message
          if (parameters.canControl == 0){Serial.println("Info : CAN control disabled.");}
          if (parameters.canControl == 1){Serial.println("Info : CAN control in Master mode.");}
          if (parameters.canControl == 2){Serial.println("Info : CAN control in Elcon Master mode.");}
          if (parameters.canControl == 3){Serial.println("Info : CAN control in Slave mode.");}
        }
        break;

      case 'm':   //m for dc current setting in whole numbers
        if (Serial.available() > 0)
        {
          uint16_t tempmaxdccur;
          uint16_t tempdccur = Serial.parseInt();       // returns 0 if no int found
          if (tempdccur != 0)
          {
            // Test that DC current request doesn't exceed system maximum
            tempmaxdccur = (tempdccur * 1000);
            if (tempmaxdccur <= maxhidccur)
            {
              // ?WHY? - Doesn't set a EEPROM value, just sets the maximum that the current run uses (initialed as 45000)
              maxdccur = (tempdccur * 1000);
              setting = 1;
              // Confirmation message
              Serial.println("[SERIAL] New setting for Maximum DC Current.");
            }
            else
            {
              Serial.println("[SERIAL] Error : New setting for Maximum DC Current exceeds allowable maximum. Max DC current unchanged.");
            }
          }
          else
          {
            Serial.println("[SERIAL] Error : Max DC Current is 'm' followed by a whole number, e.g. 'm45'. Max DC current unchanged.");
          }
        }
        break;

      case 'v':   //v for voltage setting in whole numbers
        if (Serial.available() > 0)
        {
          uint16_t tempmaxvolt;
          uint16_t tempvolt = Serial.parseInt();
          if (tempvolt != 0)
          {
            // Test that Voltage request doesn't exceed system maximum
            tempmaxvolt = (tempvolt * 100);
            if (tempmaxvolt <= maxhivolts)
            {
              parameters.voltSet = (tempvolt * 100);
              setting = 1;
              Serial.println("[SERIAL] New setting for Voltage.");
            }
            else
            {
              Serial.println("[SERIAL] Error : New setting for Voltage exceeds allowable maximum. Max voltage unchanged.");
            }
          }
          else
          {
            Serial.println("[SERIAL] Error : Set Voltage is 'v' followed by a whole number, e.g. 'v400'. Max voltage unchanged.");
          }
        }
        break;

      case 's':   //s for start AND stop
        // If currently stopped - signal the start process (two stage)
        if (state == 0)
        {
          reqdstate = 2;          // Start the process of enabling and activating the modules
          tboot = millis();   // Set the time at which the 'start' 'stop' command was issued // Now redundant, but harmless
          setting = 1;
          Serial.println("[SERIAL] Initiating Module startup.");
        }
        // if currently started (states 1,2,3) - signal to stop
        if (state >= 1)
        {
          reqdstate = 0;          // shutdown modules
          setting = 1;
          Serial.println("[SERIAL] Initiating Module shutdown.");
        }
        break;

      case 'e':   //e for enabling chargers followed by 1 to 3 digits (comprised of 1,2,3) to indicate which ones to run
        if (Serial.available() > 0)
        {
          // Test for a valid digits string (as integer) and then update setting, otherwise error
          uint8_t  tempCs;
          tempCs = Serial.parseInt();
          if (tempCs == 1 || tempCs == 2 || tempCs == 3 || tempCs == 12 || tempCs == 13 || tempCs == 23 || tempCs == 123)
          {
            parameters.enabledChargers = tempCs;
            setting = 1;
            // Confirmation message
            Serial.println("[SERIAL] Instruction accepted for enabled chargers.");
          }
          else
          {
            Serial.println("[SERIAL] Error : To enable chargers enter 'e' followed by 0,1,2,12,13,23, or 123. No changes made to enabled chargers.");
          }
        }
        break;

      case 'c': //c for current setting in whole numbers
        if (Serial.available() > 0)
        {
          uint16_t tempmaxcur;
          uint16_t tempcurreq = Serial.parseInt();
          // ?PENDING? - CurrReq is set to 0 in Parameters by default, so 0 must be a valid value ???
          //if (tempcurreq != 0)
          //{
            // Test that Voltage request doesn't exceed system maximum
            tempmaxcur = (tempcurreq * 1500);
            if (tempmaxcur <= maxhiaccur)         // ?PENDING? - is this the current value to be testing against ???
            {
              parameters.currReq = tempmaxcur;
              setting = 1;
              // Confirmation message
              Serial.println("[SERIAL] New setting for Current.");
            }
            else
            {
              Serial.println("[SERIAL] Error : New setting for Current exceeds allowable maximum. Max current unchanged.");
            }
          //}
        }
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  // Check the flag indicating that a parameter was changed and needs processing
  if (setting == 1)
  {
    // A parameter has changed so update the EEPROM store with the new values
    EEPROM.write(0, parameters);

    // clear the setting flag
    setting == 0;

    // Two blank lines
    Serial.println();
    Serial.println("NEW PARAMETERS...");
    Serial.print("Charger = ");
    if (state == 0){Serial.print("Off");}
    if (state == 1){Serial.print("On");}
    if (state == 2){Serial.print("Enabling");}
    if (state == 3){Serial.print("Activating");}
    Serial.print(" | Enabled Modules = ");
    Serial.print(parameters.enabledChargers);
    //Serial.print(" | Phases = ");
    //Serial.print(parameters.phaseconfig);
    Serial.print(" | Set voltage = ");
    Serial.print(parameters.voltSet * 0.01f, 0);
    Serial.print("V | Set current lim AC = ");
    Serial.print(parameters.currReq * 0.00066666, 0);
    Serial.print("A | DC = ");
    Serial.print(maxdccur * 0.001, 1);
    Serial.print("A | ");
    if (parameters.autoEnableCharger == 1){Serial.print("Autostart On ");}
    else{Serial.print("Autostart Off");}
    Serial.print(" | ");
    if (parameters.canControl == 0){Serial.print("Can Mode = Off          ");}
    if (parameters.canControl == 1){Serial.print("Can Mode = Master       ");}
    if (parameters.canControl == 2){Serial.print("Can Mode = Master Elcon ");}
    if (parameters.canControl == 3){Serial.print("Can Mode = Slave        ");}
    Serial.print(" | ");
    if (parameters.phaseconfig == Singlephase){Serial.print("Single Phase");}
    if (parameters.phaseconfig == Threephase){Serial.print("Three Phase ");}
    Serial.print(" | ");
    if (parameters.type == 1){Serial.print("Type 1");}
    if (parameters.type == 2){Serial.print("Type 2");}
    setting = 0;
    Serial.println();
    Serial.println();
  }

  // When under CanControl and for just ElCon/Slave modes, check that a minimum amount of time (500ms) hasn't passed since 'tcan' was set?
  // If more than 500ms, set the flag to shutdown charging - the CAN has timed out
  if (parameters.canControl > 1)
  {
    // Chargers are enabled (1) or enabling (2 or 3)
    if (state != 0)
    {
      // 500ms since last tcan setting
      if (millis() - tcan > 500)
      {
        // Shutdown the charger
        reqdstate = 0;
        Serial.println();
        Serial.println("WARNING : CAN (incoming) time-out for slave.");
      }
    }
  }

  // Test that the Enable line is 12v (CHARGER Switch is ON)
  // ?PENDING? - if scheduled charging, the enable line need not be high
  // if Enable line OFF - set status to immediately shutdown chargers
  if (digitalRead(DIG_IN_1) == LOW)   
  {
    // Don't allow chargers to enable, always instruct the chargers to disable
    reqdstate = 0;
  }

  // Check the state of the chargers vs their enabled status and take action
  // state 0 : Chargers are shutdown
  // state 1 : Chargers are enabled and activated
  // state 2 : Chargers are enabling
  // state 3 : Chargers waiting 500ms after being enabled
  // Sequence 0 off -> 2 enabling -> 3 enabled -> 1 activated -> 0 shutdown
  // bChargerEnabled : chargers are enabled (may not be active)
  switch (reqdstate)
  {
    // Request to move Charger to an OFF state
    case 0: 

      // Only if debug is set and new state
      if (debug != 0 && reqdstate != state)
      {
        Serial.println();
        Serial.println("[STATE CHANGE] State = 0 ");
      }

      // ?PRIOR? - Send the deactivate, disable commands on every loop.  May be required behavior.

      // Check that the chargers are currently enabled
      //if (state != 0)  // If already shutdown, ignore request
      //{
        // For emergency shutdown reasons - If shutdown is requested just action immediately
      //}
      digitalWrite(DIG_OUT_1, LOW);//MAINS OFF
      digitalWrite(EVSE_ACTIVATE, LOW);   // Tell the EVSE to stop providing power
      digitalWrite(CHARGER1_ACTIVATE, LOW); //chargeph1 deactivate
      digitalWrite(CHARGER2_ACTIVATE, LOW); //chargeph2 deactivate
      digitalWrite(CHARGER3_ACTIVATE, LOW); //chargeph3 deactivate
      digitalWrite(CHARGER1_ENABLE, LOW);//disable phase 1 power module
      digitalWrite(CHARGER2_ENABLE, LOW);//disable phase 2 power module
      digitalWrite(CHARGER3_ENABLE, LOW);//disable phase 3 power module
          
      // Flag that the chargers are now OFF
      state = 0;
      bChargerEnabled = false;

      // Turn the LED indicator light off
      digitalWrite(LED_BUILTIN, LOW);

      break;

    // Request to change Charger to an ON state
    case 1:

      // Check that the Chargers are currently listed as having Enabled
      if (state == 3 && bChargerEnabled == true)
      {      
        // Activate the correct chargers
        switch (parameters.enabledChargers)
        {
          // Test the most likely case first, all others are unlikely options
          case 123:
            digitalWrite(CHARGER1_ACTIVATE, HIGH);
            digitalWrite(CHARGER2_ACTIVATE, HIGH);
            digitalWrite(CHARGER3_ACTIVATE, HIGH);
            activemodules = 3;
            break;
          case 1:
            digitalWrite(CHARGER1_ACTIVATE, HIGH);
            activemodules = 1;
            break;
          case 2:
            digitalWrite(CHARGER2_ACTIVATE, HIGH);
            activemodules = 1;
            break;
          case 3:
            digitalWrite(CHARGER3_ACTIVATE, HIGH);
            activemodules = 1;
            break;
          case 12:
            digitalWrite(CHARGER1_ACTIVATE, HIGH);
            digitalWrite(CHARGER2_ACTIVATE, HIGH);
            activemodules = 2;
            break;
          case 13:
            digitalWrite(CHARGER1_ACTIVATE, HIGH);
            digitalWrite(CHARGER3_ACTIVATE, HIGH);
            activemodules = 2;
            break;
          case 23:
            digitalWrite(CHARGER2_ACTIVATE, HIGH);
            digitalWrite(CHARGER3_ACTIVATE, HIGH);
            activemodules = 2;
            break;
          default:
            // if nothing else matches, do the default
            // default is optional
            break;
        }

        // Only if debug is set and new state
        if (debug != 0 && reqdstate != state)
        {
          Serial.println();
          Serial.println("[STATE CHANGE] State = 1 ");
        }
        
        // Flag that the chargers are active
        state = 1;

        // Delay 100ms, then signal to switch the mains on (via relay) and activate the EVSE
        delay(100);
        digitalWrite(DIG_OUT_1, HIGH);        // MAINS ON (via relay)
        digitalWrite(EVSE_ACTIVATE, HIGH);    // Signal to EVSE to provide power

        // Turn the LED indicator light 
        digitalWrite(LED_BUILTIN, HIGH);
      }
      break;

    case 2:

      // Start the enable process for the selected chargers
      if (state == 0)
      {
        switch (parameters.enabledChargers)
        {
          // Check the most likely case first
          case 123:
            digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
            digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
            digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
            break;
          case 1:
            digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
            break;
          case 2:
            digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
            break;
          case 3:
            digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
            break;
          case 12:
            digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
            digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
            break;
          case 13:
            digitalWrite(CHARGER1_ENABLE, HIGH);//enable phase 1 power module
            digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
            break;
  
          case 23:
            digitalWrite(CHARGER2_ENABLE, HIGH);//enable phase 2 power module
            digitalWrite(CHARGER3_ENABLE, HIGH);//enable phase 3 power module
            break;
  
          default:
            // if nothing else matches, do the default
            break;
        }
 
        // Only if debug is set and new state
        if (debug != 0 && reqdstate != state)
        {
          Serial.println();
          Serial.println("[STATE CHANGE] State = 2 ");
        }

        // Update the state
        state = 2;
        
        // Keep the chargers as NOT ENABLED until after the 500ms has expired (to give them time to enable)
        // Once enabled they will get CAN frames
        bChargerEnabled = false;
        
        // Request to go to state=3, which will last for 500ms
        reqdstate = 3;
  
        // DEBUG Should 'tboot' be set here? instead of in multiple other locations?
        tboot = millis();   // Set the time at which the chargers were instructed to enable

         // Only if debug is set
        if (debug != 0)
        {
          Serial.println();
          Serial.println("[STATE CHANGE] State = 2 ");
        }

      }
      break;    // Missing in prior code, but no ill-effects

    case 3:
      // Check that 500ms has passed since state = 2 (signal to start enabling) was set
      if (state == 2)
      {
        state = 3;

         // Only if debug is set
        if (debug != 0)
        {
          Serial.println();
          Serial.println("[STATE CHANGE] State = 3 ");
        }
      }
      // Check that a time has passed before leaving state 3 and attempting to activate the chargers
      if (tboot <  (millis() - enablechargerswait))
      {
        // After 500ms okay to start the ACTIVATE module process
        reqdstate = 1;
        bChargerEnabled = true;
      }
      break;

    default:
      // if nothing else matches, do the default
      break;
  }

  // Every 500ms output a debug status message
  if (tlast <  (millis() - debugmsgfreq))
  {
    // mark a new time
    tlast = millis();

    // Only if debug is set
    if (debug != 0)
    {
      Serial.println();
      Serial.print("[DEBUG] Time = ");
      Serial.print(millis());
      Serial.print(" | State = ");
      Serial.print(state);
      // Serial.print(" | Phases = ");
      // Serial.print(parameters.phaseconfig);
      if (bChargerEnabled) {Serial.print(" | Chargers ON");}
      else {Serial.print(" | Chargers OFF");}
      if (digitalRead(DIG_IN_1) == HIGH) {Serial.print(" | Enable Hi");}
      else {Serial.print(" | Enable Lo");}
      Serial.print(" | AC limit = ");
      Serial.print(accurlim);
      Serial.print(" | Cable Limit = ");
      Serial.print(cablelim);
      Serial.print(" | Mod Cur Request = ");
      Serial.print(modulelimcur / 1.5, 0);
      Serial.print(" | DC AC Cur Lim = ");
      Serial.print(dcaclim);
      Serial.print(" | Active =  ");
      Serial.print(activemodules);
      Serial.print(" | DC total Cur = ");
      Serial.print(totdccur * 0.005, 2);
      Serial.print(" | DC Setpoint = ");
      Serial.print(parameters.voltSet * 0.01, 0);
      if (bPilotTimeout) {Serial.print(" | Pilot Timeout");}
      else {Serial.print(" | Pilot OK");}
      Serial.println();

      if (bChargerEnabled)
      {
        for (int x = 0; x < 3; x++)
        {
          Serial.print("[CHARGER] Phase = ");
          Serial.print(x + 1);
          Serial.print(" | Feebback AC present = ");
          Serial.print(ACpres[x]);
          Serial.print(" | AC volt = ");
          Serial.print(acvolt[x]);
          Serial.print(" | AC cur = ");
          Serial.print((accur[x] * 0.06666), 2);
          Serial.print(" & ");
          Serial.print(accur[x]);
          Serial.print(" | DC volt = ");
          Serial.print(dcvolt[x]);
          Serial.print(" | DC cur = ");
          Serial.print(dccur[x] * 0.000839233, 2);
          Serial.print(" | Inlet Targ = ");
          Serial.print(inlettarg[x]);
          Serial.print(" | Temp Lim Cur = ");
          Serial.print(curtemplim[x]);
          Serial.print(" & ");
          Serial.print(templeg[0][x]);
          Serial.print(" & ");
          Serial.print(templeg[1][x]);
          Serial.print(" | EN = ");
          Serial.print(ModEn[x]);
          Serial.print(" | Flt = ");
          Serial.print(ModFlt[x]);
          Serial.print(" | Stat = ");
          Serial.print(ModStat[x], BIN);
          Serial.println();
        }
      }
      else
      {
        if (state == 0){Serial.print("[CHARGER] Modules Turned OFF");}
        if (state == 2){Serial.print("[CHARGER] Modules Enabling...");}
        if (state == 3){Serial.print("[CHARGER] Modules Activating...");}
        Serial.println();
      }
      if (debugevse != 0)
      {
        //Serial.println();
        Serial.print("[EVSE] Proximity Status = ");
        switch (Proximity)
        {
          case Unconnected:
            Serial.print("Unconnected");
            break;
          case Buttonpress:
            Serial.print("Button Pressed");
            break;
          case Connected:
            Serial.print("Connected");
            break;
        }
        Serial.println();
      }
    }
  }

  // Check the DC current levels (maintain within limits for the source phases/types)
  DCcurrentlimit();

  // Check the AC current levels (maintain within limits for the source phases/types)
  ACcurrentlimit();

  //EVSE automatic control - check the PROXIMITY line and set the cablelim accordingly
  evseread();

  // If in AutoEnableCharger mode, check if the power cable is plugged in, is so start the charging process
  if (parameters.autoEnableCharger == 1)
  {
    if (Proximity == Connected) //check if plugged in
    {
      //digitalWrite(EVSE_ACTIVATE, HIGH);//pull pilot low to indicate ready - NOT WORKING freezes PWM reading
      if (modulelimcur > 1400) // one amp or more active modules
      {
        if (state == 0)
        {
          if (digitalRead(DIG_IN_1) == HIGH)
          {
            // Start the process of initializing the modules
            reqdstate = 2;

            // Set a timer recording the start of the initialize request
            // ??Should this be done in the code that handles the state = 2 request?
            tboot = millis();

            if (debugevse != 0){Serial.println("[EVSE] Requesting start-up (state=2)");}
          }
        }
      }
      digitalWrite(DIG_OUT_2, HIGH); //enable AC present indication
    }
    else // unplugged or buton pressed stop charging
    {
      reqdstate = 0;
      digitalWrite(DIG_OUT_2, LOW); //disable AC present indication

      // ?? Isn't this taken care of in the 'state=0' code ??
      digitalWrite(EVSE_ACTIVATE, LOW);

      if (debugevse != 0  && state != reqdstate){Serial.println("[EVSE] Requesting shutdown (state=0)");}

    }
  }
}

// Decode the Charger's internal CANbus data transmissions
// Populates data structures for each charger based on received data
void candecode(CAN_FRAME & frame)
{
  int x = 0;
  switch (frame.id)
  {

    // Status Messages
    case 0x217: //phase 1 Status message
      ModStat[0] = frame.data.bytes[0];
      break;

    case 0x219: //phase 2 Status message
      ModStat[1] = frame.data.bytes[0];
      break;

    case 0x21B: //phase 3 Status message
      ModStat[2] = frame.data.bytes[0];
      break;

    // Temp Messages
    case 0x23B: //phase 3 temp message 1
      templeg[0][2] = frame.data.bytes[0] - 40;
      templeg[1][2] = frame.data.bytes[1] - 40;
      inlettarg[2] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;
    case 0x24B: //phase 3 temp message 2
      curtemplim[2] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x239: //phase 2 temp message 1
      templeg[0][1] = frame.data.bytes[0] - 40;
      templeg[1][1] = frame.data.bytes[1] - 40;
      inlettarg[1] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;
    case 0x249: //phase 2 temp message 2
      curtemplim[1] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    case 0x237: //phase 1 temp message 1
      templeg[0][0] = frame.data.bytes[0] - 40;
      templeg[1][0] = frame.data.bytes[1] - 40;
      inlettarg[0] = frame.data.bytes[5] - 40;
      newframe = newframe | 1;
      break;
    case 0x247: //phase 1 temp message 2
      curtemplim[0] = frame.data.bytes[0] * 0.234375;
      newframe = newframe | 1;
      break;

    // Phase 1 Detail Message
    case 0x207:
      acvolt[0] = frame.data.bytes[1];
      accur[0] = (uint16_t((frame.data.bytes[5] & 0x7F) << 2) | uint16_t(frame.data.bytes[6] >> 6)) ;
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[0] = true;
      }
      else
      {
        ACpres[0] = false;
      }
      x = frame.data.bytes[2] & 0x40;
      if (x != 0)
      {
        ModEn[0] = true;
      }
      else
      {
        ModEn[0] = false;
      }
      x = frame.data.bytes[2] & 0x20;
      if (x != 0)
      {
        ModFlt[0] = true;
      }
      else
      {
        ModFlt[0] = false;
      }
      newframe = newframe | 1;
      break;
      
    // Phase 2 Detail Message
    case 0x209:
      acvolt[1] = frame.data.bytes[1];
      accur[1] = (uint16_t((frame.data.bytes[5] & 0x7F) << 2) | uint16_t(frame.data.bytes[6] >> 6)) ;
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[1] = true;
      }
      else
      {
        ACpres[1] = false;
      }
      x = frame.data.bytes[2] & 0x40;
      if (x != 0)
      {
        ModEn[1] = true;
      }
      else
      {
        ModEn[1] = false;
      }
      x = frame.data.bytes[2] & 0x20;
      if (x != 0)
      {
        ModFlt[1] = true;
      }
      else
      {
        ModFlt[1] = false;
      }
      newframe = newframe | 1;
      break;

    // Phase 3 Detail Message
    case 0x20B:
      acvolt[2] = frame.data.bytes[1];
      accur[2] = (uint16_t((frame.data.bytes[5] & 0x7F) << 2) | uint16_t(frame.data.bytes[6] >> 6)) ;
      x = frame.data.bytes[2] & 12;
      if (x != 0)
      {
        ACpres[2] = true;
      }
      else
      {
        ACpres[2] = false;
      }
      x = frame.data.bytes[2] & 0x40;
      if (x != 0)
      {
        ModEn[2] = true;
      }
      else
      {
        ModEn[2] = false;
      }
      x = frame.data.bytes[2] & 0x20;
      if (x != 0)
      {
        ModFlt[2] = true;
      }
      else
      {
        ModFlt[2] = false;
      }
      newframe = newframe | 1;
      break;

    // DC Feedback : Measured DC battery current and voltage

    // Phase 1
    case 0x227:
      dccur[0] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) ;//* 0.000839233 convert in rest of code
      dcvolt[0] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.01052864; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;

    // Phase 2
    case 0x229:
      dccur[1] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) ;//* 0.000839233 convert in rest of code
      dcvolt[1] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.01052864; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;

    // Phase 3
    case 0x22B:
      dccur[2] = ((frame.data.bytes[5] << 8) + frame.data.bytes[4]) ;//* 0.000839233 convert in rest of code
      dcvolt[2] = ((frame.data.bytes[3] << 8) + frame.data.bytes[2]) * 0.010528564; //we left shift 8 bits to make a 16bit uint.
      newframe = newframe | 2;
      break;

    default:
      // if nothing else matches, do the default
      break;
  }
}

// Decode any CAN frames sent to the charger from other modules (external)
void canextdecode(CAN_FRAME & frame)
{
  // canControl modes
  // 0 = disabled can control, 1 = master, 2 = Elcon master, 3 = slave
  
  int x = 0;  // ?PURPOSE?

// UNDERDEV
  // If charger in Master mode (so it can take CAN commands from the external port)
  if (parameters.canControl == 1)
  {
    // CAN Instruction for charger settings and On/Off
    if (frame.id == (TesLoreanChargerID + 6));
    {
      // Instructions to charger to shutdown, startup, or switch into automatic charging mode
      if (frame.data.bytes[0]==0)     // OFF
      {
        if (state != 0)
        {
          // Switch the Charger off // Disable Auto Charging
          reqdstate = 0;                              // shutdown modules
          parameters.autoEnableCharger = 0;       // Disable Auto Charging
          setting = 1;

          // Info if debugging
          if (candebug == 1){Serial.println("[CANext] Requesting shutdown (state=0)");}
        }
      }
      if (frame.data.bytes[0]==1)     // ON
      {
        if (state == 0)
        {
          // Switch the Charger on // Disable Auto Charging
          reqdstate = 2;          // Start the process of enabling and activating the modules
          tboot = millis();   // Set the time at which the 'start' 'stop' command was issued
          parameters.autoEnableCharger = 0;       // Disable Auto Charging
          setting = 1;

          // Info if debugging
          if (candebug == 1){Serial.println("[CANext] Requesting start-up (state=0)");}
        }
      }
      if (frame.data.bytes[0]==2)     // AUTO, shutdown any charging
      {
        // Switch the Charger into AutoCharging mode
        if (state != 0)
        {
          // Switch the Charger off // Disable Auto Charging
          reqdstate = 0;                              // shutdown modules
        }
        parameters.autoEnableCharger = 0;       // Disable Auto Charging

        // Info if debugging
        if (candebug == 1){Serial.println("[CANext] Requesting return AutoEnabledCharging mode");}

        setting = 1;
      }
      // Note: Considered DC AC VOlts/Current settings via CAN, but very unlikely to be used, and can be done via USB
      // or in code. In the TesLorean it is likely to just be the Trip Computer telling the unit to charge
      // during a scheduled charging event
    }
  }
  
  // If charger in Elcon Master mode
  if (parameters.canControl == 2)
  {
    // Check for Elcon Charge Control message
    if (ElconControlID == frame.id) 
    {
      // Pull in charger settings data from the CAN data
      parameters.voltSet = ((frame.data.bytes[0] << 8) + frame.data.bytes[1]) * 0.1;
      maxdccur = (frame.data.bytes[2] << 8) + frame.data.bytes[3];

      // Test a databyte flag value
      if (frame.data.bytes[4] & 0x01 == 1)
      {
        // Check if chargers are OFF
        if (state == 0)
        {
          // Start the chargers enabling process
          reqdstate = 2;
          // Start the clock (marks entry to state=2)
          tboot = millis();
        }
      }
      else  // Databyte flag was not set
      {
        // Shut the chargers down
        reqdstate = 0;
      }

      // Info if debugging
      if (candebug == 1)
      {
        Serial.println();
        Serial.print("[CANext] State = ");
        Serial.print(state);
        Serial.print(" | Voltage Set = ");
        Serial.print(parameters.voltSet);
        Serial.print(" | Module Current Limit = ");
        Serial.print(modulelimcur);
        Serial.println();
      }

      // Mark time since the last external CAN message came in
      tcan = millis();
    }
  }

  // If THIS charger is in Slave mode
  if (parameters.canControl == 3)
  {
    // Check for Slave charge control message
    if (ControlID == frame.id)
    {
      // Test a databyte flag
      if (frame.data.bytes[0] & 0x01 == 1)
      {
        // If charger is currently OFF
        if (state == 0)
        {
          // Start the charger enabling process
          reqdstate = 2;
          // Mark the time of entry to state=2
          tboot = millis();
        }
      }
      else  // Flag not set
      { 
        // If the gap is > 1sec then reset to zero gap, if gap is > 0.5sec then shutdown chargers
        // ?BUG? ?REDUNDANT? so chargers only shutdown if gap is >0.5sec and <1sec, once over 1sec it won't shutdown
        // 'tcan' is also checked in the main loop, once gap is >0.5sec, it sets state=0 to shut modules down
        if(millis()-slavetimeout > 1000)
        {
        slavetimeout = millis();
        }
        if(millis()-slavetimeout > 500)
        {
          reqdstate = 0;
        }
      }

      // Pull in charger settings data from the CAN data
      parameters.voltSet = (frame.data.bytes[1] << 8) + frame.data.bytes[2];
      maxdccur = (frame.data.bytes[3] << 8) + frame.data.bytes[4];
      modulelimcur  = (frame.data.bytes[5] << 8) + frame.data.bytes[6];

      // Info if debugging
      if (candebug == 1)
      {
        Serial.println();
        Serial.print("[CANext] State = ");
        Serial.print(state);
        Serial.print(" | Voltage Set = ");
        Serial.print(parameters.voltSet);
        Serial.print(" | Module Current Limit = ");
        Serial.print(modulelimcur);
        Serial.println();
      }

      // Mark time since the last external CAN message came in
      tcan = millis();
    }
  }
}

// Sends all the OUTGOING CAN messages
// - Charger internal control and status messages
// - General charger status message with StatusID
// - Elcon message (not only in Elcon mode)
// - DCDC Converter message (if settings indicate, send a target voltage to the Tesla DCDC converter)
// - If in Master mode, Send out status information with ControlID
// - If in Master mode, and chargers are ON (state!=0), then send message with ControlID to switch on slave
// JEC - Comment out the Elcon code (not required)
// JEC - Provide Counters to prevent flooding the external CAN with status messages (some messages wait until the interrupt has fired X times before sending)
void Charger_msgs()
{
  //Set up a structured variable according to due_can library for transmitting CAN data.
  CAN_FRAME outframe;

  // Update the TesLorean Fast/Slow counters and set flags for use in this routine
  bool FastRollover = false;
  bool SlowRollover = false;
  bool VSlowRollover = false;
  TesLoreanFastCycleCount++;
  if (TesLoreanFastCycleCount > FastCycleCountLimit)    // 100*100ms = 1 sec
  {
    TesLoreanFastCycleCount = 0;
    FastRollover = true;
  }
  TesLoreanSlowCycleCount++;
  if (TesLoreanSlowCycleCount > SlowCycleCountLimit)    // 100*100ms = 10 sec
  {
    TesLoreanSlowCycleCount = 0;
    SlowRollover = true;
  }
  TesLoreanVSlowCycleCount++;
  if (TesLoreanVSlowCycleCount > VSlowCycleCountLimit)    // 300*100ms = 30 sec
  {
    TesLoreanVSlowCycleCount = 0;
    VSlowRollover = true;
  }
  
  /////////////////////This msg addresses all modules/////////////////////////////////////////////////
  outframe.id = 0x045c;            // Set our transmission address ID
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  outframe.data.bytes[0] = lowByte(parameters.voltSet);  //Voltage setpoint
  outframe.data.bytes[1] = highByte(parameters.voltSet);//Voltage setpoint
  outframe.data.bytes[2] = 0x14;
  if (bChargerEnabled)
  {
    outframe.data.bytes[3] = 0x2e;
  }
  else outframe.data.bytes[3] = 0x0e;
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x90;
  outframe.data.bytes[7] = 0x8c;
  Can0.sendFrame(outframe);
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////Phase 1 command message////////////////////////////////////////
  outframe.id = 0x042c;            // Set our transmission address ID
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  outframe.data.bytes[0] = 0x42;
  outframe.data.bytes[2] = lowByte(modulelimcur); //AC Current setpoint
  outframe.data.bytes[3] = highByte(modulelimcur); //AC Current setpoint
  if (bChargerEnabled)
  {
    outframe.data.bytes[1] = 0xBB;
    outframe.data.bytes[4] = 0xFE;
  }
  else
  {
    outframe.data.bytes[1] = lowByte(uint16_t(ACvoltIN / 1.2));
    outframe.data.bytes[4] = 0x64;
  }
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x00;
  outframe.data.bytes[7] = 0x00;
  Can0.sendFrame(outframe);
  
  //////////////////////////////Phase 2 command message//////////////////////////////////////////////
  outframe.id = 0x43c;        //phase 2 and 3 are copies of phase 1 so no need to set them up again
  Can0.sendFrame(outframe);
  
  ///////////////////////////////Phase 3 command message/////////////////////////////////////////////
  outframe.id = 0x44c;        //phase 2 and 3 are copies of phase 1 so no need to set them up again
  Can0.sendFrame(outframe);

  ///////////Static Frame every 100ms///////////////////////////////////////////////////////////////////
  outframe.id = 0x368;            // Set our transmission address ID
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  outframe.data.bytes[0] = 0x03;
  outframe.data.bytes[1] = 0x49;
  outframe.data.bytes[2] = 0x29;
  outframe.data.bytes[3] = 0x11;
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x0c;
  outframe.data.bytes[6] = 0x40;
  outframe.data.bytes[7] = 0xff;
  Can0.sendFrame(outframe);
  
  /*////////////////////////////////////////////////////////////////////////////////////////////////////////
            External CAN
  ////////////////////////////////////////////////////////////////////////////////////////////////////////*/
  
  //// GENERAL STATUS MESSAGE /////////////////////////

  // Temp total variables for DCvoltage and ACcurrent
  uint16_t y, z = 0;

  // When not in Slave mode
  outframe.id = StatusID;
  // If in Slave Mode change the status ID (++)
  if (parameters.canControl == 3)
  {
    outframe.id = StatusID + 1;
  }

  // Data frame settings
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request

  // Populate the DC voltage data
  outframe.data.bytes[0] = 0x00;  // ?REDUNDANT? This gets replaced directly after the loop in all cases
  for (int x = 0; x < 3; x++)
  {
    y = y +  dcvolt[x] ;
  }
  outframe.data.bytes[0] = y / 3;

  // Populate the AC current data
  if (parameters.phaseconfig == Singlephase)
  {
    for (int x = 0; x < 3; x++)
    {
      z = z + (accur[x] * 66.66) ;
    }
  }
  else
  {
    z = accur[2] * 66.66;
  }
  outframe.data.bytes[1] = lowByte (z);
  outframe.data.bytes[2] = highByte (z);

  // Populate DCcurrent, Module current limit, Promixity status, Connection type
  outframe.data.bytes[3] = lowByte (uint16_t (totdccur)); //0.005Amp
  outframe.data.bytes[4] = highByte (uint16_t (totdccur));  //0.005Amp
  outframe.data.bytes[5] = lowByte (uint16_t (modulelimcur * 0.66666));
  outframe.data.bytes[6] = highByte (uint16_t (modulelimcur * 0.66666));
  // Write to same byte to apply multiple values (bits) within byte
  outframe.data.bytes[7] = 0x00;
  outframe.data.bytes[7] = Proximity << 6;
  outframe.data.bytes[7] = outframe.data.bytes[7] || (parameters.type << 4);
  Can1.sendFrame(outframe);

  //// CHARGER STATUS INFO ////
  //// Charging Status Info - TesLorean ////
  outframe.id = TesLoreanChargerID + 7;
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request

  // Picked up by the instrument cluster controller and trip computer
  if (state >= 2) // 0 = Inactive, 1 = Active, 2 = Activating
  {
    outframe.data.bytes[0] = 2;  
  }
  else
  {
    outframe.data.bytes[0] = state;  
  }
  // WARNING - Borrows results from the General frame above
  outframe.data.bytes[1] = y / 3;
  outframe.data.bytes[2] = lowByte (z);
  outframe.data.bytes[3] = highByte (z);
  outframe.data.bytes[4] = lowByte (uint16_t (totdccur)); //0.005Amp
  outframe.data.bytes[5] = highByte (uint16_t (totdccur));  //0.005Amp
  outframe.data.bytes[6] = lowByte (uint16_t (modulelimcur * 0.66666));
  outframe.data.bytes[7] = highByte (uint16_t (modulelimcur * 0.66666));
  Can1.sendFrame(outframe);

  //// Only report on the VSLOW basis ~once/30s////
  if (VSlowRollover)
  {
    ///// TEMP REPORTING //////
    // Picked up by the Thermal Module (reported temps and target inlet temps)
    
    //// Charger Temp Info Output - TesLorean ////
    outframe.id = TesLoreanChargerID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request

    // Populate with Module temps (2 per module)
    outframe.data.bytes[0] = templeg[0][0];   // Phase 1 temps
    outframe.data.bytes[1] = templeg[1][0];
    outframe.data.bytes[2] = templeg[1][1];   // Phase 2 temps
    outframe.data.bytes[3] = templeg[1][1];
    outframe.data.bytes[4] = templeg[1][2];   // Phase 3 temps
    outframe.data.bytes[5] = templeg[1][2];
    outframe.data.bytes[6] = 0x00;
    outframe.data.bytes[7] = 0x00;
    Can1.sendFrame(outframe);

    // Populate with Module inlet temp targets (1 per module)
    outframe.id = (TesLoreanChargerID + 1);
    outframe.data.bytes[0] = inlettarg[0];   // Phase 1 inlet target temp
    outframe.data.bytes[1] = inlettarg[1];   // Phase 2 inlet target temp
    outframe.data.bytes[2] = inlettarg[2];   // Phase 3 inlet target temp
    outframe.data.bytes[3] = 0x00;
    outframe.data.bytes[4] = 0x00;
    outframe.data.bytes[5] = 0x00;
    outframe.data.bytes[6] = 0x00;
    outframe.data.bytes[7] = 0x00;
    Can1.sendFrame(outframe);
  }

  //// Only report on the Fast basis ~once/second ////
  if (FastRollover)
  {
    //// CHARGING INFO ////
    
    //// Charging Status Info - TesLorean ////
    outframe.id = TesLoreanChargerID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request

    //// CHARGING INFO ////
    if (state != 0)   // Only report if Charger is active
    {
      // Picked up by the instrument cluster controller and trip computer
      outframe.id = (TesLoreanChargerID + 4);
      outframe.data.bytes[0] = 0x01;  // Hours
      outframe.data.bytes[1] = 0x01;  // Mins
      outframe.data.bytes[2] = 0x01;  // Seconds
      outframe.data.bytes[3] = 0x00;
      outframe.data.bytes[4] = 0x00;
      outframe.data.bytes[5] = 0x00;
      outframe.data.bytes[6] = 0x00;
      outframe.data.bytes[7] = 0x00;
      Can1.sendFrame(outframe);
    }
  }

  /*////////Elcon Message////////////
  // Disabled as redundant for TesLorean
  // ?PURPOSE? This is sent JUST IN CASE there is an Elcon controller
  outframe.id = ElconID;
  outframe.length = 8;            // Data payload 8 bytes
  outframe.extended = 1;          // Extended addresses - 0=11-bit 1=29bit
  outframe.rtr = 0;                 //No request
  // Populate with DCvoltage (y) and total dc current
  outframe.data.bytes[0] = highByte (y * 10 / 3);
  outframe.data.bytes[1] = lowByte (y * 10 / 3);
  outframe.data.bytes[2] = highByte (uint16_t (totdccur * 20)); //0.005Amp conv to 0.1
  outframe.data.bytes[3] = lowByte (uint16_t (totdccur * 20)); //0.005Amp conv to 0.1
  outframe.data.bytes[4] = 0x00;
  outframe.data.bytes[5] = 0x00;
  outframe.data.bytes[6] = 0x00;
  outframe.data.bytes[7] = 0x00;
  Can1.sendFrame(outframe);
  */

  ///DCDC CAN CONTROL ///////////////////

  // Check if setting indicate that Charger should set the DCDC voltage (Tesla DCDC Converter only)
  // JEC - Only need to send out to DCDC every second
  if (dcdcenable && FastRollover)
  {
    outframe.id = DCDCConverterControlID;  // Tesla specific DCDC Converter code
    outframe.length = 3;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request

    // Populate with the target DC voltage for the DCDC converter
    // JEC - Calculate the voltage required once
    // Leave calculation here just in case the Charger will sometimes run in different DC voltage levels
    // Enabling the DCDC to output voltage, makes sure that the charger 12v systems are not running from battery
    uint16_t DCDCVoltageTarget = (uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);
    outframe.data.bytes[0] = highByte (DCDCVoltageTarget);
    outframe.data.bytes[1] = lowByte (DCDCVoltageTarget);
//    outframe.data.bytes[0] = highByte (uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);
//    outframe.data.bytes[1] = lowByte (uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);

    outframe.data.bytes[1] = outframe.data.bytes[1] | 0x20;
    outframe.data.bytes[2] = 0x00;
    Can1.sendFrame(outframe);
  }

  /////////SLAVE CHARGER INSTRUCTIONS ///////////////

  // Check if in Master mode
  if (parameters.canControl == 1)
  {
    outframe.id = ControlID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request

    // Set the default byte 0 value
    outframe.data.bytes[0] = 0;

    // If charger is ON (enabling, enabled, or activated)
    if (state != 0)
    {
      // If Slave charger has been recruited (charge request > 15A), doesn't check to see if slave charge is present
      if (slavechargerenable == 1)
      {
        // Set flag for Slave charger (to switch on the Slave)
        outframe.data.bytes[0] = 0x01;
      }
    }

    // Populate with voltage, max DC current, module current limit
    outframe.data.bytes[1] = highByte(parameters.voltSet);
    outframe.data.bytes[2] = lowByte(parameters.voltSet);
    outframe.data.bytes[3] = highByte(maxdccur);
    outframe.data.bytes[4] = lowByte(maxdccur);
    outframe.data.bytes[5] = highByte(modulelimcur);
    outframe.data.bytes[6] = lowByte(modulelimcur);
    outframe.data.bytes[7] = 0;

    Can1.sendFrame(outframe);
  }
}

// Get the PROXIMITY pin value and set the 'cablelim' appropriately, taking into account source 'type'
void evseread()
{
  uint16_t val = 0;
  val = analogRead(EVSE_PROX);     // read the input pin

  // Type 2 power source connection
  if ( parameters.type == 2)
  {
    if ( val > 950)
    {
      Proximity = Unconnected;
    }
    else
    {
      Proximity = Connected;
      if ( val < 950 && val > 800)
      {
        cablelim = 13000;
      }
      if ( val < 800 && val > 700)
      {
        cablelim = 20000;
      }
      if ( val < 600 && val > 450)
      {
        cablelim = 32000;
      }
      if ( val < 400 && val > 250)
      {
        cablelim = 63000;
      }
    }
  }

  // Type 1 power source connection (e.g. J1772 USA)
  if ( parameters.type == 1)
  {
    if ( val > 800)
    {
      Proximity = Unconnected;
    }
    else
    {
      if ( val > 550)
      {
        Proximity = Buttonpress;
      }
      else
      {
        Proximity = Connected;
      }
    }
  }
}

// Just a shell function ; Interrupt Service Routine for PILOT line square wave signals
void Pilotread()
{
  Pilotcalc();
}

// This function is called by an interrupt that is monitoring the value of the PILOT line
// Calculates the AC Current Limit from the duty cyle of the PILOT line
// keeps updating 'duration since last high read', the when it goes low then calculates duty cyle
void Pilotcalc()
{
  // If the PILOT is high, just calc time since last checked HIGH, else if LOW ???
  if (digitalRead(EVSE_PILOT ) == HIGH)
  {
    // Check that pilottimer has been updated at least once
    if (pilottimer!=0)
    {
      duration = micros() - pilottimer;
    }
    pilottimer = micros();
  }
  else // Pilot LOW
  {
    if (pilottimer!=0 && duration !=0)
    {
      // Calculation of the uptime
      timehigh = micros() - pilottimer;

      // Test to see if timehigh is beyond limits
      if (timehigh > pilotgap)
      {
          // This check previously occurred in ACcurrentlimit()
          accurlim = 0;
          bPilotTimeout = true;
      }
      else
      {      
        //Calculate the duty cycle then multiply by 600 to get mA current limit
        // accurlim = (micros() - pilottimer) * 100 / duration * 600
        // milliamps = dutycycle * 60,000... multiply & divide sequence may avoid a value out of range for accurlim
        accurlim = timehigh * 100 / duration * 600;
      }
    }
  }
}

// Checks for timeout of the pilot signal (more than 1.2secs), if so set the current demand to 0
// Sets the modulelimcur based on Phases, EVSE or not
// Checks modulelimcur against limits set in Parameters
void ACcurrentlimit()
{
  if (parameters.autoEnableCharger == 1)
  {
// DEBUG - This is checking a volatile variable outside of an Interrupt Service Routine (could see byte updated values)
/*    //too big a gap in pilot signal means signal error, kill or disconnected so no current allowed
    if (Proximity == Connected)
    {
      // Note : pilottimer is a volatile variable bytes of which could be updated in the middle of this operation
      if ((micros() - pilottimer) > pilotgap) // This seems to be firing all the time?
      {
        accurlim = 0;
        // Debug
        // if (debugevse != 0){Serial.println("[EVSE] Warning : Too long a gap in pilot signal.");}
      }
    }
    else  // Power is disconnected (Button Pressed or Unconnected)
    {
        accurlim = 0;
        // Debug
        // if (debugevse != 0){Serial.println("[EVSE] Power withdrawn.");}
    }
*/

//////// ?BUG?  ACCURLIM is volatile and could be updated during the ISR Pilotcalc()
    // Grab the accurlim into a temp variable
    uint16_t tempaccurlim = 0;
    noInterrupts();
    tempaccurlim = accurlim;
    interrupts();

    // Calculate the modulelimcur per module
    if (parameters.phaseconfig == Singlephase)
    {

      modulelimcur = (tempaccurlim / 3) * 1.5 ; // all module parallel, sharing AC input current

    }
    else // Threephase
    {
      modulelimcur = tempaccurlim * 1.5; // one module per phase, EVSE current limit is per phase
    }

    // If Type 2 power source, check if modulelimcur is over limit, and if so set to limit value
    if (parameters.type == 2)
    {
      if (modulelimcur > (cablelim * 1.5))
      {
        modulelimcur = cablelim * 1.5;
      }
    }
  }
  else    // Not on AutoEnableCharger
  {
    if (parameters.phaseconfig == Singlephase)
    {
      modulelimcur = (parameters.currReq / 3); // all module parallel, sharing AC input current
    }

    // ?BUG? I would have expected to see an else to handle Threephase in Not AutoEnableCharger mode ???
    // Code shows up on the other side of the canControl IF statement ???
  }

  // If in canControl model for Master or ElconMaster
  if (parameters.canControl == 1 || parameters.canControl == 2)
  {
    if (accurlim * 1.5 > (16000 * 1.5)) //enable second charger if current available >15A // ?REDUNDANT? Multiplication * 1.5
    {
      modulelimcur = modulelimcur * 0.5;

      // Enable the slave charger to assist with charging, gets sent CAN frame to startup
      slavechargerenable = 1;
    }
    else
    {
      // Disable the slave charger from assisting with charging, gets sent CAN frame to stop
      slavechargerenable = 0;
    }
  }
  
  // if evse allows more current then set in parameters limit it
  if (parameters.phaseconfig == Threephase)
  {
    if (modulelimcur > parameters.currReq) //if evse allows more current then set in parameters limit it
    {
      modulelimcur = parameters.currReq;
    }
  }
  else // Singlephase
  {
    if (modulelimcur > (parameters.currReq / activemodules)) 
    {
      modulelimcur = (parameters.currReq / 3);
    }
  }
  
  /*
    if (modulelimcur > (dcaclim * 1.5)) //if more current then max per module or limited by DC output current
    {
    modulelimcur = (dcaclim * 1.5);
    }
  */
}

// Set DC Current Limits
// - Counts the activemodules (looks for modules with >50 acvolts and >50 dcvolts) - '50V must be some kind of minimum'
// - Totals up the DC current, but then doesn't use the value
// Sets the value of 'dcaclim' - looks like calculating DC Power (volts * amps), then AC amps (/acvolts), then spread across modules (/activemodules)
void DCcurrentlimit()
{
  totdccur = 1; // 0.005Amp
  activemodules = 0;
  for (int x = 0; x < 3; x++)
  {
    totdccur = totdccur + (dccur[x] * 0.1678466) ;
    if (acvolt[x] > 50 && dcvolt[x] > 50)
    {
      activemodules++;
    }
  }
  dcaclim = 0;

  // ?BUG/FEATURE? Only referencing the 3rd (0..2) module to calculate the 'dcaccurlim'
  // Could this cause a problem if not all 3 modules are enabled ??
  int x = 2;
  dcaclim = ((dcvolt[x] * (maxdccur + 400)) / acvolt[x]) / activemodules;

  // Make sure the dcaclim doesn't exceed the maximum mAmps for a single module
  // Indications suggest that just as AC power is applied, the lower (than normal, i.e. 150 vs 240) ACvoltage will
  // cause the DCAClim calculation to spike, which may be throwing the modules into FAULT.
  if (dcaclim > maxaccur)
  { 
    // maxaccur should be 16Amps/module
    dcaclim = maxaccur;
  }
}


