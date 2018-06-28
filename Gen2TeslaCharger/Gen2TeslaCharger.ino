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

//*********GENERAL VARIABLE   DATA ******************
uint16_t curset = 0;  // ?BUG? - variable not found in the code
int  setting = 1;     // Flag to indicate if a entry made to change a setting (setting changed = 1, not changed = 0)
int incomingByte = 0; // Temporary variable for last byte received via serial port
int state;            // Charger state (0 = turn chargers off, 1 = turn chargers on, 2 = enable chargers)
bool bChargerEnabled; // Flag to indicate if chargers are enabled or not (state = 1)(false = not enabled, true = enabled)
bool bChargerEnabling;// Flag to indicate that chargers are in the process of enabling (state=2 or state=3)
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
uint16_t cablelim = 0; // Power connection (Type 2 only) cable limit as indicated by Proximity
// Pilot EVSE variables
volatile uint32_t pilottimer = 0;         // Used to calculate the duty cycle on the Pilot line, duty cycle indicates AC current limit
volatile uint16_t timehigh, duration = 0; // 'timehigh' is an unused variable, 'duration' is used in Pilot duty cycle calculation
volatile uint16_t accurlim = 0;           // AC current limit calculated from the Pilot line
volatile int dutycycle = 0;               // Calculated duty cycle of the Pilot line

//*********Single or Three Phase Config VARIABLE   DATA ******************
// Power Source configuration
#define Singlephase 0 // all parallel on one phase Type 1
#define Threephase 1  // one module per phase Type 2

//*********Charger Control VARIABLE   DATA ******************
bool Vlimmode = true;                 // ?UNUSED? - Set charges to voltage limit mode
uint16_t modulelimcur, dcaclim = 0;
uint16_t maxaccur = 16000;            // set maximum AC current in mA (only iused to initially set 'dcaclim' in setup)
uint16_t maxdccur = 45000;            // max DC current output in mA
// activemodules = count of modules that are enabled and active
// slavechargerenable = 0 slave charger not recruited, 1 slave charger recruited (if charge request is > 15A) ; says nothing about slave being present
int activemodules, slavechargerenable = 0;0
// For validation checks
uint16_t maxhiaccur = maxaccur;            // maximum AC current in mA (only iused to initially set 'dcaclim' in setup)
uint16_t maxhidccur = maxdccur;            // maximum limit DC current output in mA
uint16_t maxhivolts = 40000;               // set to 400v; maximum limit Voltage in 0.01V (hundreds of a volt)

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

//ELCON specific - not used in TesLorean
unsigned long ElconID = 0x18FF50E5;
unsigned long ElconControlID = 0x1806E5F4;

void setup()
{
  // Start the USB port communciations
  Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.

  // Output a Charger 'keep alive' message 10 times a second
  Timer3.attachInterrupt(Charger_msgs).start(90000); // charger messages every 100ms

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
    parameters.currReq = 0;               // max current input limit per module, note: 1500 = 1A
    parameters.enabledChargers = 123;     // enable per phase - 123 is all phases - 3 is just phase 3
    parameters.mainsRelay = 48;           // ?BUG? variable is not referenced in the code. It may refer to the line on the microcontroller that outputs to the main s relay
    parameters.autoEnableCharger = 0;     // 1 = enabled, 0 = disabled auto start, with proximity and pilot control
    parameters.canControl = 0;            // 0 = disabled can control, or in the following modes 1 = master, 2 = Elcon master, 3 = slave
    parameters.dcdcsetpoint = 14000;      // voltage setpoint for DCDC Converter in mv (sent via external CAN to DCDC)
    
    // TesLorean COnfiguration
    parameters.voltSet = maxhivolts;      // 1 = 0.01V
    parameters.phaseconfig = Singlephase; //AC input configuration (US=Singlephase, EU=Threephase)
    parameters.type = 1;                  // Socket type1 or 2. Note Type 1 is J1772 USA
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

  dcaclim = maxaccur;

  bChargerEnabled = false; //  ?FIXED? are we supposed to command the charger to charge?
}

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
            Serial.println("Error : Autostart selections are 'a' followed by 0 or 1. Autostart disabled.");
          }
          setting = 1;
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
            Serial.println("Error : Phase entry must be p1 or p3");
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
            Serial.println("Error : Type entry must be t1 or t2");
          }
        }
        break;

      case 'x':   //x for can control enable
        if (Serial.available() > 0)
        {
          parameters.canControl = Serial.parseInt();    // returns 0 if no int found
          if (parameters.canControl > 3)
          {
            Serial.println("Error : CAN control selections are 'x' followed by 0,1,2, or 3. CAN control disabled.");
            parameters.canControl = 0;
          }
          setting = 1;
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
            }
            else
            {
              Serial.println("Error : New setting for Maximum DC Current exceeds allowable maximum. Max DC current unchanged.");
            }
          }
          else
          {
            Serial.println("Error : Max DC Current is 'm' followed by a whole number, e.g. 'm45'. Max DC current unchanged.");
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
            }
            else
            {
              Serial.println("Error : New setting for Voltage exceeds allowable maximum. Max voltage unchanged.");
            }
          }
          else
          {
            Serial.println("Error : Set Voltage is 'v' followed by a whole number, e.g. 'v400'. Max voltage unchanged.");
          }
        }
        break;

      case 's':   //s for start AND stop
        // ?BUG? - there does not need to be additional bytes available on the serial port before toggling start/stop
        //if (Serial.available() > 0)
        //{

          // ?BUG? Why set the LED high here?  Wait until the state setting is processed and switch on/off there
          // digitalWrite(LED_BUILTIN, HIGH);

          // If still in the initializing process (state==2) then don't do anything
          if (state == 2)
          {
            Serial.println("Error : Modules are still initializing, please wait a moment and try again.");
          }
          // If currently stopped - signal the start process (two stage)
          if (state == 0)
          {
            state = 2;          // Start the process of enabling and activating the modules
            tboot = millis();   // Set the time at which the 'start' 'stop' command was issued
            setting = 1;
          }
          // if currently started - signal to stop
          if (state == 1)
          {
            state = 0;          // shutdown modules
            setting = 1;
          }

        //}
        break;

      case 'e':   //e for enabling chargers followed by 1 to 3 digits (comprised of 1,2,3) to indicate which ones to run
        if (Serial.available() > 0)
        {
          // Test for a valid digits string (as integer) and then update setting, otherwise error
          uint8_t  tempCs
          tempCs = Serial.parseInt();
          if (tempCs == 1 || tempCs == 2 || tempCs == 3 || tempCs == 12 || tempCs == 13 || tempCs == 23 || tempCs == 123)
          {
            parameters.enabledChargers = tempCs;
            setting = 1;
          }
          else
          {
            Serial.println("Error : To enable chargers enter 'e' followed by 0,1,2,12,13,23, or 123. No changes made to enabled chargers.");
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
            }
            else
            {
              Serial.println("Error : New setting for Current exceeds allowable maximum. Max current unchanged.");
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

    // Two blank lines
    Serial.println();
    Serial.println(NEW PARAMETERS...);

    if (state == 1)
    {
      Serial.print("Charger = On ");
    }
    else
    {
      Serial.print("Charger = Off");
    }
    Serial.print(" | Enabled Modules = ");
    Serial.print(parameters.enabledChargers);
    Serial.print(" | Phases = ");
    Serial.print(parameters.phaseconfig);
    Serial.print(" | Set voltage = ");
    Serial.print(parameters.voltSet * 0.01f, 0);
    Serial.print("V | Set current lim AC = ");
    Serial.print(parameters.currReq * 0.00066666, 0);
    Serial.print("A | DC = ");
    Serial.print(maxdccur * 0.001, 1);
    Serial.print("A | ");
    if (parameters.autoEnableCharger == 1)
    {
      Serial.print("Autostart On ");
    }
    else
    {
      Serial.print("Autostart Off");
    }
    Serial.print(" | ");
    if (parameters.canControl == 1)
    {
      Serial.print("Can Mode = Master       ");
    }
    if (parameters.canControl == 2)
    {
      Serial.print("Can Mode = Master Elcon ");
    }
    if (parameters.canControl == 3)
    {
      Serial.print("Can Mode = Slave        ");
    }
    Serial.print(" | ");
    if (parameters.phaseconfig == Singlephase)
    {
      Serial.print("Single Phase");
    }
    if (parameters.phaseconfig == Threephase)
    {
      Serial.print("Three Phase ");
    }
    Serial.print(" | ");
    if (parameters.type == 1)
    {
      Serial.print("Type 1");
    }
    if (parameters.type == 2)
    {
      Serial.print("Type 2");
    }
    setting = 0;
    Serial.println();
    Serial.println();
  }

  // When under CanControl, and ElCon/Slave, check that a minimum amount of time (500ms) hasn't passed since 'tcan' was set?
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
        state = 0;
        Serial.println();
        Serial.println("WARNING : CAN (incoming) time-out for slave.");
      }
    }
  }

  // Test that the Enable line is 12v (CHARGER Switch is ON)
  // ?PENDING? - if scheduled charging, the enable line need not be high
  // if Enable line OFF, but request to enable chargers, chargers are enabled, or enabling - set status to shutdown chargers
  if (digitalRead(DIG_IN_1) == LOW && state != 1)   
  {
    // Don't allow chargers to enable OR instruct the chargers to disable
    state = 0;
  }

  // Check the state of the chargers vs their enabled status and take action
  // state 0 : Chargers are not enabled, or if enabled shut them down
  // state 1 : Chargers are activated, or if not active will be activated
  // state 2 : Enable the chargers
  // state 3 : Waiting 500ms after being enabled
  // Sequence 0 -> 2 -> 3 -> 1 (enabled & activated) -> 0 (shutdown)
  // bChargerEnabled : chargers are enabled and active
  // bChargerEnabling : chargers are enabled but not activated
  switch (state)
  {
    // Charger should be in an OFF state or changed to an OFF state
    case 0: 

      // ?PRIOR? - Send the deactivate, disable commands on every loop.  May be required behavior.

      // Check that the chargers are currently enabled
      if (bChargerEnabled == true || bChargerEnabling == true)
      {
        // Note: In prior versions the deactivate and diasble codes were sent on every loop
        digitalWrite(DIG_OUT_1, LOW);//MAINS OFF
        digitalWrite(EVSE_ACTIVATE, LOW);
        digitalWrite(CHARGER1_ACTIVATE, LOW); //chargeph1 deactivate
        digitalWrite(CHARGER2_ACTIVATE, LOW); //chargeph2 deactivate
        digitalWrite(CHARGER3_ACTIVATE, LOW); //chargeph3 deactivate
        digitalWrite(CHARGER1_ENABLE, LOW);//disable phase 1 power module
        digitalWrite(CHARGER2_ENABLE, LOW);//disable phase 2 power module
        digitalWrite(CHARGER3_ENABLE, LOW);//disable phase 3 power module
  
        // Flag that the chargers are now OFF
        bChargerEnabling = false;
        bChargerEnabled = false;
      }
      break;

    // Charger should be in an ON state or changed to an ON state
    case 1:

      // Check that the Chargers are currently listed as OFF
      if (bChargerEnabled == false && bChargerEnabling == true)
      {      
        // Enable the correct chargers
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
        
        // Flag that the chargers are enabled
        bChargerEnabling = false;
        bChargerEnabled = true;

        // Delay 100ms, then signal to switch the mains on (via relay) and activate the EVSE
        delay(100);
        digitalWrite(DIG_OUT_1, HIGH);        // MAINS ON (via relay)
        digitalWrite(EVSE_ACTIVATE, HIGH);    // Signal to EVSE to provide power
      }
      break;

    case 2:

      // ?PRIOR? - Code would send an ENABLE on every loop while in status=2 (which lasts 500ms)
      // Possible that this was a necessary practice (since CAN messages are not guarenteed delivery)

      // Start the enable process for the selected chargers
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
      // Flag that the chargers are in the enabling sequence
      bChargerEnabling = true;
      
      // Set state=3, which will last for 500ms
      state = 3;

      // DEBUG Should 'tboot' be set here? instead of in multiple other locations?
      
      break;    // Missing in prior code, but no ill-effects

    case 3:
      // Check that 500ms has passed since status=2 (signal to start enabling) was set
      if (tboot <  (millis() - 500))
      {
        // After 500ms okay to start the ACTIVATE module process
        state = 1;
      }
      break;

    default:
      // if nothing else matches, do the default
      break;
  }

  // Every 500ms output a debug status message
  if (tlast <  (millis() - 500))
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
      Serial.print(" | Phases = ");
      Serial.print(parameters.phaseconfig);
      if (bChargerEnabled)
      {
        Serial.print(" | Chargers ON");
      }
      else
      {
        Serial.print(" | Chargers OFF");
      }
      if (digitalRead(DIG_IN_1) == HIGH)
      {
        Serial.print(" | EnableLine Hi");
      }
      else
      {
        Serial.print(" | EnableLine Lo");
      }
      /*
        Serial.print(" AC limit : ");
        Serial.print(accurlim);
      */
      Serial.print(" | Cable Limit = ");
      Serial.print(cablelim);
      Serial.print(" | Module Cur Request: ");
      Serial.print(modulelimcur / 1.5, 0);
      /*
        Serial.print(" DC AC Cur Lim: ");
        Serial.print(dcaclim);
        Serial.print(" Active: ");
        Serial.print(activemodules);
      */
      Serial.print(" | DC total Cur = ");
      Serial.print(totdccur * 0.005, 2);
      Serial.print(" | DC Setpoint = ");
      Serial.print(parameters.voltSet * 0.01, 0);
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
        if (state == 0)
        {
          Serial.print("[CHARGER] Modules Turned OFF");
        }
        if (state == 2 || state == 3)
        {
          Serial.print("[CHARGER] Modules Enabling...");
        }
        Serial.println();
      }
      if (debugevse != 0)
      {
        Serial.println();
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
        // ?REDUNDANT? 
        //if (parameters.autoEnableCharger == 1)
        //{
        if (state == 0)
        {
          if (digitalRead(DIG_IN_1) == HIGH)
          {
            // Start the process of initializing the modules
            state = 2;

            // Set a timer recording the start of the initialize request
            // ??Should this be done in the code that handles the state = 2 request?
            tboot = millis();
          }
        //}
        }
      }
      digitalWrite(DIG_OUT_2, HIGH); //enable AC present indication
    }
    else // unplugged or buton pressed stop charging
    {
      state = 0;
      digitalWrite(DIG_OUT_2, LOW); //disable AC present indication

      // ?? Isn't this taken care of in the 'state=0' code ??
      digitalWrite(EVSE_ACTIVATE, LOW);
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

  // 0 = disabled can control, 1 = master, 2 = Elcon master, 3 = slave
  
  int x = 0;  // ?PURPOSE?
  
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
          state = 2;
          // Start the clock (marks entry to state=2)
          tboot = millis();
        }
      }
      else  // Databyte flag was not set
      {
        // Shut the chargers down
        state = 0;
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

  // If charger in Slave mode
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
          state = 2;
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
        state = 0;
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
void Charger_msgs()
{
  //Set up a structured variable according to due_can library for transmitting CAN data.
  CAN_FRAME outframe;  
  
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
  outframe.data.bytes[0] = 0x00;  // ?BUG? This gets replaced directly after the loop in all cases
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
  outframe.data.bytes[7] = 0x00;
  outframe.data.bytes[7] = Proximity << 6;
  outframe.data.bytes[7] = outframe.data.bytes[7] || (parameters.type << 4);
  Can1.sendFrame(outframe);

  /////////Elcon Message////////////

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

  ///DCDC CAN//////////////////////////////////////////////////////////////////////
  // Check if setting indicate that Charger should set the DCDC voltage (Tesla DCDC Converter only)
  if (dcdcenable)
  {
    outframe.id = 0x3D8;
    outframe.length = 3;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request

    // Populate with the target DC voltage for the DCDC converter
    outframe.data.bytes[0] = highByte (uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);
    outframe.data.bytes[1] = lowByte (uint16_t((parameters.dcdcsetpoint - 9000) / 68.359375) << 6);

    outframe.data.bytes[1] = outframe.data.bytes[1] | 0x20;
    outframe.data.bytes[2] = 0x00;
    Can1.sendFrame(outframe);
  }

  ////////////////////////////////////////////////////////////////////

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

// Just a shell function
void Pilotread()
{
  Pilotcalc();
}

// ?CHECK? - if it works correctly it is a very clever routine
// Calculates the AC Current Limit from the duty cyle of the PILOT line
// keeps updating 'duration since last high read', the when it goes low then calculates duty cyle
void Pilotcalc()
{
  // If the PILOT is high, just calc time since last checked HIGH, else if LOW ???
  if (  digitalRead(EVSE_PILOT ) == HIGH)
  {
    duration = micros() - pilottimer;
    pilottimer = micros();
  }
  else
  {
    //Calculate the duty cycle then multiply by 600 to get mA current limit
    accurlim = (micros() - pilottimer) * 100 / duration * 600; 
  }
}

// Checks for timeout of the pilot signal (more than 1.2secs), if so set the current demand to 0
// Sets the modulelimcur based on Phases, EVSE or not
// Checks modulelimcur against limits set in Parameters
void ACcurrentlimit()
{
  if (parameters.autoEnableCharger == 1)
  {
    //too big a gap in pilot signal means signal error, kill or disconnected so no current allowed
    if (micros() - pilottimer > 1200) 
    {
      accurlim = 0;
    }

    // Calculate the modulelimcur per module
    if (parameters.phaseconfig == Singlephase)
    {
      modulelimcur = (accurlim / 3) * 1.5 ; // all module parallel, sharing AC input current
    }
    else // Threephase
    {
      modulelimcur = accurlim * 1.5; // one module per phase, EVSE current limit is per phase
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
  if (parameters.canControl == 1 |parameters.canControl == 2)
  {
    if (accurlim * 1.5 > (16000 * 1.5)) //enable second charger if current available >15A
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
  int x = 2;
  dcaclim = ((dcvolt[x] * (maxdccur + 400)) / acvolt[x]) / activemodules;
}


