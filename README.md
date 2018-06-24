# TesLorean-Charger-Controller
Custom adaptations to the Damien Maguire &amp; Tom Debree Tesla 10kW charger control software

Damien Maguire of EVBMW.com created a replacement hardware controller for the Tesla Gen2 10kW Charger.  Damien and then Tom Debree wrote software for the controller that allows it to be controlled by Serial commands, auto-start based on settings in EEPROM memory, or with soms CANbus controls.

In the TesLorean application, the charger controller will work in conjunction with the Battery Controller, the Thermal Controller, the Tesla DCDC Converter, and the Trip Controller (center console computer) to manage and schedule charging tasks. The TesLorean controllers communicate via CANbus, sending and receiving data frames to register the status of the other modules and to issue instructions.

The TesLorean-Charger-Controller will be configured to send/receive CANbus messages known to the other contollers.  The module will remain controllable via serial bus (a USB port on the controller card) for configuration, programming, and maintenance purposes.  In operation the controller will auto-start charging if 1) the battery needs charging, 2) the J1772 port (240v 40amp) is plugged in, and 3) the CHARGE button is on in the cabin. The controller will start scheduled charging if 1) the battery needs charging, 2) the J1772 port is plugged in, and 3) the scheduled time has passed.
