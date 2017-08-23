/* RAY_power
 *  Battery monitor and power controller for VK2RAY repeater
 *  V1.0 Battery Voltage Monitor and Power Cut function when battery gets too low
 *       LCD display (if connected) will display
 *       
 *  V2.0 Serial Connection with control function added, LCD removed
 * 
 * Serial terminal operation:
 * ==========================
 * The serial console output is started by CR or LF from the terminal
 * The serial console will close after inactivity timeout
 * Only one serial terminal is available at any time
 * tty0 requires login password (insecure connection)
 * tty1 can also operate in non-human mode with STX/ETX enclosed commands
 */

/*
 * External wiring:
 * A0 - connected to battery voltage, 8.2k/2.2k Voltage divider, Bat 15.6V = 3.3V on A0
 * 4  - relay for power circuit 6
 * .
 * .
 * 9  - relay fpr power circuit 1
 * 
 * Serial (pin 0 & 1) tty0 connected to LoRaWan remote access (insecure)
 * Serial (pin 2 & 3) tty1 connected to Rapsberry Pi 
 * 
 */

#include "power.h"
#include "debug.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define DEVICE_COUNT 5  // Total number of power devices

#define VERSION_MAJOR 1
#define VERSION_MINOR 0

enum pwrStateType {
  sOff,
  sOffDelay,
  sOn,
  sOnDelay
};

typedef struct {
  char description[11];
  byte pin;
  byte offVoltage;    // implied decimal point (e.g. 125 = 12.5V)
  byte onVoltage;     // implied decimal point
  uint16_t onDelay;     // seconds required above onVoltage before switching On
  uint16_t offDelay;    // seconds below offVoltage before switching off
  byte currentState;
  unsigned long offTime;
  unsigned long onTime;
} device_item;

device_item pwr_device[DEVICE_COUNT] = {
  {"2mt Rptr ", 9, 105, 130, 180, 10, sOn, 0, 0},
  {"70cm Rptr", 8, 115, 130, 300, 10, sOn, 0, 0},
  {"Spare CH3", 7, 140, 160, 300, 10, sOff, 0, 0}, 
  {"Spare CH4", 6, 140, 160, 300, 10, sOff, 0, 0},
  {"Telemetry", 5, 115, 130, 300, 10, sOn, 0, 0}
//  {"Relay 6", 4, 115, 13.0, 300, 10, sOn, 0, 0}
};

// MIN MAX for device config
#define ON_V_MIN 105
#define ON_V_MAX 160
#define OFF_V_MIN 95
#define OFF_V_MAX 140
#define ON_DELAY_MIN 5
#define ON_DELAY_MAX 60000
#define OFF_DELAY_MIN 0
#define OFF_DELAY_MAX 60000

// EEPROM Storage address
#define EEPROM_ON_V_ADDR  0         // 1 byte per device
#define EEPROM_OFF_V_ADDR 10        // 1 byte per device
#define EEPROM_ON_DELAY_ADDR 20     // 2 bytes per device
#define EEPROM_OFF_DELAY_ADDR 40    // 2 bytes per device

#define PWR_ON true     // Output state for device ON
#define PWR_OFF false   // Output state for device OFF

// Terminal definitions
#define T1_RX_PIN 2
#define T1_TX_PIN 3

#define TERMINAL_BAUD 9600          // bps
#define TERMINAL_CHAR_TIMEOUT 1000  // ms
#define TERMINAL_TIMEOUT 180        // seconds

#define debugTTY tty0
#define tty0 Serial
SoftwareSerial tty1(T1_RX_PIN, T1_TX_PIN); // RX, TX

#define LED LED_BUILTIN

#define ANALOG_REFERENCE_V 3.3     //Reference for Analog, Voltage at full scale
#define ANALOG_REFERENCE_RAW (float)1023 // 10bit AD converter
#define ANALOG_PRESCALE_A0 4.7273        //15.6V -> 3.3V

static bool tty0_connected = false;
static bool tty1_connected = false;
uint16_t rx_parameter;         // to store a parameter > 1 byte received from tty
unsigned long tty0_disconnect_time;
unsigned long tty1_disconnect_time;

bool tty0_authorized = false;  // !! should be false;

static byte term_config_device = 0;   // device config mode when > 0

#define RX_BUF_SIZE 16
byte rxBuffer[RX_BUF_SIZE];

const char password[] = "20[awarc]17";
#define PWD_LENGTH 11

#define NUL 0
#define STX 2
#define ETX 3
#define EOT 4

bool overrideMode=false;    // manual control of power circuits via terminal

//static float batteryV;
static byte batV;   // battery voltage as byte with implied decimal point
static float scaleFactor = ANALOG_REFERENCE_V / ANALOG_REFERENCE_RAW;

debugLevels currentDebugLevel = L_INFO;

void setup() {
  // turn LED off
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Start terminal interfaces
  tty0.begin(TERMINAL_BAUD);
  tty0.print(F("\r\nStarted tty0\r\n"));
  tty1.begin(TERMINAL_BAUD);
  tty1.print(F("\r\nStarted tty1\r\n"));

  // initialise device parameters
  device_para_init();
 
  // configure pwr device outputs and switch them to the pre-set startup state
  for (int i=0; i < DEVICE_COUNT; i++) {
    pinMode(pwr_device[i].pin, OUTPUT);
    devicePwr(i+1, pwr_device[i].currentState);
  }
}

void loop() {
  readVoltages();
  terminal_loop();
  if (!overrideMode) {
    processDevices();
  }
}

void devicePwr(byte device, byte newPwrState) {
  // we can only process ON or OFF
  if ( (newPwrState != sOn) && (newPwrState != sOff) ) {
    return;
  }
  // index is 0 base, parameter device is 1 based
  device--; 
  // assign new power state to device
  pwr_device[device].currentState = newPwrState;
  // switch ON or OFF
  if (newPwrState == sOn) {
    digitalWrite(pwr_device[device].pin, PWR_ON);
    terminal_print(F("%s ON\n\r"), pwr_device[device].description);
  }
  if (newPwrState == sOff) {
    digitalWrite(pwr_device[device].pin, PWR_OFF);
    terminal_print(F("%s OFF\n\r"), pwr_device[device].description);
  }
}

void processDevices () {
  for (int i=0; i<DEVICE_COUNT; i++) {   
     
    // is the device ON?
    if (pwr_device[i].currentState == sOn) {
      // is battery voltage above cutout?
      if (batV > pwr_device[i].offVoltage) {
        continue; // nothing to do, process next device
      } else {  // battery voltage is below cutout?
        // start the off delay
        pwr_device[i].currentState = sOffDelay;
        pwr_device[i].offTime = millis() + ((unsigned long)pwr_device[i].offDelay * (unsigned long)1000);
      }
    }

    // is the device OFF?
    if (pwr_device[i].currentState == sOff) {
      // is battery below on voltage?
      if (batV < pwr_device[i].onVoltage) {
        continue; // nothing to do, process next device
      } else {  // battery is above ON voltage
        // start to on delay
        pwr_device[i].currentState = sOnDelay;
        pwr_device[i].onTime = millis() + ((unsigned long)pwr_device[i].onDelay * (unsigned long)1000);
      }
    }

    // is the device in OFF delay mode?
    if (pwr_device[i].currentState == sOffDelay) {
      // has the voltage risen back up?
      if (batV > pwr_device[i].offVoltage) {
        // return back to ON status
        pwr_device[i].currentState = sOn;
      } else {  //voltage is still below cutout
        // has the off delay expired?
        if (millis() >= pwr_device[i].offTime) {
          devicePwr(i+1, sOff);    // switch power off
        }
      }
    }

    // is the device in ON delay mode?
    if (pwr_device[i].currentState == sOnDelay) {
      // has the voltage dropped back down?
      if (batV < pwr_device[i].onVoltage) {
        // return back to OFF state
        pwr_device[i].currentState = sOff;
      } else {    // battery is above on voltage
        if (millis() >= pwr_device[i].onTime) {
          devicePwr(i+1, sOn);    // switch power on
        }
      }
    }    
  } // for loop - device
}

/*
 * Battery Voltage - read raw and convert to voltage at AI pin voltage
 */
  
void readVoltages () {
  float pinV = (float)analogRead(A0) * scaleFactor;
  // Use pre-scale to calculate measured voltages (i.e. voltage divider)
  float batteryV = pinV * ANALOG_PRESCALE_A0;
  batV = (int)(batteryV *10);   // convert to byte with implied decimal point
}


/*
 * ===============================================================
 * Utility functions
 * ===============================================================
 */

// print debug output on console interface
void debug(debugLevels level, char *sFmt, ...)  // %f is not supported
{
  if (level > currentDebugLevel) return;  // bypass if level is not high enough
  char acTmp[128];       // place holder for sprintf output
  va_list args;          // args variable to hold the list of parameters
  va_start(args, sFmt);  // mandatory call to initilase args 

  vsprintf(acTmp, sFmt, args);
  debugTTY.print(acTmp);
  // mandatory tidy up
  va_end(args);
  return;
}

/*
 * Perform sanity check on device parameters
 * return: true is OK, otherwise false
 */
bool parameter_check (byte onV, byte offV, uint16_t onD, uint16_t offD) {

  if (onV <= offV) return false;
  if (onV < ON_V_MIN) return false;
  if (onV > ON_V_MAX) return false;

  if (offV < OFF_V_MIN) return false;
  if (offV > OFF_V_MAX) return false;

  if (onD < ON_DELAY_MIN) return false;
  if (onD > ON_DELAY_MAX) return false;

  if (offD < OFF_DELAY_MIN) return false;
  if (offD > OFF_DELAY_MAX) return false;
  
  return true;
}

bool device_para_check (byte devNum) {
  if (devNum > DEVICE_COUNT) return false;
  devNum--;   // zero based index for array

  return parameter_check (pwr_device[devNum].onVoltage, pwr_device[devNum].offVoltage, pwr_device[devNum].onDelay, pwr_device[devNum].offDelay );
}

/*
 * Read device parameters from EEPROM
 * devNum = device number, 1... DEVICE_COUNT
 */
bool device_para_read (byte devNum) {
  if (devNum > DEVICE_COUNT) return false;
  devNum--;   // zero based index for array
  byte onV, offV;
  uint16_t onDelay, offDelay;
  int addr;

  // read parameter values from EEPROM
  onV = EEPROM.read(EEPROM_ON_V_ADDR + devNum);
  offV = EEPROM.read(EEPROM_OFF_V_ADDR + devNum);
  addr = EEPROM_ON_DELAY_ADDR + devNum*2;
  onDelay = (EEPROM.read(addr) << 8);
  onDelay += EEPROM.read(++addr);
  addr = EEPROM_OFF_DELAY_ADDR + devNum*2;
  offDelay = (EEPROM.read(addr) << 8);
  offDelay += EEPROM.read(++addr);

  // perform sanity check
  if (!parameter_check(onV, offV, onDelay, offDelay)) 
    return false;

  // store parameters in device
  pwr_device[devNum].onVoltage = onV;
  pwr_device[devNum].offVoltage = offV;
  pwr_device[devNum].onDelay = onDelay;
  pwr_device[devNum].offDelay = offDelay;
  return true;
}

/*
 * Initialized device parameters from EEPROM data
 */
bool device_para_init () {
  byte i;
  for (i=1; i<=DEVICE_COUNT; i++) {
    if (!device_para_read(i)) {
      debug(L_ERROR, "\n\rDevice %d: parameters stored in EEPROM are invalid and have been ignored", i); 
    }
  }
}

/*
 * Write device parameters to EEPROM
 * devNum = device number, 1... DEVICE_COUNT
 */
bool device_para_write (byte devNum) {
  int addr;
  if (devNum > DEVICE_COUNT) return false;
  // sanity check on parameter values
  if ( !device_para_check(devNum) ) return false;
  devNum--;   // zero based index for array
  
  EEPROM.write(EEPROM_ON_V_ADDR + devNum, pwr_device[devNum].onVoltage );
  EEPROM.write(EEPROM_OFF_V_ADDR + devNum, pwr_device[devNum].offVoltage );
  addr = EEPROM_ON_DELAY_ADDR + devNum*2;
  EEPROM.write(addr, pwr_device[devNum].onDelay >> 8);
  EEPROM.write(++addr, pwr_device[devNum].onDelay & 0x00FF);
  addr = EEPROM_OFF_DELAY_ADDR + devNum *2;
  EEPROM.write(addr, pwr_device[devNum].offDelay >> 8);
  EEPROM.write(++addr, pwr_device[devNum].offDelay & 0x00FF);
  return true;
}


/*
 * ===============================================================
 * Terminal functions
 * ===============================================================
 */

void terminal_print_state(byte device) {
  switch (pwr_device[device].currentState) {
    case sOn:
      terminal_print(F("On"));
      break;
    case sOff:
      terminal_print(F("Off"));
      break;
    case sOnDelay:
      terminal_print(F("On Delay (On in %lds)"), (pwr_device[device].onTime - millis()) / 1000 );
      break;
    case sOffDelay:
      terminal_print(F("Off Delay (Off in %lds)"), (pwr_device[device].offTime - millis()) / 1000 );
      break;
    default:
      terminal_print(F("Undefined"));
      break;
  }
}

/*
 * print device info for all devices
 */
void terminal_info() {
  terminal_print(F("\n\rDevice Settings:\n\r"));
  terminal_print(F("Device       OnV   OnD  OffV   OffD\n\r"));
  for (int i=0; i < DEVICE_COUNT; i++) {
    terminal_print(F("%d: %s %3dV %4ds %3dV %4ds\r\n"), i+1, pwr_device[i].description, pwr_device[i].onVoltage, pwr_device[i].onDelay, pwr_device[i].offVoltage, pwr_device[i].offDelay);
  }
  terminal_print(F("\n\r"));
}

/*
 * print device status for all devices
 */
void terminal_status() {
  terminal_print(F("\n\rDevice Status:\n\r"));
  for (int i=0; i < DEVICE_COUNT; i++) {
    terminal_print(F("%d: %s is "), i+1, pwr_device[i].description);
    terminal_print_state(i);
    terminal_print(F("\n\r"));
  }
  terminal_print(F("\n\r"));
}

/*
 * print supply voltage
 */
void terminal_voltage() {
  terminal_print(F("\n\r============================\n\r"));
  terminal_print(F("Battery Voltage: %d\n\r"), batV );
}

/*
 * print device configuration menu
 */
void terminal_device_config_menu() {
  terminal_print(F("Device configuration menu\n\n\r"));
  // is a valid device selected ?
  if (term_config_device > DEVICE_COUNT) {
    terminal_print(F("\n\r [1..%d] Select Device\n\r"), DEVICE_COUNT);
  } else {
    terminal_print(F("Selected Device: %d - %s:\n\r"), term_config_device, pwr_device[term_config_device-1].description);
    terminal_print(F("ON  Voltage: %d   Delay: %d\n\r"), pwr_device[term_config_device-1].onVoltage, pwr_device[term_config_device-1].onDelay);
    terminal_print(F("OFF Voltage: %d   Delay: %d\n\r"), pwr_device[term_config_device-1].offVoltage, pwr_device[term_config_device-1].offDelay);
    terminal_print(F("\n\r  nxxx  - set new ON Voltage [Volts * 10]\n\r"));
    terminal_print(F("  fxxx  - set new OFF Voltage [Volts * 10]\n\r"));
    terminal_print(F("  txxx  - set new ON Delay [seconds]\n\r"));
    terminal_print(F("  uxxx  - set new OFF Delay [seconds]\n\r"));
    terminal_print(F("  s     - save device parameters to EEPROM\n\r"));
    terminal_print(F("  r     - read device parameters from EEPROM\n\r"));
  }
  terminal_print(F("  x     - exit device configuration\n\r"));
  terminal_print(F("\n\r"));
}

void terminal_menu() {
  terminal_print(F("\n\rPower Management V%d.%d\n\r"), VERSION_MAJOR, VERSION_MINOR);
  if (overrideMode) {
    terminal_print(F("!!! --- Override Mode Active - Battery Monitor inactive --- !!!\n\r"));
  }

  if (term_config_device > 0) {
    terminal_device_config_menu();
    return;
    }
  
  terminal_print(F("\n\r  fx - power off, x=1..%d (device number)\n\r"), DEVICE_COUNT);
  terminal_print(F("  nx - switch on, x=1..%d (device number)\n\r"), DEVICE_COUNT);
  if (overrideMode) {
    terminal_print(F("  o -  toggle override mode - NOW ACTIVE !\n\r"));
  } else {
    terminal_print(F("  o -  toggle override mode (manual power control)\n\r"));
  }
  terminal_print(F("  s  - status\n\r"));
  terminal_print(F("  i  - info\n\r"));
  terminal_print(F("  v  - battery voltage\n\r"));
  terminal_print(F("  d  - device config\n\r"));
  terminal_print(F("  x  - exit (close connection)\n\r"));
  if (overrideMode) {
    terminal_print(F("\n\r!!! --- Override Mode Active - Battery Monitor inactive --- !!!\n\r"));
  }
  terminal_print(F("\n\rEnter command:\n\r"));
}

void terminal_invalid() {
  terminal_print(F("\n\rInvalid Command\n\r"));
}

byte terminal_verify_device(byte asciiDevNum, bool printInvalid) {
  // returns the device number (1..DEVICE_COUNT) or 0 if device was invalid
  // returns 0 for an invalid device
  // check if device number is in range
  if ( (asciiDevNum > ('0'+DEVICE_COUNT)) || (asciiDevNum < '1') ) {
    if (printInvalid) {
      terminal_print(F("\n\rInvalid device number %c (range is 1 to %d)\n\r"), asciiDevNum, DEVICE_COUNT);
    }
    return (0);
  }
  return (asciiDevNum - 48);
}

void terminal_device_config() {
  term_config_device = 99;
}

void terminal_process_device_config_cmd(byte cmd, uint16_t parameter) {
  byte devNum;
  bool valid_device = false;
  
  // check if we have a valid device selected
  if ( (term_config_device > 0) && (term_config_device <= DEVICE_COUNT) )
    valid_device = true;
    
  switch(cmd) {
    case 'n':   // ON Voltage
      if (valid_device) {
        if (parameter_check(rx_parameter, pwr_device[term_config_device-1].offVoltage, pwr_device[term_config_device-1].onDelay, pwr_device[term_config_device-1].offDelay) )
          pwr_device[term_config_device-1].onVoltage = rx_parameter;
        else
          terminal_print(F("\r\nError: Specified On Voltage is out of range, change ignored\r\n"));
      }
      break;
    case 'f':   // OFF Voltage
      if (valid_device) {
        if (parameter_check(pwr_device[term_config_device-1].onVoltage, rx_parameter,  pwr_device[term_config_device-1].onDelay, pwr_device[term_config_device-1].offDelay) )
          pwr_device[term_config_device-1].offVoltage = rx_parameter;
        else
          terminal_print(F("\r\nError: Specified Off Voltage is out of range, change ignored\r\n"));
      }
      break;
    case 't': // ON Delay
      if (valid_device) {
        if (parameter_check(pwr_device[term_config_device-1].onVoltage, pwr_device[term_config_device-1].offVoltage, rx_parameter, pwr_device[term_config_device-1].offDelay) )
          pwr_device[term_config_device-1].onDelay = rx_parameter;
        else
          terminal_print(F("\r\nError: Specified On Delay is out of range, change ignored\r\n"));
      }
      break;
    case 'u': // OFF Delay
      if (valid_device) {
        if (parameter_check(pwr_device[term_config_device-1].onVoltage, pwr_device[term_config_device-1].offVoltage, pwr_device[term_config_device-1].onDelay, rx_parameter) )
          pwr_device[term_config_device-1].offDelay = rx_parameter;
        else
          terminal_print(F("\r\nError: Specified Off Delay is out of range, change ignored\r\n"));
      }
      break;
    case 's': // Save values to EEPROM
      if (valid_device) {
        device_para_write(term_config_device);
      }
      break;
    case 'r': // Read values to EEPROM
      if (valid_device) {
        device_para_read(term_config_device);
      }
      break;
    case 'x': // exit device config menu
      term_config_device = 0;
      break;
    default:
      // did we select a new device number
      devNum = terminal_verify_device(cmd, true);
      if ( devNum != 0 ) {
        term_config_device = devNum;
      } 
      break;
  }
  terminal_menu();
}

void terminal_process_cmd(byte cmd, uint16_t parameter) {
  byte devNum;
  if (term_config_device > 0) {
    terminal_process_device_config_cmd (cmd, parameter);
    return;
  }
  switch(cmd) {
    case 'f':
      devNum = terminal_verify_device(parameter, true) ;
      if (devNum)
        devicePwr(devNum, sOff);
      break;
    case 'n':
      devNum = terminal_verify_device(parameter, true) ;
      if (devNum)
        devicePwr(devNum, sOn);
      break;
    case 'o':
      overrideMode = !overrideMode;
      break;
    case 'i':
      terminal_info();
      break;
    case 's':
      terminal_voltage();
      terminal_status();
      break;
    case 'v':
      terminal_voltage();
      break;
    case 'd':
      terminal_device_config();
      break;
    case 'x':
      terminal_disconnect();
    default:
      terminal_invalid();
      break;
  }
  terminal_menu();
}

void tty1_process_data() {
  char txBuffer[16];
  switch (rxBuffer[1]) {
    case 'v':
      sprintf(txBuffer, "%c%d%c%c", STX, batV, ETX, EOT);
      break;
    default:
      sprintf(txBuffer, "%c", EOT);
      break;
  }
  // send txBuffer up to and including EOT
  for (int i=0; i<15; i++) {
    tty1.write(txBuffer[i]);
    if (txBuffer[i] == EOT) break;
  }
}

bool terminal_rx_parameter (char *rxbuf, byte rxlen) {
  int i=rxlen; 
  int m=1;

  // work through string backwards starting at string end, ddo not evaluate first char (it's the command) 
  while(--i > 0) {
    // do we have a numeric char?
    if ( (rxbuf[i] < '0') || (rxbuf[i] > '9') ) {
      continue; // not numeric -> skip to next char
    }
    // found a numeric char
    rx_parameter += (rxbuf[i]-'0') * m;
  //debug(L_INFO, "\n\rEvaluating [%s] len %d, i=%d, %c - %d\r\n", rxbuf, rxlen, i, rxbuf[i], rx_parameter);  
    m*=10;  // switch decimal multiplier to next digit   
  }
}

int terminal_rx(byte tty) {
  int i,x,m;
  // clear RX buffer
  for (i=0; i<RX_BUF_SIZE; i++) {
    rxBuffer[i] = 0;
  }
  i = 0;
  rx_parameter = 0;
  unsigned long timeout = millis() + TERMINAL_CHAR_TIMEOUT;   // critial for terminals which do not wait for CR/LF before sending
  while (millis() <= timeout) {
    if ( (tty == 0) && tty0.available() ) {
      rxBuffer[i++] = tty0.read();
      timeout = millis() + TERMINAL_CHAR_TIMEOUT;
    }
    if ( (tty == 1) && tty1.available() ) {
      rxBuffer[i++] = tty1.read();
      timeout = millis() + TERMINAL_CHAR_TIMEOUT;
    }
  }
  rxBuffer[i] = 0; // mark string end

  // have we received more than just a one char command?
  if (rxBuffer[1] >= ' ') {  // yes - convert parameter to integer;
    terminal_rx_parameter(rxBuffer, i);
    //debug(L_INFO, "\n\r!! Got parameter value <%d>\r\n", rx_parameter);
  } 
  return i;  
}

bool terminal_login() {
  for (int i=0; i<PWD_LENGTH; i++) {
    if (rxBuffer[i] != password[i]) return false;
  }
  return true;
}

void terminal_disconnect() {
  if (tty0_connected) {
    terminal_print(F("tty0 disconnected\n\r"));
    tty0_authorized = false;
    tty0_connected = false;
  }
  if (tty1_connected) {
    terminal_print(F("tty1 disconnected\n\r"));
    tty1_connected = false;
  }
  term_config_device = 0;
}

/*
 * print to terminal
 * sFmt:    string stored in Flash Memory using the "F()" macro
 * storing the string in Flash memory leaves more memory for heap (local data) and variables
 */
void terminal_print( const __FlashStringHelper *sFmt, ...) {
  char sFormat[256];     // to hold format string from FLASH memory
  char acTmp[128];       // place holder for sprintf output

  // move string from FLASH to SRAM (local memory)
  strcpy_PF(sFormat, sFmt);
  
  va_list args;          // args variable to hold the list of parameters
  va_start(args, sFormat);  // mandatory call to initilase args 

  vsprintf(acTmp, sFormat, args);
  if (tty0_connected) {
    tty0.print(acTmp);
  }
  if (tty1_connected) {
    tty1.print(acTmp);
  }

  // mandatory tidy up
  va_end(args);
}

void terminal_loop () {
  byte rx_char;

  // tty0
  if (tty0.available()) {
    terminal_rx(0);
    if (tty0_connected) {
      // tty0 needs authorisation, it is subject to breakin attempts via RF
      if (tty0_authorized) {
        terminal_process_cmd(rxBuffer[0],rxBuffer[1]);
      } else {
        if (terminal_login()) {
          tty0_authorized = true;
          terminal_menu();
        } else {
          terminal_print(F(" failed.\n\r"));
          terminal_disconnect();
        }
      }      
    } else {  // not connected
      if ( (rxBuffer[0] == 0x0A) || (rxBuffer[0] == 0x0D) ){
        if (tty1_connected) {
          tty0.print(F("\n\rtty1 is busy - unable to connect on tty0\n\r"));
          terminal_print(F("\nlogin attempt on tty0\n"));
        } else {
          tty0_connected = true;
          tty0_authorized = false;
          terminal_print(F("\n\rVK2RAY\n\rconnected tty0, timeout %ds\n\rlogin:"), TERMINAL_TIMEOUT);
        }
      }      
    }
    tty0_disconnect_time = millis() + ((unsigned long) TERMINAL_TIMEOUT * (unsigned long) 1000);
  }

  // tty1
  if (tty1.available()) {
    terminal_rx(1);
    if (tty1_connected) {
      terminal_process_cmd(rxBuffer[0],rxBuffer[1]);
    } else {  // not connected
      if ( (rxBuffer[0] == 0x0A) || (rxBuffer[0] == 0x0D) ) {
        if (tty0_connected) {
          tty0.print(F("\n\rtty0 is busy - unable to connect on tty1\n\r"));
          terminal_print(F("\n\rlogin attempt on tty1\n\r"));
        } else {
          tty1_connected = true;
          terminal_print(F("\n\rVK2RAY\n\rconnected tty1, timeout %ds\n\r"), TERMINAL_TIMEOUT);
          terminal_menu();
        }
      }
      // STX is a non-human request (from Raspberry Pi "aprslog" program) for measurements
      if (rxBuffer[0] == STX) {
        tty1_process_data();
      }
    }
    tty1_disconnect_time = millis() + ((unsigned long) TERMINAL_TIMEOUT * (unsigned long) 1000);
  }

  // check timeout
  if ( (tty0_connected) && (millis() > tty0_disconnect_time) ) {
    terminal_print(F("\ntimeout ..... "));
    terminal_disconnect();
  }
  if ( (tty1_connected) && (millis() > tty1_disconnect_time) ) {
    terminal_print(F("\ntimeout ..... "));
    terminal_disconnect();
  }
}


