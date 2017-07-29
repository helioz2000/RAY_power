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
  char description[20];
  byte pin;
  float offVoltage;
  float onVoltage;
  uint16_t onDelay;     // seconds required above onVoltage before switching On
  uint16_t offDelay;    // seconds below offVoltage before switching off
  byte currentState;
  unsigned long offTime;
  unsigned long onTime;
} device_item;

device_item pwr_device[DEVICE_COUNT] = {
  {"2mt Repeater", 9, 10.5, 11.0, 180, 10, sOn, 0, 0},
  {"70cm Repeater", 8, 11.5, 13.0, 300, 10, sOn, 0, 0},
  {"Spare CH3", 7, 11.5, 13.0, 300, 10, sOn, 0, 0}, 
  {"Spare CH4", 6, 11.5, 13.0, 300, 10, sOn, 0, 0},
  {"Telemetry", 5, 11.5, 13.0, 300, 10, sOn, 0, 0}
//  {"Relay 6", 4, 11.5, 13.0, 300, 10, sOn, 0, 0}
};

#define PWR_ON true     // Output state for device ON
#define PWR_OFF false   // Output state for device OFF

#define T1_RX_PIN 2
#define T1_TX_PIN 3

#define TERMINAL_BAUD 9600

#define debugTTY tty0
#define tty0 Serial
SoftwareSerial tty1(T1_RX_PIN, T1_TX_PIN); // RX, TX

#define LED LED_BUILTIN

#define ANALOG_REFERENCE_V 3.3     //Reference for Analog, Voltage at full scale
#define ANALOG_REFERENCE_RAW (float)1023 // 10bit AD converter
#define ANALOG_PRESCALE_A0 4.7273        //15.6V -> 3.3V

static bool tty0_connected = false;
static bool tty1_connected = false;

unsigned long tty0_disconnect_time;
unsigned long tty1_disconnect_time;

bool tty0_authorized = false;

#define TERMINAL_TIMEOUT 180    // seconds

#define RX_BUF_SIZE 16
byte rxBuffer[RX_BUF_SIZE];

const char password[] = "20[awarc]17";
#define PWD_LENGTH 11

#define NUL 0
#define STX 2
#define ETX 3
#define EOT 4

bool overrideMode=false;    // manual control of power circuits via terminal

static float batteryV;
static float scaleFactor = ANALOG_REFERENCE_V / ANALOG_REFERENCE_RAW;

debugLevels currentDebugLevel = L_INFO;

void setup() {
  // turn LED off
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // configure pwr device outputs and switch them the pre-set state
  for (int i=0; i < DEVICE_COUNT; i++) {
    pinMode(pwr_device[i].pin, OUTPUT);
    devicePwr(i+1, pwr_device[i].currentState);
  }

  // Start terminal interfaces
  tty0.begin(TERMINAL_BAUD);
  tty0.println("Started tt0");
  tty1.begin(TERMINAL_BAUD);
  tty1.println("Started tt1");
}

void loop() {
  readVoltages();
  terminal_loop();
  if (!overrideMode) {
    processDevices();
  }
}

void processDevices () {
  for (int i=0; i<DEVICE_COUNT; i++) {   
     
    // is the device ON?
    if (pwr_device[i].currentState == sOn) {
      // is battery voltage above cutout?
      if (batteryV > pwr_device[i].offVoltage) {
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
      if (batteryV < pwr_device[i].onVoltage) {
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
      if (batteryV > pwr_device[i].offVoltage) {
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
      if (batteryV < pwr_device[i].onVoltage) {
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

void readVoltages () {
  // Batter Voltage
  // read raw and convert to voltage at AI pin voltage
  float pinV = (float)analogRead(A0) * scaleFactor;
  // Use pre-scale to calculate measured voltages (i.e. voltage divider)
  batteryV = pinV * ANALOG_PRESCALE_A0;
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
    terminal_print("%s ON\n\r", pwr_device[device].description);
  }
  if (newPwrState == sOff) {
    digitalWrite(pwr_device[device].pin, PWR_OFF);
    terminal_print("%s OFF\n\r", pwr_device[device].description);
  }
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
 * ===============================================================
 * Terminal functions
 * ===============================================================
 */

void terminal_print_state(byte device) {
  switch (pwr_device[device].currentState) {
    case sOn:
      terminal_print("On");
      break;
    case sOff:
      terminal_print("Off");
      break;
    case sOnDelay:
      terminal_print("On Delay");
      break;
    case sOffDelay:
      terminal_print("Off Delay");
      break;
    default:
      terminal_print("Undefined");
      break;
  }
}

void terminal_status() {
  terminal_print("\n\rDevice Status:\n\r");
  for (int i=0; i < DEVICE_COUNT; i++) {
    terminal_print(pwr_device[i].description);
    terminal_print(" is ");
    terminal_print_state(i);
    terminal_print("\n\r");
  }
  terminal_print("\n\r");
}

void terminal_voltage() {
  terminal_print("\n\r============================\n\r");
  terminal_print("Battery Voltage: %d.%dV\n\r", (int) batteryV, (int) (batteryV *100) % 100);  
}

void terminal_menu() {
  terminal_print("\n\rPower Management V%d.%d\n\r", VERSION_MAJOR, VERSION_MINOR);
  if (overrideMode) {
    terminal_print("!!! --- Override Mode Active - Battery Monitor inactive --- !!!\n\r");
  }
  terminal_print("\n\r  fx - power off, x=1..%d (device number)\n\r", DEVICE_COUNT);
  terminal_print("  nx - switch on, x=1..%d (device number)\n\r", DEVICE_COUNT);
  if (overrideMode) {
    terminal_print("  o -  toggle override mode - NOW ACTIVE !\n\r");
  } else {
    terminal_print("  o -  toggle override mode (manual power control)\n\r");
  }
  terminal_print("  s  - status\n\r");
  terminal_print("  v  - battery voltage\n\r");
  terminal_print("  x  - exit (close connection)\n\r");
  if (overrideMode) {
    terminal_print("\n\r!!! --- Override Mode Active - Battery Monitor inactive --- !!!\n\r");
  }
  terminal_print("\n\rEnter command:\n\r");
}

void terminal_invalid() {
  terminal_print("\n\rInvalid Command\n\r");
}

byte terminal_verify_device(byte asciiDevNum, bool printInvalid) {
  // returns the device number (1..DEVICE_COUNT) or 0 if device was invalid
  // returns 0 for an invalid device
  // check if device number is in range
  if ( (asciiDevNum > ('0'+DEVICE_COUNT)) || (asciiDevNum < '1') ) {
    if (printInvalid) {
      terminal_print("\n\rInvalid device number %c (range is 1 to %d)\n\r", asciiDevNum, DEVICE_COUNT);
    }
    return (0);
  }
  return (asciiDevNum - 48);
}

void terminal_process_cmd(byte cmd, byte parameter) {
  byte devNum;
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
    case 's':
      terminal_voltage();
      terminal_status();
      break;
    case 'v':
      terminal_voltage();
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
      sprintf(txBuffer, "%c%d.%d%c%c", STX, (int) batteryV, (int) (batteryV *100) % 100, ETX, EOT);
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

int terminal0_rx() {
  int i;
  // clear RX buffer
  for (i=0; i<RX_BUF_SIZE; i++) {
    rxBuffer[i] = 0;
  }
  i = 0;
  unsigned long timeout = millis() + 250;   // critial for terminals which send one character at a time
  while (millis() <= timeout) {
    if (tty0.available()) {
      rxBuffer[i++] = tty0.read();
    }
  }
  return i;
}

int terminal1_rx() {
  // receive from uart until timeout
  int i;
  // clear RX buffer
  for (i=0; i<RX_BUF_SIZE; i++) {
    rxBuffer[i] = 0;
  }
  i = 0;
  unsigned long timeout = millis() + 200;
  while (millis() <= timeout) {
    if (tty1.available()) {
      rxBuffer[i++] = tty1.read();
      if (i >= RX_BUF_SIZE) i--;  // prevent buffer overrun      
    }
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
    terminal_print("tty0 disconnected\n\r");
    tty0_authorized = false;
    tty0_connected = false;
  }
  if (tty1_connected) {
    terminal_print("tty1 disconnected\n\r");
    tty1_connected = false;
  }
}

void terminal_print(char *sFmt, ...) {
  char acTmp[128];       // place holder for sprintf output
  va_list args;          // args variable to hold the list of parameters
  va_start(args, sFmt);  // mandatory call to initilase args 

  vsprintf(acTmp, sFmt, args);
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
    terminal0_rx();
    if (tty0_connected) {
      // tty0 needs authorisation, it is subject to breakin attempts via RF
      if (tty0_authorized) {
        terminal_process_cmd(rxBuffer[0],rxBuffer[1]);
      } else {
        if (terminal_login()) {
          tty0_authorized = true;
          terminal_menu();
        } else {
          terminal_print(" failed.\n\r");
          terminal_disconnect();
        }
      }      
    } else {  // not connected
      if ( (rxBuffer[0] == 0x0A) || (rxBuffer[0] == 0x0D) ){
        if (tty1_connected) {
          tty0.print("\n\rtty1 is busy - unable to connect on tty0\n\r");
          terminal_print("\nlogin attempt on tty0\n");
        } else {
          tty0_connected = true;
          tty0_authorized = false;
          terminal_print("\n\rVK2RAY\n\rconnected tty0, timeout %ds\n\rlogin:", TERMINAL_TIMEOUT);
        }
      }      
    }
    tty0_disconnect_time = millis() + ((unsigned long) TERMINAL_TIMEOUT * (unsigned long) 1000);
  }

  // tty1
  if (tty1.available()) {
    terminal1_rx();
    if (tty1_connected) {
      terminal_process_cmd(rxBuffer[0],rxBuffer[1]);
    } else {  // not connected
      if ( (rxBuffer[0] == 0x0A) || (rxBuffer[0] == 0x0D) ) {
        if (tty0_connected) {
          tty0.print("\n\rtty0 is busy - unable to connect on tty1\n\r");
          terminal_print("\n\rlogin attempt on tty1\n\r");
        } else {
          tty1_connected = true;
          terminal_print("\n\rVK2RAY\n\rconnected tty1, timeout %ds\n\r", TERMINAL_TIMEOUT);
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
    terminal_print("\ntimeout ..... ");
    terminal_disconnect();
  }
  if ( (tty1_connected) && (millis() > tty1_disconnect_time) ) {
    terminal_print("\ntimeout ..... ");
    terminal_disconnect();
  }
}

