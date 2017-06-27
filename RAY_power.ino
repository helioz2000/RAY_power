/* RAY_power
 *  Battery monitor and power controller for VK2RAY repeater
 *  V1.0 Battery Voltage Monitor and Power Cut function when battery gets too low
 *       LCD display (if connected) will display
 *       
 *  V2.0 Serial Connection with control function added, LCD removed
 * 
 * Serial terminal operation:
 * ==========================
 * The serial console output is switched is started by CR+CR+CR
 * The serial port will close after inactivity timeout
 * 
 */

/*
 * External wiring:
 * A0 - connected to battery voltage, 8.2k/2.2k Voltage divider, Bat 15.6V = 3.3V on A0
 */

#include "power.h"
#include "debug.h"
#include <SoftwareSerial.h>

#define DEVICE_COUNT 4

typedef struct {
  char description[20];
  byte pin;
  float offVoltage;
  float onVoltage;
  uint16_t onDelay;
  uint16_t offDelay;
  unsigned long t1;
  unsigned long t2;
} device_item;

device_item pwr_device[DEVICE_COUNT] = {
  {"Relay 1", 6, 11.0, 12.0, 180, 10, 0, 0},
  {"Relay 2", 7, 11.5, 13.0, 600, 10, 0, 0},
  {"Relay 3", 8, 11.5, 13.0, 600, 10, 0, 0}, 
  {"Relay 4", 9, 11.5, 13.0, 600, 10, 0, 0}
};

enum baudrate_t {
  b300,
  b600,
  b1200,
  b2400,
  b4800,
  b9600,
  b19200,
  b38400,
  b56800
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
#define POWER_ON_PIN 3  // pin to enable power
#define LED_RED_PIN 4   // LED to indicate shutdown battery
#define LED_GREEN_PIN 5 // LED to indicate power ON
#define BUTTON_PIN 2    // Button, switched to GND, internal pullup required

#define ANALOG_REFERENCE_V 3.3     //Reference for Analog, Voltage at full scale
#define ANALOG_REFERENCE_RAW (float)1023 // 10bit AD converter

#define PRESCALE_A0 4.7273        //15.6V -> 3.3V
//#define PRESCALE_V2 4.0

#define LOOPTIME 1    // seconds

#define V_CUT 11.5      // [V] cut power after delay
#define V_CUT_NOW 10.0  // [V] cut power immediately
#define V_ON  13.0      // [V] switch back on after delay

#define V_CUT_DELAY 10    // seconds of voltage <= V_CUT
#define V_ON_DELAY 300    // seconds of voltage >= V_ON
#define MIN_OFF_TIME 20   //900  // seconds = 15 minutes 

static bool tty0_connected = false;
static bool tty1_connected = false;

unsigned long tty0_disconnect_time;
unsigned long tty1_disconnect_time;

#define TERMINAL_TIMEOUT 10000;

#define RX_BUF_SIZE 8
byte rxBuffer[RX_BUF_SIZE];

static float batteryV;
static float scaleFactor = ANALOG_REFERENCE_V / ANALOG_REFERENCE_RAW;

debugLevels currentDebugLevel = L_INFO;

void setup() {
  // turn LED off
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // configure pwr device outputs and switch them off 
  for (int i=0; i < DEVICE_COUNT; i++) {
    pinMode(pwr_device[i].pin, OUTPUT);
    devicePwr(i, PWR_OFF);
  }
  
  tty0.begin(TERMINAL_BAUD);
  tty0.println("Started tt0");
  tty1.begin(TERMINAL_BAUD);
  tty1.println("Started tt1");
}

void loop() {
  readVoltages();
  terminal_loop();
/*
  debugTTY.print("A0 = ");
  debugTTY.print(batteryV);
  debugTTY.println("V");
  delay(1000);
  */
/*
  cycleFlash = !cycleFlash;    // flash
  
  // Low voltage cutout logic
  if (batteryV < V_CUT) {
    if (batteryV < V_CUT_NOW)             // immediate cutout
      v_cut_timer = V_CUT_DELAY;    // eliminate and further delay 
    if (v_cut_timer >= V_CUT_DELAY) {
      system_power = false;
    } else {
      v_cut_timer += LOOPTIME;
    }    
  } else {
    v_cut_timer = 0;
  }

  // Power restore logic
  if ( (batteryV >= V_ON) && !system_power ) {
    if ( (v_on_timer > V_ON_DELAY) && (off_timer > MIN_OFF_TIME) ) {   // only if we had the minimum off time
      system_power = true;
      off_timer = 0;
    } else {      
      v_on_timer += LOOPTIME;
    }
  }

  // Button - overrides the power relay to off state
  if (!digitalRead(BUTTON_PIN))   // button pressed?
    system_power = false;

  // Drive indicators and power relay
  if (!system_power) {
    off_timer += LOOPTIME;              // timer to measure off time
    digitalWrite(POWER_ON_PIN, LOW);    // Power cutout relay
    digitalWrite(LED_GREEN_PIN, LOW);   // Set LED indicators
    digitalWrite(LED_RED_PIN, cycleFlash);  // flashing
  } else {
    digitalWrite(POWER_ON_PIN, HIGH);   // Power cutout relay
    digitalWrite(LED_GREEN_PIN, HIGH);  // Set LED indicators
    digitalWrite(LED_RED_PIN, LOW);
  }

  // reset the on time
  if (batteryV < V_ON) 
    v_on_timer = 0;

  // Debug output
  if (serialDebug) {
    Serial.print("v1: ");
    Serial.print(batteryV);
    Serial.print("  v_cut_timer: ");
    Serial.print(v_cut_timer);
    Serial.print("  v_on_timer: "); 
    Serial.print(v_on_timer);
    Serial.print("  off_timer: ");
    Serial.print(off_timer);
    Serial.print("  Button: ");
    Serial.print(digitalRead(BUTTON_PIN));
    if (system_power)
      Serial.println(" - Power is ON");
    else
      Serial.println(" - Power is OFF");
  }     

  // Power saving sleep cycle
  while ((int32_t) (millis() < sleep_until) ) {
      power_save();
      // check for console input
      if (Serial.available()) {
        if (Serial.read() > ' ')      // ignore control chars such as CR and LF
          serialDebug = !serialDebug; // toggle debug output
      }
    } 
  // calculate next sleep time
  sleep_until = millis() + (LOOPTIME * 1000);
  */
}

void readVoltages () {
  // Batter Voltage
  // read raw and convert to voltage at AI pin
  float pinV = (float)analogRead(A0) * scaleFactor;
  // Use pre-scale to calculate measured voltages (i.e. voltage divider)
  batteryV = pinV * PRESCALE_A0;
}

void devicePwr(byte device, bool newState) {
  digitalWrite(pwr_device[device].pin, newState);
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

void terminal_menu() {
  terminal_print("\nPower Management Menu\n");
  terminal_print("  v - battery voltage\n");
  terminal_print("  x - exit (close connection)\n");
  terminal_print("\nEnter command:\n");
}

void terminal_process_cmd(byte rx_char) {
  switch(rx_char) {
    case 'v':
      terminal_print("\n=================================\n");
      terminal_print("Battery Voltage: %d.%dV\n", (int) batteryV, (int) (batteryV *100) % 100);
      break;
    case 'x':
      terminal_disconnect();
    default:
      break;
  }
  terminal_menu();
}

int terminal0_rx() {
  int i;
  // clear RX buffer
  for (i=0; i<RX_BUF_SIZE; i++) {
    rxBuffer[i] = 0;
  }
  i = 0;
  unsigned long timeout = millis() + 150;
  while (millis() <= timeout) {
    if (tty0.available()) {
      rxBuffer[i++] = tty0.read();
    }
  }
  return i;
}

int terminal1_rx() {
  int i;
  // clear RX buffer
  for (i=0; i<RX_BUF_SIZE; i++) {
    rxBuffer[i] = 0;
  }
  i = 0;
  unsigned long timeout = millis() + 200;
  while (millis() <= timeout) {
    if (tty1.available()) {
      rxBuffer[i] = tty1.read();      
    }
  }
  return i;
}

void terminal_disconnect() {
  if (tty0_connected) {
    terminal_print("timeout ... tty0 disconnected\n");
    tty0_connected = false;
  }
  if (tty1_connected) {
    terminal_print("timeout ... tty1 disconnected\n");
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
  
  if (tty0.available()) {
    terminal0_rx();
    if (tty0_connected) {
      terminal_process_cmd(rxBuffer[0]);
    } else {  // not connected
      if ( (rxBuffer[0] == 0x0A) || (rxBuffer[0] == 0x0D) ){
        tty0_connected = true;
        terminal_print("\nVK2RAY\nconnected tty0");
        terminal_menu();
      }      
    }
    tty0_disconnect_time = millis() + TERMINAL_TIMEOUT;
  }
  
  if (tty1.available()) {
    terminal1_rx();
    if (tty1_connected) {
      terminal_process_cmd(rxBuffer[0]);
    } else {  // not connected
      if ( (rxBuffer[0] == 0x0A) || (rxBuffer[0] == 0x0D) ) {
        tty1_connected = true;
        terminal_print("\nVK2RAY\nconnected tty1\n");
        terminal_menu();
      }
    }
    tty1_disconnect_time = millis() + TERMINAL_TIMEOUT;
  }

  // check timeout
  if ( (tty0_connected) && (millis() > tty0_disconnect_time) ) {
    terminal_disconnect();
  }
  if ( (tty1_connected) && (millis() > tty1_disconnect_time) ) {
    terminal_disconnect();
  }
}

