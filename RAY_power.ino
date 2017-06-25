/* RAY_power
 *  Battery monitor and power controller for VK2RAY repeater
 *  V1.0 Battery Voltage Monitor and Power Cut function when battery gets too low
 *       LCD display (if connected) will display
 * 
 * 
 * Serial debugging:
 * =================
 * 
 * The serial console output is switched off at the start.
 * console output can be enabled by sending one character (any character > 20h)
 * 
 */


#define USE_LCD

#ifdef USE_LCD
  #include <LCD.h>
  #include "LCD2004A.h"
  static LCD2004A lcd;
#endif

#include "power.h"


#define POWER_ON_PIN 3  // pin to enable power
#define LED_RED_PIN 4   // LED to indicate shutdown battery
#define LED_GREEN_PIN 5 // LED to indicate power ON
#define BUTTON_PIN 2    // Button, switched to GND, internal pullup required

#define ANALOG_REFERENCE_V 5.0     //Reference for Analog, Voltage at full scale
#define ANALOG_REFERENCE_RAW (float)1023 // 10bit AD converter

#define PRESCALE_V1 4.0
#define PRESCALE_V2 4.0

#define LOOPTIME 1    // seconds

#define V_CUT 11.5      // [V] cut power after delay
#define V_CUT_NOW 10.0  // [V] cut power immediately
#define V_ON  13.0      // [V] switch back on after delay

#define V_CUT_DELAY 10    // seconds of voltage <= V_CUT
#define V_ON_DELAY 300    // seconds of voltage >= V_ON
#define MIN_OFF_TIME 20 //900  // seconds = 15 minutes 

static uint32_t v_cut_timer = 0, off_timer = 0, v_on_timer = 0;
static bool system_power = true;
static bool serialDebug = false;
static bool cycleFlash;
static float v1, v2, v3, scaleFactor;
static int raw1, raw2, raw3;
static uint32_t sleep_until;

void setup() {
  delay(3000);  // over comes auto reset at startup
  // put your setup code here, to run once:
#ifdef USE_LCD
  lcd.setup();
#endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_ON_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  Serial.begin(9600);
  Serial.println("Started");
  sleep_until = millis() + (LOOPTIME * 1000);
}

void loop() {
  
  raw1 = analogRead(A0);
  raw2 = analogRead(A1);
  raw3 = analogRead(A2);

  scaleFactor = ANALOG_REFERENCE_V / ANALOG_REFERENCE_RAW;

  // read raw and convert to voltage at AI
  v1 = (float)raw1 * scaleFactor;
  v2 = (float)raw2 * scaleFactor;
  v3 = (float)raw3 * scaleFactor;
#ifdef USE_LCD
  lcd.printVoltages(2, v1, v2, v3);
#endif
  // Use pre-scale to calculate measured voltages
  v1 = v1 * PRESCALE_V1;    
  v2 = v2 * PRESCALE_V2;

#ifdef USE_LCD
  lcd.printVoltages(1, v1, v2, v3);
  lcd.printRaw(3, raw1, raw2, raw3);
#endif

  cycleFlash = !cycleFlash;    // flash
  
  // Low voltage cutout logic
  if (v1 < V_CUT) {
    if (v1 < V_CUT_NOW)             // immediate cutout
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
  if ( (v1 >= V_ON) && !system_power ) {
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
  if (v1 < V_ON) 
    v_on_timer = 0;

  // Debug output
  if (serialDebug) {
    Serial.print("v1: ");
    Serial.print(v1);
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
}
