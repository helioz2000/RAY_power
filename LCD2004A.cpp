/* Demonstration sketch for PCF8574T I2C LCD Backpack 
Uses library from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads GNU General Public License, version 3 (GPL-3.0) */

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#include "LCD2004A.h"

// definitions for our I2C LCD
#define LCD_TXT_TITLE "- Battery  Monitor -"
#define LCD_TXT_CLR_LINE "                    "
#define LCD_CHAR_PER_LINE 20
#define LCD_LINES 4
#define LCD_I2C_ADDRESS 0x3F  // Use I2Cscanner to find the correct address for your module
#define LCD_BACKLIGHT_PIN 3
// Need to redefine pins for this LCD
#define LCD_EN_PIN 2
#define LCD_RW_PIN 1
#define LCD_RS_PIN 0
#define LCD_D4_PIN 4
#define LCD_D5_PIN 5
#define LCD_D6_PIN 6
#define LCD_D7_PIN 7

// module variables
// define the LCD display
static LiquidCrystal_I2C  lcd(LCD_I2C_ADDRESS, LCD_EN_PIN, LCD_RW_PIN, LCD_RS_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN); 

LCD2004A::LCD2004A() {
}

boolean LCD2004A::setup()
{
  // activate LCD module
  lcd.begin (LCD_CHAR_PER_LINE, LCD_LINES);     // LCD module size
  lcd.setBacklightPin(LCD_BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // set cursor to 0,0
  lcd.print(LCD_TXT_TITLE); 
}

void LCD2004A::printVoltages(int line, float v1, float v2, float v3) {
  char v1strbuf[6], v2strbuf[6], v3strbuf[6], strbuf[30];
  //lcd.setCursor (0,line);        // clear 2nd line
  //lcd.print(LCD_TXT_CLR_LINE);
  lcd.setCursor (0,line);        // go to start of 2nd line
  dtostrf(v1, 5, 2, v1strbuf);
  dtostrf(v2, 5, 2, v2strbuf);
  dtostrf(v3, 5, 2, v3strbuf);
  sprintf(strbuf, "%sV %sV %sV", v1strbuf, v2strbuf, v3strbuf);
  lcd.print(strbuf);
}

void LCD2004A::printRaw(int line, int v1, int v2, int v3) {
  char strbuf[30];
  //lcd.setCursor (0,line);        // clear line 3
  //lcd.print(LCD_TXT_CLR_LINE);
  lcd.setCursor (0,line);        // go to start of line 3
  sprintf(strbuf, " %4d   %4d   %4d", v1, v2, v3);
  lcd.print(strbuf);
}

void LCD2004A::test()
{
//  lcd.home (); // set cursor to 0,0
//  lcd.print("- VK3ERW  LCD demo -"); 
  lcd.setCursor (0,1);        // go to start of 2nd line
  lcd.print(millis());
  lcd.print(" millis");
  
  lcd.setCursor (0,2);        
  lcd.print(".... Third Line ....");
  lcd.setCursor (0,4);     
  lcd.print("01234567890123456789");

//  delay(1000);
//  lcd.setBacklight(LOW);      // Backlight off
//  delay(250);
//  lcd.setBacklight(HIGH);     // Backlight on
//  delay(1000);
}

