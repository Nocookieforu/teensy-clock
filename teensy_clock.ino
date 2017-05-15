
/** Teensy read of Adafruit GA1A12S202 Log-scale Analog Light Sensor.
 *  Page for Adafruit Light Sensor:
 *  https://www.adafruit.com/product/1384
 *
 * Currently using 47 kOhm resistor to sense current. Minimum output
 * current is 5 uA, and maximum output current is approx 45 uA.
 * At 5 uA of current, V = (5 uA) * (47 kOhm) = 0.23 V
 * At 45 uA of current, V = (45 uA) * (47 kOhm) = 2.12 V
 */

#include <stddef.h>
#include <stdint.h>
#include "LiquidCrystal.h"

// Analog sensor hooked to Teensy pin 20 (A6)
#define LIGHT_ANALOG_PIN      (A6)

// Storage and index for current measurements
#define CURRENT_NUM_READS     (16)
int     current_reads[CURRENT_NUM_READS] = {0};
size_t  current_ind = 0;

// LiquidCrystal Init for Adafruit character LCDs
// RW = not used, RS = 14, EN = 15
// DB7 = 16, DB6 = 17, DB5 = 18, DB4 = 19
LiquidCrystal lcd(14, 15, 19, 18, 17, 16);

#define REDLITE   23
#define GREENLITE 22
#define BLUELITE  21

// Functions defined below
void lcd_setup(void);
void lcd_write_voltage(int voltage);
void setBacklight(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
                      
void setup() {
  // Clear variables
  current_ind = 0;
  size_t i;
  for (i = 0; i < CURRENT_NUM_READS; i++)
  {
    current_reads[i] = 0;
  }
  Serial.begin(9600);
  
  // Turn on backlight
  pinMode(REDLITE,   OUTPUT);
  pinMode(GREENLITE, OUTPUT);
  pinMode(BLUELITE,  OUTPUT);
  setBacklight(0, 100, 0, 100);
    
  // Set up LCD
  lcd.begin(16, 2);
  lcd.write("Starting light sensor");
  delay(1000);
  lcd_setup();
}

void loop() {
  long in_avg = 0;
  // Read light sensor
  for (current_ind = 0; current_ind < CURRENT_NUM_READS; current_ind++)
  {
    current_reads[current_ind] = analogRead(LIGHT_ANALOG_PIN);
    in_avg += current_reads[current_ind];
    delay(1);
  }
  in_avg = in_avg / 16;
  
  // TODO: Translate to lumens
  setBacklight(0, (in_avg/4) & 0xFF, 0, 100);
  
  // Display on LCD
  lcd_write_voltage(in_avg);
  
  delay(200);
}

void lcd_setup(void)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write("Light Voltage:");
}

// Lookup table for Hexadecimal character mapping
char hex_chars[16] = {'0', '1', '2', '3',
                      '4', '5', '6', '7',
                      '8', '9', 'A', 'B',
                      'C', 'D', 'E', 'F'};

void lcd_write_voltage(int voltage)
{
  // Set LCD cursor to bottom row
  lcd.setCursor(0, 1);
  
  // value_str is whole width of LCD (16 chars)
  char voltage_str[17] = "0x0000          ";

  // Set four hex characters in string
  int i;
  for (i = 0; i < 4; i++)
  {
    voltage_str[5 - i] = hex_chars[(voltage >> (i*4)) & 0x0F];
  }
  
  lcd.write(voltage_str);
}

/** Set LCD Backlight for Adafruit RGB LCD
 *  From:
 *  https://learn.adafruit.com/character-lcds/rgb-backlit-lcds
 */
void setBacklight(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
  // normalize the red LED - its brighter than the rest!
  r = map(r, 0, 255, 0, 100);
  g = map(g, 0, 255, 0, 150);
 
  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);
 
  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);
  //Serial.print("R = "); Serial.print(r, DEC);
  //Serial.print(" G = "); Serial.print(g, DEC);
  //Serial.print(" B = "); Serial.println(b, DEC);
  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
}
