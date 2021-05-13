/*-------------------------------------------------------
 * 
 * analog_io.ino
 * 
 * General purpose Analog driver
 * 
 * ----------------------------------------------------*/
#include "io_includes.h"
#include "Wire.h"

/*----------------------------------------------------------------
 * 
 * function: init_analog()
 * 
 * brief: Initialize the analog I/O
 * 
 * return: None
 * 
 *--------------------------------------------------------------*/
void init_analog_io(void)
{
  pinMode(LED_PWM, OUTPUT);
  Wire.begin();
  
/*
 * All done, begin the program
 */
  return;
}

/*----------------------------------------------------------------
 * 
 * function: set_LED_PWM()
 * function: blink_LED_PWM()
 * function: set_LED_off()
 * 
 * brief: Program the PWM value
 * 
 * return: None
 * 
 *----------------------------------------------------------------
 *
 * json_LED_PWM is a number 0-100 %  It must be scaled 0-255
 * 
 * The function ramps the level between the current and desired
 * 
 *--------------------------------------------------------------*/
static unsigned int old_LED_percent = 0;

void set_LED_PWM
  (
  int new_LED_percent                            // Desired LED level (0-100%)
  )
{
  while ( new_LED_percent != old_LED_percent )  // Change in the brightness level?
  {
    analogWrite(LED_PWM, old_LED_percent * 256 / 100);  // Write the value out
    
    if ( new_LED_percent < old_LED_percent )
    {
      old_LED_percent--;                        // Ramp the value down
    }
    else
    {
      old_LED_percent++;                        // Ramp the value up
    }

    delay(ONE_SECOND/50);                       // Worst case, take 2 seconds to get there
  }
  
/*
 * All done, begin the program
 */
  if ( new_LED_percent == 0 )
  {
    digitalWrite(LED_PWM, 0);
  }
  return;
}


/*----------------------------------------------------------------
 * 
 * function: read_feedback(void)
 * 
 * brief: return the reference voltage
 * 
 * return: ADC value of the reference voltage
 * 
 *--------------------------------------------------------------*/
unsigned int read_reference(void)
{
  return analogRead(V_REFERENCE);
}

/*----------------------------------------------------------------
 * 
 * function: revision(void)
 * 
 * brief: Return the board revision
 * 
 * return: Board revision level
 * 
 *--------------------------------------------------------------
 *
 *  Read the analog value from the resistor divider, keep only
 *  the top 4 bits, and return the version number.
 *  
 *  The analog input is a number 0-1024(arduino) 0-4095(ESP32)
 *  which is banded and used to look up a table of revision numbers.
 *  
 *  To accomodate unknown hardware builds, if the revision is
 *  undefined (< 100) then the last 'good' revision is returned
 *  
 *--------------------------------------------------------------*/
//                                 0      1  2  3     4     5  6      7    8  9   A   B   C   D   E   F
#ifdef ESP32
  static unsigned int version[] = {REV_400, 1, 2, 3,  4, 5, 6,        7, 8, 9, 10, 11, 12, 13, 14, 15};
#else
  static unsigned int version[] = {REV_210, 1, 2, 3, REV_300, 5, 6, REV_220, 8, 9, 10, 11, 12, 13, 14, 15};
#endif
unsigned int revision(void)
{
  unsigned int revision;

/* 
 *  Read the resistors and determine the board revision
 */
  revision =  version[analogRead(ANALOG_VERSION) * 16 / 1024];

/*
 * Fake the revision if it is undefined
 */
  if ( revision <= REV_100 )
  {
    revision = REV_300;
  }

/*
 * Nothing more to do, return the board revision
 */
  return revision;
}

/*----------------------------------------------------------------
 * 
 * function: max_analog
 * 
 * brief: Return the value of the largest analog input
 * 
 * return: Largest analog voltage from the sensor channels
 * 
 *--------------------------------------------------------------*/
uint16_t max_analog(void)
{
  uint16_t return_value;

  return_value = analogRead(NORTH_ANA);

  if ( analogRead(EAST_ANA) > return_value ) 
    return_value = analogRead(EAST_ANA);

  if ( analogRead(SOUTH_ANA) > return_value ) 
    return_value = analogRead(SOUTH_ANA);

  if ( analogRead(WEST_ANA) > return_value ) 
    return_value = analogRead(WEST_ANA);

  return return_value;
  
}
/*----------------------------------------------------------------
 * 
 * function: cal_analog
 * 
 * brief: Use the Pots to calibrate the analog input threshold
 * 
 * return: None
 * 
 *---------------------------------------------------------------
 *
 * The analog inputs are sampled, and the LED display is used to
 * show trip voltage in 250 mv increments.
 * 
 *--------------------------------------------------------------*/
#define THRESHOLD 10            // Accept an input +/- 20 steps

void cal_analog(void)
{
  uint16_t reference;
  uint16_t steps;
  
  show_analog(0);
  reference = analogRead(V_REFERENCE);

  steps = 1000.0 * TO_VOLTS(reference - max_analog()) / 250.0;
  
  Serial.println(); Serial.print(steps); Serial.print("  ");

  if ( steps > 7 )
    {
     steps = 0;
    }
  set_LED(steps & 4, steps & 2, steps & 1);

/*
 * All done, return
 */
  return;

}

/*----------------------------------------------------------------
 * 
 * funciton: temperature_C()
 * 
 * brief: Read the temperature sensor and return temperature in degrees C
 * 
 *----------------------------------------------------------------
 *
 * See TI Documentation for LM75
 *
 *--------------------------------------------------------------*/
 #define RTD_SCALE      (0.5)   // 1/2C / LSB

double temperature_C(void)
{
  double return_value;
  int raw;                // Allow for negative temperatures
  int i;

  raw = 0xffff;
  
/*
 *  Point to the temperature register
 */
  Wire.beginTransmission(TEMP_IC);
  Wire.write(0),
  Wire.endTransmission();

/*
 * Read in the temperature register
 */
  Wire.requestFrom((uint8_t) TEMP_IC, (uint8_t) 2);
  raw = Wire.read();
  raw <<= 8;
  raw += Wire.read();
  raw >>= 7;
  
  if ( raw & 0x0100 )
    {
    raw |= 0xFF00;      // Sign extend
    }

/*
 *  Return the temperature in C
 */
  return_value =  (double)(raw) * RTD_SCALE ;
  
#if (SAMPLE_CALCULATIONS )
  return_value = 23.0;
#endif
    
  return return_value;

}
