/*-------------------------------------------------------
 * 
 * analog_io.ino
 * 
 * General purpose Analog driver
 * 
 * ----------------------------------------------------*/

#include "freETarget.h"
#include "analog_io.h"
#ifdef ESP32
  #include "gpioESP32.h"
#else
  #include "gpio.h"
#endif
#include "Wire.h"

/*----------------------------------------------------------------
 * 
 * void init_analog()
 * 
 * Initialize the analog I/O
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
 * void set_LED_PWM()
 * void blink_LED_PWM()
 * void set_LED_off()
 * 
 * Program the PWM value
 * 
 *----------------------------------------------------------------
 *
 * json_LED_PWM is a number 0-100 %  It must be scaled 0-255
 * 
 *--------------------------------------------------------------*/
static unsigned int led_pwm = 0;

void set_LED_PWM(int percent)
{
  if ( percent != led_pwm )
  {
    led_pwm = percent * 256 / 100;
    analogWrite(LED_PWM, led_pwm);  // Prime the PWM
  }
  
/*
 * All done, begin the program
 */
  return;
}


/*----------------------------------------------------------------
 * 
 * unsigned int read_feedback(void)
 * 
 * return the reference voltage
 * 
 *--------------------------------------------------------------*/
unsigned int read_reference(void)
{
  return analogRead(V_REFERENCE);
}

/*----------------------------------------------------------------
 * 
 * unsigned int revision(void)
 * 
 * Return the board revision
 * 
 *--------------------------------------------------------------
 *
 *  Read the analog value from the resistor divider, keep only
 *  the top 4 bits, and return the version number.
 *  
 *  The analog input is a number 0-1024 
 *--------------------------------------------------------------*/
//                                 0      1  2  3     4     5  6      7    8  9   A   B   C   D   E   F
#ifdef ESP32
  static unsigned int version[] = {REV_400, 1, 2, 3,  4, 5, 6,        7, 8, 9, 10, 11, 12, 13, 14, 15};
#else
  static unsigned int version[] = {REV_210, 1, 2, 3, REV_300, 5, 6, REV_220, 8, 9, 10, 11, 12, 13, 14, 15};
#endif
unsigned int revision(void)
{
  return version[analogRead(ANALOG_VERSION) * 16 / 1024];
}

/*----------------------------------------------------------------
 * 
 * void max_analog
 * 
 * Return the value of the largest analog input
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
 * void cal_analog
 * 
 * Use the Pots to calibrate the analog input threshold
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
  set_LED(LED_S, (steps & 1) == 0);
  set_LED(LED_X, (steps & 2) == 0);
  set_LED(LED_Y, (steps & 4) == 0);

  
/*
 * All done, return
 */
  return;

}

/*----------------------------------------------------------------
 * 
 * double temperature_C()
 * 
 * Read the temperature sensor and return temperature in degrees C
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
  Wire.requestFrom(TEMP_IC, 2);
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
