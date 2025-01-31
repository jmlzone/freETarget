#ifndef _ANALOG_IO_H_
#define _ANALOG_IO_H_

/*
 * Global functions
 */

void init_analog_io(void);          // Setup the analog hardware
unsigned int read_reference(void);  // Read the feedback channel
void show_analog(int v);            // Display the analog values
void cal_analog(void);              // Calibrate the analog threshold
double temperature_C(void);         // Temperature in degrees C
unsigned int revision(void);        // Return the board revision
void set_LED_PWM(int percent);      // Ramp the PWM duty cycle
void set_LED_PWM_now(int percent);  // Set the PWM duty cycle

/*
 *  Port Definitions
 */
#ifdef ESP32
#define NORTH_ANA      A3       // North Analog Input
#define EAST_ANA       A6       // East Analog Input
#define SOUTH_ANA      A7       // South Analog Input
#define WEST_ANA       A4       // West Analog Input
#define SPARE_2A       A18      // Not Used
#define V_REFERENCE    A0       // Reference Input
#define ANALOG_VERSION A13      // Analog Version Input
#define LED_PWM        16       // PWM Port
#else
#define NORTH_ANA    1          // North Analog Input
#define EAST_ANA     2          // East Analog Input
#define SOUTH_ANA    3          // South Analog Input
#define WEST_ANA     4          // West Analog Input
#define SPARE_2A     7          // Not Used
#define V_REFERENCE  0          // Reference Input
#define ANALOG_VERSION 5        // Analog Version Input
#define LED_PWM      5          // PWM Port
#endif
#define MAX_ANALOG  0x3ff       // Largest analog input

#ifdef ESP32
  #define TO_VOLTS(x) ( ((double)(x) * 3.3) / 4096.0 )
#else
  #define TO_VOLTS(x) ( ((double)(x) * 5.0) / 1024.0 )
#endif
#define TEMP_IC   (0x9E >> 1)

#endif

