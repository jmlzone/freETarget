/*
 * ESP 32 version
 * Global functions
 */
#ifndef _GPIOIO_H_
#define _GPIPIO_H_

void init_gpio(void);                         // Initialize the GPIO ports
void arm_counters(void);                      // Make the board ready
unsigned int is_running(void);                // Return a bit mask of running sensors 
void set_LED(unsigned int led, bool state);   // Manage the LEDs
void set_LED(unsigned int led, uint8_t r, uint8_t g, uint8_t b);   // Manage the LED as rgb
void set_LED(unsigned int led, uint32_t c);   // Manage the LED as rgb

unsigned int read_DIP(void);                  // Read the DIP switch register
unsigned int read_counter(unsigned int direction);
void stop_counters(void);                     // Turn off the counter registers
void stop_enable_counters(void);              // this HAL shoudl be added to the non esp32 version
void read_timers(void);                       // Read and return the counter registers
void drive_paper(void);                       // Turn on the paper motor
void clock_start(void);
void enable_interrupt(void);                  // Turn on the face strike interrupt
void disable_interrupt(void);                 // Turn off the face strike interrupt

/*
 *  Port Definitions
 */

#define CTS_U       14
#define RTS_U       27
#define LED_PWM     26   // PWM Port
#define LED_S        0   // pixel positions
#define LED_X        1   // pixel positions
#define LED_Y        2   // pixel positions

#define TRIP_NORTH   0x01
#define TRIP_EAST    0x02
#define TRIP_SOUTH   0x04
#define TRIP_WEST    0x08

#define PAPER        21                    // Paper advance drive active low (TX1)
#define PAPER_ON      0
#define PAPER_OFF     1
#define PAPER_ON_300  1
#define PAPER_OFF_300 0

#define FACE_SENSOR   4

#define SPARE_1      32
#define SPARE_2      33

#define J10_1      VCC
#define J10_2       14                    // TX3
#define J10_3       15                    // RX3
#define J10_4       19                    // RX1
#define J10_5       18                    // TX1
#define J10_6      GND

#define EOF 0xFF

/* 
 I2C registers in the fpga
 0 RO 0x10 -- we only want to shoot 10's
 1 RW control 
 2 RO north low
 3 RO north high
 4 RO south low
 5 RO south hi
 6 RO east low
 7 RO east high
 8 RO west low
 9 RO west hi
 a run status west,east,south,north
*/
#define FPGA_ADDR 0x10
#define CONTROL 1
#define NORTH_LOW 2
#define STATUS 0x0a
#define DIP 0x0b
// Bit definitions in control
#define CLEAR 1<<0
#define STOP  1<<1
#define QUIET 1<<2
#define START 1<<3

#endif
