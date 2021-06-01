/*
 * Global functions
 */
#ifndef _GPIOIO_H_
#define _GPIPIO_H_

void init_gpio(void);                                     // Initialize the GPIO ports
void arm_counters(void);                                  // Make the board ready
unsigned int is_running(void);                            // Return a bit mask of running sensors 
void set_LED(int state_RDY, int state_X, int state_y);    // Manage the LEDs
unsigned int read_DIP(void);                              // Read the DIP switch register
unsigned int read_counter(unsigned int direction);
void stop_counters(void);                                 // Turn off the counter registers
void trip_counters(void);
bool read_in(unsigned int port);                          // Read the selected port
void read_timers(void);                                   // Read and return the counter registers
void drive_paper(void);                                   // Turn on the paper motor
void enable_interrupt(void);                              // Turn on the face strike interrupt
void disable_interrupt(void);                             // Turn off the face strike interrupt

/*
 *  Port Definitions
 */
#ifndef ESP32
#define D0          37                     //  Byte Port bit locations
#define D1          36                     //
#define D2          35                     //
#define D3          34                     //
#define D4          33                     //
#define D5          32                     //
#define D6          31                     //
#define D7          30                     //

#define NORTH_HI    50                    // Address port but locations
#define NORTH_LO    51
#define EAST_HI     48
#define EAST_LO     49
#define SOUTH_HI    43                    // Address port but locations
#define SOUTH_LO    47
#define WEST_HI     41
#define WEST_LO     42

#define RUN_NORTH   25
#define RUN_EAST    26
#define RUN_SOUTH   27
#define RUN_WEST    28

#define QUIET       29
#define RCLK        40
#define CLR_N       39
#define STOP_N      52       
#define CLOCK_START 53

#define DIP_A        9                    // 
#define DIP_B       10
#define DIP_C       11
#define DIP_D       12

#define CTS_U        7
#define RTS_U        6
#define LED_PWM      5          // PWM Port
#define LED_RDY      4
#define LED_X        3
#define LED_Y        2
#define LON          1          // Turn the LED on
#define LOF          0          // Turn the LED off
#define LXX         -1          // Leave the LED alone
#define L(A, B, C) ((A) == '*'), ((B) == '*'), ((C) == '*')

#define NORTH        0
#define EAST         1
#define SOUTH        2
#define WEST         3
#define TRIP_NORTH   0x01
#define TRIP_EAST    0x02
#define TRIP_SOUTH   0x04
#define TRIP_WEST    0x08

#define PAPER        18                    // Paper advance drive active low (TX1)
#define PAPER_ON      0
#define PAPER_OFF     1
#define PAPER_ON_300  1
#define PAPER_OFF_300 0

#define FACE_SENSOR  19

#define SPARE_1      22
#define SPARE_2      23

#define AUX_SERIAL         Serial3    // Auxilary Connector
#define DISPLAY_SERIAL     Serial2    // Serial port for slave display

#else
#include <Wire.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
void set_LEDCOLOR(unsigned int led, uint8_t r, uint8_t g, uint8_t b);   // Manage the LED as rgb
void set_LEDCOLOR(unsigned int led, uint32_t c);   // Manage the LED as rgb
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void spiSendFileToFPGA(char * fn);
void stop_enable_counters(void);              // this HAL should be added to the non esp32 version

// ESP32 definitions
//#define AUX_SERIAL_HW      HardwareSerial(2)    // Auxilary Connector tx-rx test ok
#define AUX_TX             16
#define AUX_RX             17
//#define DISPLAY_SERIAL_HW  HardwareSerial(1)    // Serial port for slave display tx-rx test ok
#define DISPLAY_TX         12
#define DISPLAY_RX         13
#define CDONE               2
#define CRESET_B            0
#define QUIETN             19
// 5 analogs checked, 0 - 4059
#define VREF                A0
#define NORTH_ANA           A3
#define EAST_ANA            A6
#define SOUTH_ANA           A7
#define WEST_ANA            A4
#define BD_REV              A13

#define PIXEL_PIN    22    // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT   3

#define CTS_U       14
#define RTS_U       27
#define LED_PWM     26   // PWM Port
#define LED_RDY      0   // pixel positions
#define LED_X        1   // pixel positions
#define LED_Y        2   // pixel positions
#define LON          1          // Turn the LED on
#define LOF          0          // Turn the LED off
#define LXX         -1          // Leave the LED alone
#define L(A, B, C) ((A) == '*'), ((B) == '*'), ((C) == '*')

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
#define NORTH_HIGH 3
#define EAST_LOW 4
#define EAST_HIGH 5
#define SOUTH_LOW 6
#define SOUTH_HIGH 7
#define WEST_LOW 8
#define WEST_HIGH 9
#define STATUS 0x0a
#define DIP 0x0b
#define SLOPECONTROL 0x0c
#define RECSEL 0x0d
#define RECCTL 0x0e
#define TRIGGERDEPTH 0x0f
#define CONV0 0x10
#define CONV1 0x11
#define CONV2 0x12
#define CONV3 0x13
#define CONV4 0x14
#define CONV5 0x15
#define TRACE01 0x30
#define TRACE23 0x31

// Bit definitions in control
#define CLEAR 1<<0
#define STOP  1<<1
#define QUIET 1<<2
#define START 1<<3

// Bit definitions in slope control
#define SLOPE_MODE 1<<7
#define SLOPE_NEG  1<<6

// bit definitions RECCTL
#define REC_ENABLE 1<<7

#endif
#define J10_1      VCC
#define J10_2       14                    // TX3
#define J10_3       15                    // RX3
#define J10_4       19                    // RX1
#define J10_5       18                    // TX1
#define J10_6      GND


#define EOF 0xFF


#endif
