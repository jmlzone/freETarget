/*-------------------------------------------------------
 * 
 * gpio.ino
 * 
 * General purpose GPIO driver
 * 
 * ----------------------------------------------------*/
#include "io_includes.h"
#include <Adafruit_NeoPixel.h>
#define PIXEL_PIN    36    // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT   3
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

typedef struct {
  byte port;
  byte in_or_out;
  byte value;
} GPIO_INIT;

GPIO_INIT init_table[] = {
  {LED_PWM,     OUTPUT, 1},
  {RTS_U,       OUTPUT, 1},
  {CTS_U,       INPUT_PULLUP, 0},

  {FACE_SENSOR, INPUT_PULLUP, 0},
  
  {SPARE_1,     OUTPUT, 1},               // 18-Paper drive active low
  {SPARE_2,     INPUT_PULLUP, 0},
  
  {EOF, EOF, EOF} };

void face_ISR(void);

/*-----------------------------------------------------
 * 
 * function: gpio_init
 * 
 * brief: Initalize the various GPIO ports
 * 
 * return: None
 * 
 *-----------------------------------------------------
 *
 * The GPIO programming is held in a stgrucutre and 
 * copied out to the hardware on power up.
 *-----------------------------------------------------*/

void init_gpio(void)
{
  int i;

  i = 0;
  while (init_table[i].port != 0xff )
  {
    pinMode(init_table[i].port, init_table[i].in_or_out);
    if ( init_table[i].in_or_out == OUTPUT )
    {
      digitalWrite(init_table[i].port, init_table[i].value);
    }
    i++;
    strip.begin();
    strip.clear();
  }

/*
 * Special case of the witness paper
 */
  pinMode(PAPER, OUTPUT);
  if ( revision() < REV_300 )
  { 

    digitalWrite(PAPER, PAPER_OFF);
  }
  else 
  {
    digitalWrite(PAPER, PAPER_OFF_300);
  }
  
/*
 * All done, return
 */  
  return;
}


/*-----------------------------------------------------
   i2c wrapper functions
   (because the 'wire library' is a bit weak)
 *-----------------------------------------------------*/
 uint8_t i2c_buf[16]; // A static buffer used for I2C write and read
 void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val){
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(val);
  Wire.endTransmission();
}
 void i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t n){
   int i;
   Wire.beginTransmission(dev_addr);
   Wire.write(reg_addr);
   for(i=0; i<n;i++) {
     Wire.write(i2c_buf[i]);
   }
   Wire.endTransmission();
 }
 uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr) {
   Wire.beginTransmission(dev_addr);
   Wire.write(reg_addr);
   Wire.endTransmission();
   Wire.requestFrom(dev_addr,1);
   return(Wire.read());
 }
 void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t n) {
   Wire.beginTransmission(dev_addr);
   Wire.write(reg_addr);
   Wire.endTransmission();
   Wire.requestFrom(dev_addr,n);
   for(i=0; i<n;i++) {
     i2c_buf[i] = Wire.read();
   }
 }
 
/*-----------------------------------------------------
 * 
 * function: is_running
 * 
 * brief: Determine if the clocks are running
 * 
 * return: TRUE if the counters are running
 * 
 *-----------------------------------------------------
 *
 * Read in the running registers, and return a 1 for every
 * register that is running.
 * 
 *-----------------------------------------------------*/

unsigned int is_running (void)
{
  unsigned int i;
  i = i2c_read_reg(FPGA_ADDR,STATUS) & 0x0f;
 /*
  *  Return the running mask
  */
  return i;
  }

/*-----------------------------------------------------
 * 
 * function: arm_counters
 * 
 * brief: Strobe the control lines to start a new cycle
 * 
 * return: NONE
 * 
 *-----------------------------------------------------
 *
 * The counters are armed by
 * 
 *   Stopping the current cycle
 *   Taking the counters out of read
 *   Making sure the oscillator is running
 *   Clearing the counters
 *   Enabling the counters to run again
 * 
 *-----------------------------------------------------*/
void arm_counters(void)
  {
    i2c_write_reg(FPGA_ADDR,CONTROL,(CLEAR|STOP));
    //i2c_write_reg(FPGA_ADDR,CONTROL,0); non need these bits self clear
/*
 *   Attach the interrupt
 */
  if ( revision() >= REV_300 )
  {
    attachInterrupt(digitalPinToInterrupt(FACE_SENSOR),  face_ISR, CHANGE);
  }
  
  return;
  }

/*
 *  Stop the oscillator
 */
void stop_counters(void)
  {
    i2c_write_reg(FPGA_ADDR,CONTROL,(STOP|QUIET));
  return;
  }
void stop_enable_counters(void)
  {
    i2c_write_reg(FPGA_ADDR,CONTROL,STOP);
    //i2c_write_reg(FPGA_ADDR,CONTROL, 0); not needed this bit self clears
  return;
  }
void start_clock(void)
  {
    i2c_write_reg(FPGA_ADDR,CONTROL,START);
    i2c_write_reg(FPGA_ADDR,CONTROL, 0);
  return;
  }

/*-----------------------------------------------------
 * 
 * function: enable_interrupt
 * function: disable_interrupt
 * 
 * brief: Turn on the face interrupt
 * 
 * return: NONE
 * 
 *-----------------------------------------------------
 *
 * Enable interrupts works by attaching an interrupt
 * 
 *-----------------------------------------------------*/
void enable_interrupt(void)
{
  if ( revision() >= REV_300 )
  {
    attachInterrupt(digitalPinToInterrupt(FACE_SENSOR),  face_ISR, CHANGE);


  }
  



  return;
}

void disable_interrupt(void)
{
  if ( revision() >= REV_300 )
  {
    detachInterrupt(digitalPinToInterrupt(FACE_SENSOR));
  }

  return;
}

/*-----------------------------------------------------
 * 
 * function: read_DIP
 * 
 * brief: READ the jumper block setting
 * 
 * return: TRUE for every position with a jumper installed
 * 
 *-----------------------------------------------------
 *
 * The DIP register is read and formed into a word.
 * The word is complimented to return a 1 for every
 * jumper that is installed.
 * 
 * OR in the json_dip_switch to allow remote testing
 * OR in  0xF0 to allow for compile time testing
 *-----------------------------------------------------*/
unsigned int read_DIP(void)
{
  unsigned int return_value;
  
  return_value = i2c_read_reg(FPGA_ADDR,DIP) & 0x0F;  // DIP Switch
  return_value |= json_dip_switch;  // JSON message
  return_value |= 0xF0;             // COMPILE TIME

  return return_value;
}  

/*
 * Turn a LED on or off
 */
void set_LED(unsigned int led, bool state)
  {
  if ( state == 0 )
    strip.setPixelColor(led, 0,0,0);    // Off
  else
    strip.setPixelColor(led,127,0,0);    // ON = LOW (medium red)
  strip.show();
  return;  
  }
// overloads for setting led colors
void set_LED(unsigned int led,uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(led,r,g,b);
}
void set_LED(unsigned int led, uint32_t c) {
  strip.setPixelColor(led,c);
}
/*-----------------------------------------------------
 * 
 * function: read_timers
 * 
 * brief:   Read the timer registers
 * 
 * return:  All four timer registers read and stored
 * 
 *-----------------------------------------------------
 *
 * Force read each of the timers
 * 
 *-----------------------------------------------------*/
void read_timers(void)
{
  i2c_read_bytes(FPGA_ADDR,NORTH_LOW,8); 
  timer_value[N] = (i2c_buf[1] <<8) + i2c_buf[0];
  timer_value[E] = (i2c_buf[5] <<8) + i2c_buf[4];
  timer_value[S] = (i2c_buf[3] <<8) + i2c_buf[2];
  timer_value[W] = (i2c_buf[7] <<8) + i2c_buf[6];
  return;
}

/*-----------------------------------------------------
 * 
 * function: drive_paper
 * 
 * brief:    Turn on the witness paper motor for json_paper_time
 * 
 * return:  None
 * 
 *-----------------------------------------------------
 *
 * The function turns on the motor for the specified
 * time.
 * 
 * There is a hardare change between Version 2.2 which
 * used a transistor and 3.0 that uses a FET.
 * The driving circuit is reversed in the two boards.
 * 
 *-----------------------------------------------------*/
 void drive_paper(void)
 {
   if ( is_trace )
   {
     Serial.print("\r\nAdvancing paper...");
   }
    
   if ( revision() < REV_300 )
   {
     digitalWrite(PAPER, PAPER_ON);          // Advance the motor drive time
   }
   else
   {
     digitalWrite(PAPER, PAPER_ON_300);          // Advance the motor drive time
   }
   
   for (i=0; i != json_paper_time; i++ )
   {
     j = 7 * (1.0 - ((float)i / float(json_paper_time)));
     set_LED(LED_S, (bool) (j & 1));                // Show the paper advancing
     set_LED(LED_X, (bool) (j & 2));                // 
     set_LED(LED_Y, (bool) (j & 4));                // 
     delay(PAPER_STEP);                    // in 100ms increments
    }
    
   if ( revision() < REV_300 )
   {
     digitalWrite(PAPER, PAPER_OFF);          // Advance the motor drive time
   }
   else
   {
     digitalWrite(PAPER, PAPER_OFF_300);          // Advance the motor drive time
     digitalWrite(PAPER, PAPER_OFF_300);          // Advance the motor drive time
   }

 /*
  * All done, return
  */
  return;
 }
/*-----------------------------------------------------
 * 
 * function: face_ISR
 * 
 * brief:    Face Strike Interrupt Service Routint
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 *
 * Sensor #5 is attached to the digital input #19 and
 * is used to generate an interrrupt whenever a face
 * strike has been detected.
 * 
 * The ISR simply counts the number of cycles.  Anything
 * above 0 is an indication that sound was picked up
 * on the front face.
 * 
 *-----------------------------------------------------*/
 void face_ISR(void)
 {
  face_strike = true;      // Got a face strike

  if ( is_trace )
  {
    Serial.print("\r\nface_ISR()");
  }

  disable_interrupt();

  return;
 }
