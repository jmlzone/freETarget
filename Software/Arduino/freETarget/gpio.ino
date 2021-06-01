/*-------------------------------------------------------
 * 
 * gpio.ino
 * 
 * General purpose GPIO driver
 * 
 * ----------------------------------------------------*/
#include "io_includes.h"
#include "i2c_bb.h"
typedef struct {
  byte port;
  byte in_or_out;
  byte value;
} GPIO_INIT;
#ifndef ESP32
const GPIO_INIT init_table[] = {
  {D0,          INPUT_PULLUP, 0 },
  {D1,          INPUT_PULLUP, 0 },
  {D2,          INPUT_PULLUP, 0 },
  {D3,          INPUT_PULLUP, 0 },
  {D4,          INPUT_PULLUP, 0 },     
  {D5,          INPUT_PULLUP, 0 },
  {D6,          INPUT_PULLUP, 0 },

  {NORTH_HI,    OUTPUT, 1},
  {NORTH_LO,    OUTPUT, 1},
  {EAST_HI,     OUTPUT, 1},
  {EAST_LO,     OUTPUT, 1},
  {SOUTH_HI,    OUTPUT, 1},
  {SOUTH_LO,    OUTPUT, 1},
  {WEST_HI,     OUTPUT, 1},
  {WEST_LO,     OUTPUT, 1},      
        
  {RUN_NORTH,   INPUT_PULLUP, 0},
  {RUN_EAST,    INPUT_PULLUP, 0},
  {RUN_SOUTH,   INPUT_PULLUP, 0},
  {RUN_WEST,    INPUT_PULLUP, 0},     

  {QUIET,       OUTPUT, 1},
  {RCLK,        OUTPUT, 1},
  {CLR_N,       OUTPUT, 1},
  {STOP_N,      OUTPUT, 1},
  {CLOCK_START, OUTPUT, 0},
  
  {DIP_A,       INPUT_PULLUP, 0},
  {DIP_B,       INPUT_PULLUP, 0},
  {DIP_C,       INPUT_PULLUP, 0},
  {DIP_D,       INPUT_PULLUP, 0},  

  {LED_RDY,     OUTPUT, 1},
  {LED_X,       OUTPUT, 1},
  {LED_Y,       OUTPUT, 1},

  {LED_PWM,     OUTPUT, 0},
  {RTS_U,       OUTPUT, 1},
  {CTS_U,       INPUT_PULLUP, 0},

  {FACE_SENSOR, INPUT_PULLUP, 0},
  
  {SPARE_1,     OUTPUT, 1},               // 18-Paper drive active low
  {SPARE_2,     INPUT_PULLUP, 0},
  
  {EOF, EOF, EOF} };

#else

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
GPIO_INIT init_table[] = {
  {LED_PWM,     OUTPUT, 1},
  {RTS_U,       OUTPUT, 1},
  {CTS_U,       INPUT_PULLUP, 0},

  {FACE_SENSOR, INPUT_PULLUP, 0},
  
  {SPARE_1,     OUTPUT, 1},               // 18-Paper drive active low
  {SPARE_2,     INPUT_PULLUP, 0},
  {EOF, EOF, EOF} };
#endif
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
  }
#ifdef ESP32
  strip.begin();
  strip.clear();
#endif
  disable_interrupt();
  set_LED_PWM(0);             // Turn off the illumination for now
  
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
#ifndef ESP32
/*-----------------------------------------------------
 * 
 * function: read_port
 * 
 * brief: Read 8 bits from a port
 * 
 * return: Eight bits returned from the port
 * 
 *-----------------------------------------------------
 *
 * To make the byte I/O platform independent, this
 * function reads the bits in one-at-a-time and collates
 * them into a single byte for return
 * 
 *-----------------------------------------------------*/
int port_list[] = {D7, D6, D5, D4, D3, D2, D1, D0};

unsigned int read_port(void)
{
  int i;
  int return_value = 0;

/*
 *  Loop and read in all of the bits
 */
  for (i=0; i != 8; i++)
    {
    return_value <<= 1;
    return_value |= digitalRead(port_list[i]) & 1;
    }

 /*
  * Return the result 
  */
  return (return_value & 0x00ff);
}

/*-----------------------------------------------------
 * 
 * function: read_counter
 * 
 * brief: Read specified counter register
 * 
 * return: 16 bit counter register
 * 
 *-----------------------------------------------------
 *
 * Set the address bits and read in 16 bits
 * 
 *-----------------------------------------------------*/

int direction_register[] = {NORTH_HI, NORTH_LO, EAST_HI, EAST_LO, SOUTH_HI, SOUTH_LO, WEST_HI, WEST_LO};

unsigned int read_counter
  (
  unsigned int direction         // What direction are we reading?
  )
{
  int i;
  unsigned int return_value_LO, return_value_HI;     // 16 bit port value
  
/*
 *  Reset all of the address bits
 */
  for (i=0; i != 8; i++)
    {
    digitalWrite(direction_register[i], 1);
    }
  digitalWrite(RCLK,  1);   // Prepare to read
  
/*
 *  Set the direction line to low
 */
  digitalWrite(direction_register[direction * 2 + 0], 0);
  return_value_HI = read_port();
  digitalWrite(direction_register[direction * 2 + 0], 1);
  
  digitalWrite(direction_register[direction * 2 + 1], 0);
  return_value_LO = read_port();
  digitalWrite(direction_register[direction * 2 + 1], 1);

/*
 *  All done, return
 */
  return (return_value_HI << 8) + return_value_LO;
}
#else

/*-----------------------------------------------------
   i2c wrapper functions
   (because the 'wire library' is a bit weak)
   ESP32 wire library has a 128 byte max buffer
 *-----------------------------------------------------*/
 uint8_t i2c_buf[16]; // A static buffer used for I2C write and read
 uint8_t cnt, avail, err, retries;
 #ifdef I2C_BB
 void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val){
 i2c_buf[0] = reg_addr;
 i2c_buf[1] = val;
 i2cbb_write (dev_addr, 2, i2c_buf);
}
 uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr) {
 i2cbb_read(dev_addr, 1, reg_addr, i2c_buf);
 return(i2c_buf[0]);
}
 void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t n) {
 i2cbb_read(dev_addr, n, reg_addr, i2c_buf);
}
 #else
 #define MAX_I2C_RETRIES 15
 void i2c_err_chk(uint8_t n, uint8_t exp) {
   avail = Wire.available();
   err = Wire.lastError();
   if((exp != avail) || (n!=avail)) {
    Serial.print("I2C ERR n != Exp != Avail expected ");
    Serial.print(n);
    Serial.print(", "),
    Serial.print(exp);
    Serial.print(" but have ");
    Serial.print(avail);
    Serial.print(" available\r\n"); 
   }
   if(err) {
    Serial.print("I2C ERR code: ");
    Serial.print(Wire.getErrorText(err));
    Serial.print("\r\n");
   }
 }
 void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val){
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(val);
  Wire.endTransmission();
  Wire.flush();
}
 uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr) {
   Wire.beginTransmission(dev_addr);
   Wire.write(reg_addr);
   Wire.endTransmission();
   cnt = Wire.requestFrom(dev_addr,1,true);
   i2c_err_chk(1,cnt);
   i2c_buf[0] = Wire.read();
   Wire.flush();
   return(i2c_buf[0]);
 }
 void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t n) {
   cnt = 0;
   retries = 0;
   while((cnt !=n) && (retries<MAX_I2C_RETRIES)) {
    Wire.flush();
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    cnt = Wire.requestFrom(dev_addr,n,true);
    retries++;
   }
   i2c_err_chk(n,cnt);
   for(i=0; i<n;i++) {
     i2c_buf[i] = Wire.read();
   }
   Wire.flush();
 }
#endif
void stop_enable_counters(void)
  {
    i2c_write_reg(FPGA_ADDR,CONTROL,STOP);
    //i2c_write_reg(FPGA_ADDR,CONTROL, 0); not needed this bit self clears
  return;
  }

#endif
/*-----------------------------------------------------
 * 
 * function: is_running
 * 
 * brief: Determine if the clocks are running
 * 
 * return: TRUE if any of the counters are running
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
#ifndef ESP32
  i = 0;
  
  if ( digitalRead(RUN_NORTH) == 1 )
    i += 1;
    
  if ( digitalRead(RUN_EAST) == 1 )
    i += 2;

  if ( digitalRead(RUN_SOUTH) == 1 )
    i += 4;
    
  if ( digitalRead(RUN_WEST) == 1 )
    i += 8;  
#else
  i = i2c_read_reg(FPGA_ADDR,STATUS) & 0x0f;
#endif
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
#ifndef ESP32
  digitalWrite(CLOCK_START, 0);   // Make sure Clock start is OFF
  digitalWrite(STOP_N, 0);        // Cycle the stop just to make sure
  digitalWrite(RCLK,   0);        // Set READ CLOCK to LOW
  digitalWrite(QUIET,  1);        // Arm the counter
  digitalWrite(CLR_N,  0);        // Reset the counters 
  digitalWrite(CLR_N,  1);        // Remove the counter reset 
  digitalWrite(STOP_N, 1);        // Let the counters run
#else
  //delay(2); // allow the clock time to start
  i2c_write_reg(FPGA_ADDR,CONTROL,(CLEAR|STOP));
#endif
  return;
  }



/*
 *  Stop the oscillator
 */
void stop_counters(void)
  {
#ifndef ESP32
  digitalWrite(STOP_N,0);   // Stop the counters
  digitalWrite(QUIET, 0);   // Kill the oscillator 
#else
  i2c_write_reg(FPGA_ADDR,CONTROL,(STOP|QUIET));
  //delay(2); // allow the clock time to propigate
  //delay(2); // allow the clock time to stop
#endif
  return;
  }

/*
 *  Trip the counters for a self test
 */
void trip_counters(void)
{
#ifndef ESP32
  digitalWrite(CLOCK_START, 0);
  digitalWrite(CLOCK_START, 1);     // Trigger the clocks from the D input of the FF
  digitalWrite(CLOCK_START, 0);
#else
  i2c_write_reg(FPGA_ADDR,CONTROL,START); // self clears
#endif
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
#ifndef ESP32
  if ( revision() < REV_300 )          // The silkscreen was reversed in Version 3.0  oops
  {
    return_value =  (~((digitalRead(DIP_A) << 0) + (digitalRead(DIP_B) << 1) + (digitalRead(DIP_C) << 2) + (digitalRead(DIP_D) << 3))) & 0x0F;  // DIP Switch
  }
  else
  {
    return_value =  (~((digitalRead(DIP_D) << 0) + (digitalRead(DIP_C) << 1) + (digitalRead(DIP_B) << 2) + (digitalRead(DIP_A) << 3))) & 0x0F;  // DIP Switch
  }
#else
  return_value = i2c_read_reg(FPGA_ADDR,DIP) & 0x0F;  // DIP Switch
#endif
  return_value |= json_dip_switch;  // JSON message
  return_value |= 0xF0;             // COMPILE TIME

  return return_value;
}  

/*-----------------------------------------------------
 * 
 * function: set_LED
 * 
 * brief:    Set the state of all the LEDs
 * 
 * return:   None
 * 
 *-----------------------------------------------------
 *
 * The state of the LEDs can be turned on or off 
 * 
 * -1 Leave alone
 *  0 Turn LED off
 *  1 Turn LED on
 * 
 *-----------------------------------------------------*/
void set_LED
  (
    int state_RDY,        // State of the Rdy LED
    int state_X,          // State of the X LED
    int state_Y           // State of the Y LED
    )
{
#ifndef ESP32
  if ( state_RDY >= 0 )
  {
    digitalWrite(LED_RDY, state_RDY == 0 );
  }
  
  if ( state_X >= 0 )
  {
    digitalWrite(LED_X, state_X == 0);
  }

  if ( state_Y >= 0 )
  {
    digitalWrite(LED_Y, state_Y == 0);
  }
#else
  if ( state_RDY >= 0 )
  {
    strip.setPixelColor(LED_RDY, (state_RDY?127:0),0,0); // half red intensity
  }
  
  if ( state_X >= 0 )
  {
    strip.setPixelColor(LED_X, (state_X?127:0),0,0); // half red intensity
  }

  if ( state_Y >= 0 )
  {
    strip.setPixelColor(LED_Y, (state_Y?127:0),0,0); // half red intensity
  }
 strip.show();
#endif
    
  return;  
  }
#ifdef ESP32
void set_LEDCOLOR(unsigned int led,uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(led,r,g,b);
}
void set_LEDCOLOR(unsigned int led, uint32_t c) {
  strip.setPixelColor(led,c);
}
#endif

/* 
 *  HAL Discrete IN
 */
bool read_in(unsigned int port)
{
  return digitalRead(port);
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
#ifndef ESP32 
  timer_value[N] = read_counter(N);  
  timer_value[E] = read_counter(E);
  timer_value[S] = read_counter(S);
  timer_value[W] = read_counter(W);
#else
  i2c_read_bytes(FPGA_ADDR,NORTH_LOW,8); 
  timer_value[N] = (i2c_buf[1] <<8) + i2c_buf[0];
  timer_value[E] = (i2c_buf[5] <<8) + i2c_buf[4];
  timer_value[S] = (i2c_buf[3] <<8) + i2c_buf[2];
  timer_value[W] = (i2c_buf[7] <<8) + i2c_buf[6];
#endif
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
     digitalWrite(PAPER, PAPER_ON);               // Advance the motor drive time
   }
   else
   {
     digitalWrite(PAPER, PAPER_ON_300);          // Advance the motor drive time
   }
   
   for (i=0; i != json_paper_time; i++ )
   {
     j = 7 * (1.0 - ((float)i / float(json_paper_time)));
     set_LED(j & 4, j & 2, j & 1);                // Show the count going downb
     delay(PAPER_STEP);                           // in 100ms increments
    }
    
   if ( revision() < REV_300 )
   {
     digitalWrite(PAPER, PAPER_OFF);              // Advance the motor drive time
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

 /*
 * Common function to indicate a fault // Cycle LEDs 5x
 */
void blink_fault
  (                                        
  unsigned int fault_code                 // Fault code to blink
  )
{
  unsigned int i;

  for (i=0; i != 3; i++)
  {
    set_LED(fault_code & 4, fault_code & 2, fault_code & 1);  // Blink the LEDs to show an error
    delay(ONE_SECOND/4);
    fault_code = ~fault_code;
    set_LED(fault_code & 4, fault_code & 2, fault_code & 1);                    // Blink the LEDs to show an error
    delay(ONE_SECOND/4);
    fault_code = ~fault_code;
  }

 /*
  * Finished
  */
  return;
}
#ifdef ESP32
//ESP32 / fpga only functions
void spiSendFileToFPGA(char * fn) {
  int i, sz, status;
  uint8_t data;
  // file should be 104090 long docs say 104161 bytes.
  Serial.print("opening file ");
  Serial.print(fn);
  // open file to send
  File f = SPIFFS.open(fn, "r");
  if(f!=NULL) {
    Serial.print(" open sucessful\n");
  } else {
    Serial.print(" open failed\n");
    
  }
  if(f!=NULL) {
     sz = f.size();
     vspi = new SPIClass(VSPI);
     //initialise vspi with default pins
     //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    vspi->begin();    // put FPGA in mode to recieve file
    digitalWrite(SS, LOW); //pull SS low to prep other end for transfer as slave
    digitalWrite(CRESET_B, LOW); //pull CRESET_B low to prep other end for config
    vspi->beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE3)); // clock hi
    delayMicroseconds(200); // wait atleast 200 us;
    digitalWrite(CRESET_B, HIGH); //pull CRESET_B low to prep other end for config
    delayMicroseconds(1200); // wait atleast 1200 us;
    digitalWrite(SS, HIGH); //pull ss high siglan start
    data = 0xa5;
    vspi->transfer(data); // send 8 dummy clocks
    digitalWrite(SS, LOW); //pull SS low 
    // for testing only
    //data = 0x5a;
    //vspi->transfer(data); // send 8 dummy clocks
    
    // send file
    for(i=0; i<sz; i++) {
      status = f.read((uint8_t *) &data,1);
      //data = fgetc(f);
      //if(i<16) {Serial.print(data,HEX); Serial.print(" ");}
      //if(i==16) {Serial.print("...\n");}
      vspi->transfer(data);
    }
    digitalWrite(SS, HIGH); //pull ss high to signify end of data transfer
    f.close();
    // send 100 (104) clocks
    for(i=0; i<13; i++) {
      data = 0;
      vspi->transfer(data);
    }
    // check that the FPGA is happy
    if(digitalRead(CDONE)) {
      Serial.print("\nProgramming success\n");
    } else {
      Serial.print("\nProgramming fail\n");
    }
    //digitalWrite(SS, HIGH); //pull ss high to signify end of data transfer
    f.close();
    // send 49 (56) clocks
    for(i=0; i<7; i++) {
      data = 0;
      vspi->transfer(data);
    }
    vspi->endTransaction();
    vspi->end();
    delete vspi;
  }
}

  
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
   Serial.printf("Listing directory: %s\r\n", dirname);

   File root = fs.open(dirname);
   if(!root){
      Serial.println("− failed to open directory");
      return;
   }
   if(!root.isDirectory()){
      Serial.println(" − not a directory");
      return;
   }

   File file = root.openNextFile();
   while(file){
      if(file.isDirectory()){
         Serial.print("  DIR : ");
         Serial.println(file.name());
         if(levels){
            listDir(fs, file.name(), levels -1);
         }
      } else {
         Serial.print("  FILE: ");
         Serial.print(file.name());
         Serial.print("\tSIZE: ");
         Serial.println(file.size());
      }
      file = root.openNextFile();
   }
}
unsigned int read_counter(unsigned int direction){         // What direction are we reading?
  static uint8_t REGADDR[4] = {NORTH_LOW,EAST_LOW,SOUTH_LOW,WEST_LOW};
  i2c_read_bytes(FPGA_ADDR,REGADDR[direction],2); 
  return((i2c_buf[1] << 8) + i2c_buf[0]);
}

#endif
