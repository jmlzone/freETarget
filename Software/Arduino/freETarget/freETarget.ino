/*----------------------------------------------------------------
 * 
 * freETarget
 * 
 * Software to run the Air-Rifle / Small Bore Electronic Target
 * 
 *-------------------------------------------------------------*/
#include "io_includes.h"
#include "compute_hit.h"
#include "EEPROM.h"
#include "nonvol.h"
#include "mechanical.h"
#include "diag_tools.h"

history_t history;  

double       s_of_sound;                // Speed of sound
unsigned int shot = 0;                  // Shot counter
bool         face_strike = 0;           // Miss indicator
bool         is_trace = false;          // TRUE if trace is enabled

const char* names[] = { "ANON",    "BOSS",   "MINION",
                        "DOC",     "DOPEY",  "HAPPY",   "GRUMPY", "BASHFUL", "SNEEZEY", "SLEEPY",
                        "RUDOLF",  "DONNER", "BLITXEM", "DASHER", "PRANCER", "VIXEN",   "COMET", "CUPID", "DUNDER",
                        "ODIN",    "WODEN",   "THOR",   "BALDAR",
                        0};
                  
char* nesw = "NESW";                    // Cardinal Points
                 
/*----------------------------------------------------------------
 * 
 * function: setup()
 * 
 * brief: Initialize the board and prepare to run
 * 
 * return: None
 * 
 *--------------------------------------------------------------*/

void setup(void) 
{
  int i;
  
/*
 *  Setup the serial port
 */
  Serial.begin(115200);
#ifdef ESP32
  AUX_SERIAL.begin(115200,SERIAL_8N1,AUX_RX,AUX_TX); 
  DISPLAY_SERIAL.begin(115200,SERIAL_8N1,DISPLAY_RX,DISPLAY_TX); 
#else
  AUX_SERIAL.begin(115200); 
  DISPLAY_SERIAL.begin(115200); 
#endif
  PRINT("\r\nfreETarget "); PRINT(SOFTWARE_VERSION); PRINT("\r\n");
  
/*
 *  Set up the port pins
 */
  init_gpio();  
  is_trace = read_DIP() & (VERBOSE_TRACE);   
  init_sensors();
  init_analog_io();

/*
 * Run the power on self test
 */
  POST_1();                           // Cycle the LEDs
  while( POST_2() == false )          // If the timers fail, 
  {
    continue;                         // Don't continue to the application
  }
  POST_3();                           // Show the trip point
  
/*
 * Initialize variables
 */
  read_nonvol();
  set_LED_PWM(json_LED_PWM);
  
/*
 * Turn off the self test
 */ 
  switch (json_test)
  {
    case T_HELP:
    case T_PAPER:
    case T_PASS_THRU:
    case T_SET_TRIP:
    case T_XFR_LOOP:
      json_test = T_HELP;
      break;

    default:
      Serial.print("\r\nStarting Test: "); Serial.print(json_test);
      break;
  }

/*
 * Ready to go
 */
  show_echo(0);

  return;
}

/*----------------------------------------------------------------
 * 
 * function: loop()
 * 
 * brief: Main control loop
 * 
 * return: None
 * 
 *----------------------------------------------------------------
 */
#define SET_MODE      0         // Set the operating mode
#define ARM       (SET_MODE+1)  // State is ready to ARM
#define WAIT      (ARM+1)       // ARM the circuit
#define AQUIRE    (WAIT+1)      // Aquire the shot
#define REDUCE    (AQUIRE+1)    // Reduce the data
#define WASTE     (REDUCE+1)    // Wait for the shot to end
#define SEND_MISS (WASTE+1)     // Got a trigger, but was defective

unsigned int state = SET_MODE;
unsigned long now;              // Interval timer 
unsigned long power_save;       // Power save timer
unsigned int sensor_status;     // Record which sensors contain valid data
unsigned int location;          // Sensor location 
unsigned int i, j;              // Iteration Counter
int ch;
unsigned int shot_number;

void loop() 
{

/*
 * Take care of any commands coming through
 */
  if ( read_JSON() )
  {
    now = micros();               // Reset the power down timer if something comes in
    set_LED_PWM(json_LED_PWM);    // Put the LED back on if it was off
  }

/*
 * Cycle through the state machine
 */
  switch (state)
  {

/*
 *  Check for special operating modes
 */
  default:
  case SET_MODE:
    is_trace = read_DIP() & (VERBOSE_TRACE);     // Turn on tracing if the DIP switch is in place
    
    if ( read_DIP() & CALIBRATE )
    {
      json_test = T_SET_TRIP;
    }
    if ( json_test == 0 )       // No self test started
    {
      state = ARM;              // Carry on to the target
    }
    else
    {
      self_test(json_test);     // Run the self test
    }
    
    break;
/*
 * Arm the circuit
 */
  case ARM:
    arm_counters();

    enable_interrupt();               // Turn on the face strike interrupt
    face_strike = false;              // Reset the face strike count
    
    set_LED_PWM(json_LED_PWM);        // Turn the LEDs on

    sensor_status = is_running();
    power_save = micros();            // Start the power saver time
    
    if ( sensor_status == 0 )
    { 
      if ( is_trace )
      {
        Serial.print("\r\n\nWaiting...");
      }
      set_LED(LED_S, (bool) true);     // Show we are waiting
      set_LED(LED_X, (bool) false);    // No longer processing
      set_LED(LED_Y, (bool) false);   
      state = WAIT;             // Fall through to WAIT
    }
    else
    {
      set_LED(LED_X, (bool) true);    // show a fault  
      if ( sensor_status & TRIP_NORTH  )
      {
        Serial.print("\r\n{ \"Fault\": \"NORTH\" }");
        set_LED(LED_S, (bool) false);   // Fault code North
        set_LED(LED_Y, (bool) false);  
        delay(ONE_SECOND);
      }
      if ( sensor_status & TRIP_EAST  )
      {
        Serial.print("\r\n{ \"Fault\": \"EAST\" }");
        set_LED(LED_S, (bool) false);   // Fault code East
        set_LED(LED_Y, (bool) true);  
        delay(ONE_SECOND);
      }
      if ( sensor_status & TRIP_SOUTH )
      {
        Serial.print("\r\n{ \"Fault\": \"SOUTH\" }");
        set_LED(LED_S, (bool) true);  // Fault code South
        set_LED(LED_Y, (bool) true);  
         delay(ONE_SECOND);
      }
      if ( sensor_status & TRIP_WEST )
      {
        Serial.print("\r\n{ \"Fault\": \"WEST\" }");
        set_LED(LED_S, (bool) true);   // Fault code West
        set_LED(LED_Y, (bool) false);  
        delay(ONE_SECOND);
      }      
    }
    break;
    
/*
 * Wait for the shot
 */
  case WAIT:
    if ( (json_power_save != 0 ) 
        && (((micros()-power_save) / 1000000 / 60) >= json_power_save) )
    {
      set_LED_PWM(0);
    }

    sensor_status = is_running();

    if ( face_strike )                    // Something hit the front
    {
      state = SEND_MISS;                  // Show it's a miss
      break;
    }

    if ( sensor_status != 0 )             // Shot detected
    {
      now = micros();                     // Remember the starting time
      set_LED(LED_S, (bool) false);              // No longer waiting
      set_LED(LED_X, (bool) true);               // Aquiring
      set_LED(LED_Y, (bool) false);              // Aquiring
      state = AQUIRE;
    }
    break;

/*
 *  Aquire the shot              
 */  
  case AQUIRE:
    if ( (micros() - now) > SHOT_TIME )   // Enough time already
    { 
      stop_counters(); 
      state = REDUCE;                     // 3, 4 Have enough data to performe the calculations
    }
    break;

 
/*
 *  Reduce the data to a score
 */
  case REDUCE:   
    if ( is_trace )
    {
      Serial.print("\r\nTrigger: "); 
      
      if ( sensor_status & TRIP_NORTH ) Serial.print("N");
      else                              Serial.print("-");
      if ( sensor_status & TRIP_EAST )  Serial.print("E");
      else                              Serial.print("-");
      if ( sensor_status & TRIP_SOUTH ) Serial.print("S");
      else                              Serial.print("-");
      if ( sensor_status & TRIP_WEST )  Serial.print("W");
      else                              Serial.print("-");

      Serial.print("\r\nReducing...");
    }
    set_LED(LED_S, (bool) true);               // Light All
    set_LED(LED_X, (bool) true);               // 
    set_LED(LED_Y, (bool) true);               //
    location = compute_hit(sensor_status, &history, false);

    if ( (timer_value[N] == 0) || (timer_value[E] == 0) || (timer_value[S] == 0) || (timer_value[W] == 0) ) // If any one of the timers is 0, that's a miss
    {
      Serial.print("is_running"); Serial.print(is_running()); Serial.print(" ");
      
      state = SEND_MISS;
      delay(ONE_SECOND);
      break;
    }
    send_score(&history, shot_number, sensor_status);
    state = WASTE;
    shot_number++;                   
    break;
    

/*
 *  Wait here to make sure the RUN lines are no longer set
 */
  case WASTE:
    delay(1000);                              // Hang out for a second
    if ( (json_paper_time * PAPER_STEP) > (PAPER_LIMIT) )
    {
      json_paper_time = 0;                    // Check for an infinit loop
    }
    if ( json_paper_time != 0 )
    {
      drive_paper();
    }
    state = SET_MODE;
    break;

/*    
 * Show an error occured
 */
  case SEND_MISS:   
    state = SET_MODE;                         // Next time go to waste time 
    
    if ( is_trace )
    {
      Serial.print("\r\nFace Strike...\r\n");
    } 

    face_strike = false;

    for (i=0; i != 6; i++)
    {
      set_LED(LED_S, (bool) (i & 1));                  // Blink the LEDs to show an error
      set_LED(LED_X, (bool) (i & 1));                  // 
      set_LED(LED_Y, (bool) (i & 1));                  //
      delay(ONE_SECOND/4);
    }
    
    if ( json_send_miss == 0)
    {
      break;
    }
    
    set_LED_PWM(0);                           // Flash the LED
    delay(ONE_SECOND/4);
    set_LED_PWM(json_LED_PWM);
    send_miss(shot);
 
    break;
    }

/*
 * All done, exit for now
 */
  return;
}
