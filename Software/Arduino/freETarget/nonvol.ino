/*-------------------------------------------------------
 * 
 * nonvol.ino
 * 
 * Nonvol storage managment
 * 
 * ----------------------------------------------------*/
#include "nonvol.h"
#include "freETarget.h"
#include "json.h"
#ifdef ESP32
  #include <FS.h>
  #include <SPIFFS.h>
  nvmdata_t nvmdata;
#endif
/*----------------------------------------------------------------
 * 
 * function: init_nonvol
 * 
 * brief: Initialize the NONVOL back to factory settings
 * 
 * return: None
 *---------------------------------------------------------------
 *
 * The variable NONVOL_INIT is corrupted and the NONVOL read back
 * in and initialized.
 * 
 *------------------------------------------------------------*/
void init_nonvol(int v)
{
  unsigned int nonvol_init;

  nonvol_init = 0;                        // Corrupt the init location
#ifndef ESP32
  EEPROM.put(NONVOL_INIT, nonvol_init);
#else
  nvmdata.init = nonvol_init;
#endif
  Serial.print("\r\nReset to factory defaults\r\n");
  read_nonvol();                          // Force in new values
  show_echo(0);                           // Display these settings
  set_trip_point(0);                      // And stay forever in the trip mode
  
/*
 * All done, return
 */
  return;
}

/*----------------------------------------------------------------
 * 
 * funciton: read_nonvol
 * 
 * brief: Read nonvol and set up variables
 * 
 * return: Nonvol values copied to RAM
 * 
 *---------------------------------------------------------------
 *
 * Read the nonvol into RAM.  
 * 
 * If the results is uninitalized then force the factory default.
 * Then check for out of bounds and reset those values
 *
 *------------------------------------------------------------*/
void read_nonvol(void)
{
  unsigned int nonvol_init;
/*
 * Read the nonvol marker and if uninitialized then set up values
 */
#ifndef ESP32
  EEPROM.get(NONVOL_INIT, nonvol_init);
  if ( nonvol_init != INIT_DONE)                       // EEPROM never programmed
  {
    Serial.print("\r\nInitializing NON-VOL");
    gen_position(0); 
    EEPROM.put(NONVOL_DIP_SWITCH,  0);   // No, set up the defaults
    EEPROM.put(NONVOL_SENSOR_DIA,  230.0); 
    EEPROM.put(NONVOL_PAPER_TIME,  0);
    EEPROM.put(NONVOL_TEST_MODE,   0);
    EEPROM.put(NONVOL_CALIBRE_X10, 45);
    EEPROM.put(NONVOL_LED_PWM,     50);
    EEPROM.put(NONVOL_POWER_SAVE, 30);
    EEPROM.put(NONVOL_NAME_ID,    1);
    EEPROM.put(NONVOL_1_RINGx10, 1555);
    EEPROM.put(NONVOL_SEND_MISS,  0);
    EEPROM.put(NONVOL_SERIAL_NO,  0);
    
    nonvol_init = INIT_DONE;
    EEPROM.put(NONVOL_INIT, INIT_DONE);
  }

/*
 * Read in the values and check against limits
 */
  EEPROM.get(NONVOL_DIP_SWITCH, json_dip_switch);     // Read the nonvol settings
  EEPROM.get(NONVOL_SENSOR_DIA, json_sensor_dia);
  EEPROM.get(NONVOL_TEST_MODE,  json_test);
  EEPROM.get(NONVOL_LED_PWM,    json_LED_PWM);
  
  EEPROM.get(NONVOL_PAPER_TIME, json_paper_time);
  if ( (json_paper_time * PAPER_STEP) > (PAPER_LIMIT) )
  {
    json_paper_time = 0;                              // Check for an infinit loop
    EEPROM.put(NONVOL_PAPER_TIME, json_paper_time);   // and limit motor on time
  }
  
  EEPROM.get(NONVOL_CALIBRE_X10,  json_calibre_x10);
  if ( json_calibre_x10 > 100 )
  {
    json_calibre_x10 = 45;                            // Check for an undefined pellet
    EEPROM.put(NONVOL_CALIBRE_X10, json_calibre_x10); // Default to a 4.5mm pellet
  }
  json_calibre_x10 = 0;                               // AMB
  
  EEPROM.get(NONVOL_SENSOR_ANGLE,  json_sensor_angle);
  if ( json_sensor_angle == 0xffff )
  {
    json_sensor_angle = 45;                             // Check for an undefined Angle
    EEPROM.put(NONVOL_SENSOR_ANGLE, json_sensor_angle);// Default to a 4.5mm pellet
  }

  EEPROM.get(NONVOL_NAME_ID,  json_name_id);
  if ( json_name_id == 0xffff )
  {
    json_name_id = 0;                                 // Check for an undefined Name
    EEPROM.put(NONVOL_NAME_ID, json_name_id);         // Default to a 4.5mm pellet
  }
  
  json_sensor_angle = 45;                            // AMB
  
  EEPROM.put(NONVOL_SENSOR_ANGLE, json_sensor_angle);
    
  EEPROM.get(NONVOL_NORTH_X, json_north_x);  
  EEPROM.get(NONVOL_NORTH_Y, json_north_y);  
  EEPROM.get(NONVOL_EAST_X,  json_east_x);  
  EEPROM.get(NONVOL_EAST_Y,  json_east_y);  
  EEPROM.get(NONVOL_SOUTH_X, json_south_x);  
  EEPROM.get(NONVOL_SOUTH_Y, json_south_y);  
  EEPROM.get(NONVOL_WEST_X,  json_west_x);  
  EEPROM.get(NONVOL_WEST_Y,  json_west_y);  

  EEPROM.get(NONVOL_1_RINGx10,  json_1_ring_x10);

  EEPROM.get(NONVOL_POWER_SAVE, json_power_save);
  EEPROM.get(NONVOL_LED_PWM,    json_LED_PWM);
  EEPROM.get(NONVOL_SEND_MISS,  json_send_miss);
  EEPROM.get(NONVOL_SERIAL_NO,  json_serial_number);
#else
  File f = SPIFFS.open("/nvm.dat", "r");
  if(!f) { // file does not exist
    Serial.println("No nvm file, use defaults");
    nvmdata.init = 0;
  } else {
    Serial.println("Reading NVM file");
    f.read((byte*)&nvmdata,sizeof(nvmdata));
    f.close();
  }
  if ( nvmdata.init != INIT_DONE)                       // EEPROM never programmed
  {
    Serial.print("\r\nInitializing NON-VOL");
    gen_position(0); 
    nvmdata.dip_switch = 0;   // No, set up the defaults
    nvmdata.sensor_dia = 230.0; 
    nvmdata.paper_time = 0;
    nvmdata.test_mode = 0;
    nvmdata.calibre_x10 = 45;
    nvmdata.sensor_angle = 45;
    nvmdata.led_pwm = 50;
    nvmdata.power_save = 30;
    nvmdata.name_id = 1;
    nvmdata.ring_x10 = 1555;
    nvmdata.send_miss = 0;
    nvmdata.init = INIT_DONE;
    nonvol_init = 1; // need to write
  }

/*
 * Read in the values and check against limits
 */
  json_dip_switch = nvmdata.dip_switch;     // Read the nonvol settings
  json_sensor_dia = nvmdata.sensor_dia;
  json_test = nvmdata.test_mode;
  json_LED_PWM = nvmdata.led_pwm;
  
  json_paper_time = nvmdata.paper_time;
  if ( (json_paper_time * PAPER_STEP) > (PAPER_LIMIT) )
  {
    json_paper_time = 0;                              // Check for an infinit loop
    nvmdata.paper_time = json_paper_time;   // and limit motor on time
    nonvol_init = 1; // need to write
  }
  
  json_calibre_x10 = nvmdata.calibre_x10;
  if ( json_calibre_x10 > 100 )
  {
    json_calibre_x10 = 45;                            // Check for an undefined pellet
    nvmdata.calibre_x10 = json_calibre_x10; // Default to a 4.5mm pellet
    nonvol_init = 1; // need to write
  }
  json_calibre_x10 = 0;                               // AMB
  
  json_sensor_angle = nvmdata.sensor_angle;
  if ( json_sensor_angle == 0xffff )
  {
    json_sensor_angle = 45;                             // Check for an undefined Angle
    nvmdata.sensor_angle = json_sensor_angle;// Default to a 4.5mm pellet
    nonvol_init = 1; // need to write    
  }

  json_name_id = nvmdata.name_id;
  if ( json_name_id == 0xffff )
  {
    json_name_id = 0;                                 // Check for an undefined Name
    nvmdata.name_id = json_name_id;         // Default to a 4.5mm pellet
    nonvol_init = 1; // need to write    
  }
  
  json_north_x = nvmdata.north_x;  
  json_north_y = nvmdata.north_y;  
  json_east_x = nvmdata.east_x;  
  json_east_y = nvmdata.east_y;  
  json_south_x = nvmdata.south_x;  
  json_south_y = nvmdata.south_y;  
  json_west_x = nvmdata.west_x;  
  json_west_y = nvmdata.west_y;  
  json_1_ring_x10 = nvmdata.ring_x10;
  json_power_save = nvmdata.power_save;
  json_LED_PWM = nvmdata.led_pwm;
  json_send_miss = nvmdata.send_miss;
  json_serial_number = nvmdata.serial_number;

  if(nonvol_init == 1) {
    write_nvm_dat();
  }
#endif
/*
 * All done, begin the program
 */
  return;
}

/*----------------------------------------------------------------
 *
 * function: gen_postion
 * 
 * brief: Generate new position varibles based on new sensor diameter
 * 
 * return: Position values stored in NONVOL
 * 
 *---------------------------------------------------------------
 *
 *  This function resets the offsets to 0 whenever a new 
 *  sensor diameter is entered.
 *  
 *------------------------------------------------------------*/
void gen_position(int v)
{
 /*
  * Work out the geometry of the sensors
  */
  json_north_x = 0;
  json_north_y = 0;
  
  json_east_x = 0;
  json_east_y = 0;

  json_south_x = 0;
  json_south_y = 0;
  
  json_west_x = 0;
  json_west_y = 0;

 /*
  * Save to persistent storage
  */
#ifndef ESP32
  EEPROM.put(NONVOL_NORTH_X, json_north_x);  
  EEPROM.put(NONVOL_NORTH_Y, json_north_y);  
  EEPROM.put(NONVOL_EAST_X,  json_east_x);  
  EEPROM.put(NONVOL_EAST_Y,  json_east_y);  
  EEPROM.put(NONVOL_SOUTH_X, json_south_x);  
  EEPROM.put(NONVOL_SOUTH_Y, json_south_y);  
  EEPROM.put(NONVOL_WEST_X,  json_west_x);  
  EEPROM.put(NONVOL_WEST_Y,  json_west_y);  
#else
  nvmdata.north_x = json_north_x;
  nvmdata.north_y = json_north_y;
  nvmdata.east_x = json_east_x;
  nvmdata.east_y = json_east_y;
  nvmdata.south_x = json_south_x;
  nvmdata.south_y = json_south_y;
  nvmdata.west_x = json_west_x;
  nvmdata.west_y = json_west_y;
  write_nvm_dat();
#endif
 /* 
  *  All done, return
  */
  return;
}
#ifdef ESP32
/*
 write the nvm data file
*/
void write_nvm_dat() {
  File f = SPIFFS.open("/nvm.dat", "w");
  if(f && f.write((byte*)&nvmdata,sizeof(nvmdata)) ) {
    f.close();
    Serial.print("wrote new config\n");
  }
}
void update_nvm(unsigned int d, unsigned int v) {
  switch(d) {
  case NONVOL_INIT: nvmdata.init = v; break;
  case NONVOL_DIP_SWITCH: nvmdata.dip_switch = v; break;
  case NONVOL_PAPER_TIME: nvmdata.paper_time = v; break;
  case NONVOL_TEST_MODE: nvmdata.test_mode = v; break;
  case NONVOL_CALIBRE_X10: nvmdata.calibre_x10 = v; break;
  case NONVOL_SENSOR_ANGLE: nvmdata.sensor_angle = v; break;
  case NONVOL_NORTH_X: nvmdata.north_x = v; break;
  case NONVOL_NORTH_Y: nvmdata.north_y = v; break;
  case NONVOL_EAST_X: nvmdata.east_x = v; break;
  case NONVOL_EAST_Y: nvmdata.east_y = v; break;
  case NONVOL_SOUTH_X: nvmdata.south_x = v; break;
  case NONVOL_SOUTH_Y: nvmdata.south_y = v; break;
  case NONVOL_WEST_X : nvmdata.west_x = v; break;
  case NONVOL_WEST_Y: nvmdata.west_y = v; break;
  case NONVOL_POWER_SAVE: nvmdata.power_save = v; break;
  case NONVOL_NAME_ID: nvmdata.name_id = v; break;
  case NONVOL_1_RINGx10: nvmdata.ring_x10 = v; break;
  case NONVOL_LED_PWM: nvmdata.led_pwm = v; break;
  case NONVOL_SEND_MISS: nvmdata.send_miss = v; break;
  }
}
void update_nvm(unsigned int d, double v){
  switch(d) {
  case NONVOL_SENSOR_DIA: nvmdata.sensor_dia = v; break;
  }
}
//void update_nvm(unsigned int d, float v){
 // 
//}

#endif
