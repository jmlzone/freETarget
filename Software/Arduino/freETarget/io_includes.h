/* consolidate and conditionalize includes */
#include "freETarget.h"
#ifdef ESP32
  #include <analogWrite.h>
  #include <SPI.h>
  #include <FS.h>
  #include <SPIFFS.h>
  #include <Adafruit_NeoPixel.h>
#endif
#include "gpio.h"
#include "analog_io.h"
#include "json.h"
