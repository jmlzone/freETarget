/* consolidate and conditionalize includes */
#include "freETarget.h"
#ifdef ESP32
  #include "gpioESP32.h"
  #include <analogWrite.h>
#else
  #include "gpio.h"
#endif
#include "analog_io.h"
#include "json.h"
