// act 

#include <Arduino.h>
#ifdef USE_ESP_DEBUG
  #include <esp_log.h>
#endif
#include "defines.h"
#include "JSON.h"
JsonDocument doc;

void joystick_setup(){
  JOY_SERIAL.begin(57600);
}

void joystick_loop(ControlData *_control_data){
#ifdef USE_ESP_DEBUG
  ESP_LOGD(TAG_JOYSTICK, "x:%d \ty:%d \tz:%d \trz:%d", 
          _control_data->joys.x, _control_data->joys.y, _control_data->joys.z, _control_data->joys.rz);
#endif
  doc["x"] = _control_data->joys.x;
  doc["y"] = _control_data->joys.y;
  doc["z"] = _control_data->joys.z;
  doc["rz"] = _control_data->joys.rz;
  String str;
  serializeJson(doc, str);
  str += '@';
  JOY_SERIAL.println(str);
}