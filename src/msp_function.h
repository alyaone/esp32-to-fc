#pragma once

#include "defines.h"

#include <Arduino.h>
#ifdef USE_ESP_DEBUG
  #include <esp_log.h>
#endif
#include <MSP.h>
#ifdef USE_GARMIN_LIDAR
  #include "msp_protocol_v2_sensor_msg.h"
#endif
// #ifdef USE_GARMIN_LIDAR
//   LIDARLite rangefinder;
// #endif


MSP msp;

void msp_begin() 
{
  MSP_SERIAL.begin(115200);
  msp.begin(MSP_SERIAL);

#ifdef USE_ESP_DEBUG
  ESP_LOGI(TAG_MSP, "MSP Initialized");
#endif
}

void process_msp(ControlData *_control_data) {

#ifdef USE_GARMIN_LIDAR
  mspSensorRangefinderDataMessage_t rngfndr = { (uint8_t)0, (int32_t)rngfndr_distance };
  msp.send(MSP2_SENSOR_RANGEFINDER, &rngfndr, sizeof(rngfndr));
  MSP_SERIAL.flush();
#endif

  uint16_t randInt = random(1450, 1550);
  uint16_t smallRand = random(10, 20);
  
  uint16_t rollValue = control_data.pwm_roll;
  uint16_t pitchValue = control_data.pwm_pitch;
  uint16_t throttleValue = control_data.pwm_thr;// + smallRand;
  uint16_t yawValue = control_data.pwm_yaw;
  uint16_t arm = (data.arm == 1) ? 1900 : 1100;
  uint16_t mode1 = control_data.aux1;
  uint16_t mode2 = control_data.aux2;

/*
  uint8_t _rcData[10];
  _rcData[0] = control_data.pwm_roll & 0xFF;         // Roll (channel 1)
  _rcData[1] = (rollValue >> 8) & 0xFF;
  _rcData[2] = pitchValue & 0xFF;        // Pitch (channel 2)
  _rcData[3] = (pitchValue >> 8) & 0xFF;
  _rcData[4] = throttleValue & 0xFF;     // Throttle (channel 3)
  _rcData[5] = (throttleValue >> 8) & 0xFF;
  _rcData[6] = yawValue & 0xFF;          // Yaw (channel 4)
  _rcData[7] = (yawValue >> 8) & 0xFF;
  _rcData[8] = arm & 0xFF;
  _rcData[9] = (arm >> 8) & 0xFF;
  _rcData[10] = randInt & 0xFF;
  _rcData[11] = (randInt >> 8) & 0xFF;
  _rcData[12] = mode1 & 0xFF;
  _rcData[13] = (mode1 >> 8) & 0xFF;
*/

  uint16_t rcData[14] = {rollValue, pitchValue, throttleValue, yawValue, arm, mode1, mode2, randInt, randInt, randInt, randInt, randInt, randInt, randInt};

  msp.command(MSP_SET_RAW_RC, &rcData, sizeof(rcData), true);
  MSP_SERIAL.flush();
  // Serial.print(control_data.pwm_roll);
  // Serial.print("\t");
  // Serial.print(control_data.pwm_pitch);
  // Serial.print("\t");
  // Serial.print(control_data.pwm_yaw);
  // Serial.print("\t");
  // Serial.println(control_data.pwm_thr);
}