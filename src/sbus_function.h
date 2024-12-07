#pragma once

#include "defines.h"

#include <Arduino.h>
#ifdef USE_ESP_DEBUG
  #include <esp_log.h>
#endif
#include <sbus.h>

//sbus
bfs::SbusTx sbus_tx(&SBUS_SERIAL, 16, 17, true, false); //enable internal inverter
bfs::SbusData data_to_fc; //sbus encoded data to fc

void sbus_begin()
{
  sbus_tx.Begin();

  // initial testing 
  for (int i = 1000; i < 1100; i++){
    data_to_fc.ch[3] = i;
    sbus_tx.data(data_to_fc);
    sbus_tx.Write();
  }
    
  data_to_fc.failsafe = 0; 
#ifdef USE_ESP_DEBUG
  ESP_LOGI(TAG_SBUS, "SBUS Initialized");
#endif
}

void process_sbus(ControlData *_control_data){
  data_to_fc.ch[0] = NORMALIZED_PWM_TO_FC(_control_data->pwm_roll); // a : roll
  data_to_fc.ch[1] = NORMALIZED_PWM_TO_FC(_control_data->pwm_pitch);// e : pitch
  data_to_fc.ch[2] = NORMALIZED_PWM_TO_FC(_control_data->pwm_thr);
  data_to_fc.ch[3] = NORMALIZED_PWM_TO_FC(_control_data->pwm_yaw);   // r : yaw
  data_to_fc.ch[4] = NORMALIZED_PWM_TO_FC(_control_data->pwm_arm);  // rc5 : 
  data_to_fc.ch[5] = NORMALIZED_PWM_TO_FC(_control_data->aux1);  // rc6 : 
  data_to_fc.ch[7] = NORMALIZED_PWM_TO_FC(_control_data->aux2);// random(1000, 2000); // ping
  data_to_fc.ch[8] = random(1000, 2000);
  sbus_tx.data(data_to_fc);
  sbus_tx.Write();
#ifdef USE_ESP_DEBUG
  ESP_LOGI(TAG_SBUS, "SBUS Data Sent");
#endif
}