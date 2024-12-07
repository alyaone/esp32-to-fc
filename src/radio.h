#pragma once

#include "defines.h"

#include <Arduino.h>
#ifdef USE_ESP_DEBUG
  #include <esp_log.h>
#endif
#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 5);

void radio_setup() {
  radio.begin();
  radio.openReadingPipe(0, glove_radio_address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_HIGH);  
  radio.startListening();
#ifdef USE_ESP_DEBUG
  ESP_LOGI(TAG_RADIO, "Radio Initialized");
#endif
}

void radio_loop(){
  // if (radio.readable()) {
  radio.read(&data, sizeof(data));
/* 
  ESP_LOGD(TAG_RADIO, "yl:%d yr:%d left:%d right:%d forw:%d back:%d lvl:%d up:%d down:%d idle:%d Pitch1: %.2f Yaw1: %.2f Roll2: %.2f Pitch2: %.2f Yaw2: %.2f Aux1: %d Aux2: %d Vertical: %d Arm: %d",
         data.move.YAW_LEFT, data.move.YAW_RIGHT, data.move.LEFT, data.move.RIGHT, data.move.NOSEDOWN, data.move.NOSEUP,
         data.heightControl.ALT_UP, data.heightControl.ALT_DOWN,
         data.roll1, data.pitch1, data.yaw1, data.roll2, data.pitch2, data.yaw2, data.aux1, data.aux2, data.vertical, data.arm);
*/
  // }
}