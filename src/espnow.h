#pragma once

#include <Arduino.h>
#ifdef USE_ESP_DEBUG
  #include <esp_log.h>
#endif
#include <esp_now.h>
#include <WiFi.h>

#include "defines.h"

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&data, incomingData, sizeof(data));
#ifdef USE_ESP_DEBUG
  ESP_LOGD(TAG_ESPNOW, "Recv ESPNOW");
#endif
}

void esp_now_setup()
{
  WiFi.mode(WIFI_STA);
  // task : add specific peer  
  if (esp_now_init() == ESP_OK) {
#ifdef USE_ESP_DEBUG
    ESP_LOGI(TAG_ESPNOW, "ESP-NOW initialized successfully");
#endif
  } else {
#ifdef USE_ESP_DEBUG
    ESP_LOGE(TAG_ESPNOW, "Failed to initialize ESP-NOW");
#endif
  }
  esp_now_register_recv_cb(OnDataRecv);
}