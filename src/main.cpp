#include <Arduino.h>
#include "alyatest_mpu.h"
#include <MSP.h>
#define MSP_SERIAL Serial2

MSP msp;

// put function declarations here:
void setup() {

  MSP_SERIAL.begin(115200);
  msp.begin(MSP_SERIAL);
  // Serial.begin(115200);

    // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  // put your setup code here, to run 
}

void loop() {
  uint16_t randInt1 = myData.valueRoll;
  uint16_t randInt2 = myData.valuePitch;
  uint16_t randInt3 = myData.throttle;
  uint16_t randInt4 = random(1500, 1600);
  uint16_t randInt5 = myData.arm;
  uint16_t randInt6 = myData.pushold;
  uint16_t randInt7 = random(1000, 1550);

  uint16_t smallRand1 = random(1000, 2000);
  uint16_t smallRand2 = random(1000, 2000);
  uint16_t smallRand3 = random(1000, 2000);
  uint16_t smallRand4 = random(1000, 2000);
  uint16_t smallRand5 = random(1000, 2000);
  uint16_t smallRand6 = random(1000, 2000);
  uint16_t smallRand7 = random(1000, 2000);
  
  // uint16_t rollValue = control_data.pwm_roll;
  // uint16_t pitchValue = control_data.pwm_pitch;
  // uint16_t throttleValue = random(1500,1600);// + smallRand;
  // uint16_t yawValue = control_data.pwm_yaw;
  // uint16_t arm = (data.arm == 1) ? 1900 : 1100;
  // uint16_t mode1 = control_data.aux1;
  // uint16_t mode2 = control_data.aux2;

/*
  uint8_t _rcData[10];
  _rcData[0] = control_data.pwm_roll & 0xFF;       // Roll (channel 1)
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

  // uint16_t rcData[14] = {rollValue, pitchValue, throttleValue, 
  //                        yawValue, arm, mode1, mode2, randInt, 
  //                        randInt, randInt, randInt, randInt, randInt, randInt};

  //coba test
  uint16_t rcData[8] = {randInt1, randInt2, randInt3, 
                         randInt4, randInt5, randInt6, randInt7, smallRand1};

  Serial.println(randInt1);
  Serial.print("valueRoll: ");
  Serial.print(myData.valueRoll);
  Serial.print(", valuePitch: ");
  Serial.println(myData.valuePitch);
  Serial.print("valueThrottle: ");
  Serial.print(myData.throttle);
  Serial.print("valueArm: ");
  Serial.print(myData.arm);
  Serial.print("pushold: ");
  Serial.print(myData.pushold);
  msp.command(MSP_SET_RAW_RC, &rcData, sizeof(rcData), true);
  MSP_SERIAL.flush();

  
}
