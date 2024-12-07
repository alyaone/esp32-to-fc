#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  uint16_t roll;
  uint16_t pitch;
  uint16_t valueRoll;
  uint16_t valuePitch;
  uint16_t yaw;
  uint16_t throttle;
  uint16_t potPin = 32;
  uint16_t potVal;
  int pushold;
  int pwmchannel = 0;
  int pwmfreq = 50;
  int pwmresolution = 16;
  int cam;
  uint16_t arm;
  bool pick;
  byte mode;
} 
struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Rotation X: ");
  Serial.print(myData.roll);
  Serial.print(", Y: ");
  Serial.print(myData.pitch);
  Serial.print(", Z: ");
  Serial.print(myData.yaw);
  Serial.println(" rad/s");

  Serial.print("valueRoll: ");
  Serial.print(myData.valueRoll);
  Serial.print(", valuePitch: ");
  Serial.println(myData.valuePitch);
  Serial.print("valueThrottle: ");
  Serial.print(myData.throttle);
  Serial.print("valueArm: ");
  Serial.print(myData.arm);
}
 

 
