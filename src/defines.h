#pragma once
#include <Arduino.h>

// ------Parameters, Functions and Directives
// 
// #define USE_ESP_DEBUG   // ESP_LOGx
#define USE_WIFI        // use espnow. radio is used if disabled

//----communication method to vehicle
#define USE_MSP         // can ovveride rc, send sensor data. bidirectional
// #define USE_MAVLINK     // more features than msp
// #define USE_SBUS        // work as using a pwm based receiver 
// #define USE_JOYSTICK    // (just interface to teensy as hid device) for gamepad (tested on uncrash fpv)

//----control method
#define MANUAL_CONTROL
  // #define GUIDED_CONTROL

//----sensor
// #define USE_GARMIN_LIDAR
#define MSP2_SENSOR_RANGEFINDER     0x1F01

//----log tags (for esp debug)
#define TAG_ESPNOW "ESPNOW"
#define TAG_JOYSTICK "JOY"
#define TAG_RADIO "RADIO"
#define TAG_SBUS "SBUS"

//----serial dev
#define JOY_SERIAL Serial2
#define SBUS_SERIAL Serial2
#define MSP_SERIAL Serial2

//----directive functions
#define NORMALIZED_PWM(x) ( x * 0.6251f + 881.74f)          // normalize incoming sbus 
#define NORMALIZED_PWM_TO_FC(x) ((x - 881.74f) / 0.6251f)   // normalize output sbus

//----data structures
// glove data
struct TILT {
  bool YAW_LEFT, YAW_RIGHT, LEFT, RIGHT, NOSEDOWN, NOSEUP;//, LEVEL;
};  // 7 bytes

struct VERTICAL {
  bool ALT_UP, ALT_DOWN;//, ALT_IDLE;
};  // 3 bytes

struct RawData 
{
  TILT move;
  VERTICAL heightControl;
  int aux1;
  int aux2;
  int camera_x;
  int camera_z;
  bool arm;
  bool grip;
  bool failsafe;
  bool isVertical;
};

RawData data; // received data from radio

// joystick data 
struct JoystickData{
  int x, y, z, rz;
};

// control data (on receiver device)
struct ControlData{
  JoystickData joys;
//rc data
  uint16_t pwm_pitch;
  uint16_t pwm_roll;
  uint16_t pwm_yaw;
  uint16_t pwm_thr;
//guided data
  float ang_pitch;
  float ang_roll;
  float ang_yaw;
  float thr;
  bool arming;

  uint16_t aux1;
  uint16_t aux2;
  uint16_t pwm_arm;
};

ControlData control_data;

//----control parameters
#define MAX_THROTTLE    1800
#define MID_THROTTLE    1500
#define MAX_PITCH_THRS  
#define MAX_ROLL_THRS   
#define MAX_YAW_THRS    
#define TRIM_LEFT_ANGLE 
#define TRIM_RIGHT_ANGLE
#define TRIM_YAW_ANGLE
#define PWM_PITCH_UP  1250
#define PWM_PITCH_DOWN 1750
#define PWM_ROLL_LEFT   1250
#define PWM_ROLL_RIGHT  1750
#define PWM_YAW_LEFT   1400
#define PWM_YAW_RIGHT  1600
#define DEFAULT_THROTTLE 985
#define PWM_MIDDLE  1500
#define THROTTLE_SENSITIVITY 3.5
#define THROTTLE_DOWN_SENSITIVITY 3.5
#define THROTTLE_VALUE 70
#define PWM_ARM 1100
#define PWM_DISARM 1900
#define SERVO_GRIP_ANGLE 30

//----
const uint64_t glove_radio_address = 0xF0F0F0F0E1LL;

float rngfndr_distance;

void process_control_data(ControlData *_control_data) //process thresholded data
{
  
  if (data.failsafe) {
    _control_data->arming = 0;
    _control_data->pwm_thr = DEFAULT_THROTTLE;
    _control_data->pwm_arm = PWM_DISARM;
  } else {
    _control_data->pwm_arm = PWM_ARM;
  }

  // (data_pwm.arm == 0) ? 1100 : 1900;

  if (data.heightControl.ALT_UP) 
  {
    _control_data->pwm_thr += THROTTLE_SENSITIVITY;
    _control_data->thr += THROTTLE_SENSITIVITY;
  }
  else if (data.heightControl.ALT_DOWN)
  {
    _control_data->pwm_thr -= THROTTLE_DOWN_SENSITIVITY;
    _control_data->thr -= THROTTLE_DOWN_SENSITIVITY; 
  }
  else
  {
    if (data.isVertical){
      _control_data->pwm_thr += 0;
      _control_data->thr += 0;
    } else {
      _control_data->pwm_thr = MID_THROTTLE;
      _control_data->thr = MID_THROTTLE;
    }
  }

  /*
  if (!data.arm){
    _control_data->pwm_thr = DEFAULT_THROTTLE;
    _control_data->thr = DEFAULT_THROTTLE;
  }else{
  if (data.heightControl.ALT_UP) 
  {
    _control_data->pwm_thr = MID_THROTTLE +  THROTTLE_VALUE;
    _control_data->thr = MID_THROTTLE + THROTTLE_VALUE;
  }
  else if (data.heightControl.ALT_DOWN)
  {
    _control_data->pwm_thr = MID_THROTTLE - THROTTLE_VALUE;
    _control_data->thr = MID_THROTTLE - THROTTLE_VALUE; 
  }
  else
  {
    _control_data->pwm_thr = MID_THROTTLE;
    _control_data->thr = MID_THROTTLE;
  }
  }
  */

  if (data.move.NOSEUP)
  {
    _control_data->pwm_pitch = PWM_PITCH_UP;

  }
  else if (data.move.NOSEDOWN)
  {
    _control_data->pwm_pitch = PWM_PITCH_DOWN;

  }
  else 
  {
    _control_data->pwm_pitch = PWM_MIDDLE;
  }

  if (data.move.RIGHT)
  {
    _control_data->pwm_roll = PWM_ROLL_RIGHT;

  }
  else if (data.move.LEFT)
  {
    _control_data->pwm_roll = PWM_ROLL_LEFT;

  }
  else 
  {
    _control_data->pwm_roll = PWM_MIDDLE;

  }

  if (data.move.YAW_RIGHT)
  {
    _control_data->pwm_yaw = PWM_YAW_RIGHT;
  }
  else if (data.move.YAW_LEFT)
  {
    _control_data->pwm_yaw = PWM_YAW_LEFT;
  }
  else 
  {
    _control_data->pwm_yaw = PWM_MIDDLE;
  }
  
  if (control_data.pwm_thr > MAX_THROTTLE) control_data.pwm_thr = MAX_THROTTLE;
  if (control_data.pwm_thr < DEFAULT_THROTTLE) control_data.pwm_thr = DEFAULT_THROTTLE;

  _control_data->joys.x = map(_control_data->pwm_pitch, 1000, 2000, 0, 1024);
  _control_data->joys.y = map(_control_data->pwm_roll, 1000, 2000, 0, 1024);
  _control_data->joys.z = map(_control_data->pwm_thr, 1000, 2000, 0, 1024);
  _control_data->joys.rz = map(_control_data->pwm_yaw, 1000, 2000, 0, 1024);
  _control_data->aux1 = map(data.aux1, 0, 4095, 985, 2000);
  _control_data->aux2 = map(data.aux2, 0, 4095, 985, 2000);

#ifdef USE_ESP_DEBUG
  ESP_LOGD("CONTROL_DATA", "r:%d \tp:%d\t", _control_data->pwm_roll, _control_data->pwm_pitch);
#endif
}

