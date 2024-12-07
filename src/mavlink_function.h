#pragma once
#include <Arduino.h>
#include <common/mavlink.h>
#include "defines.h"

#define MAVLINK_SERIAL Serial2

#define stabilize 0
#define althold 1
#define loiter 2
#define guided 3

const uint8_t system_id = 1;
const uint8_t component_id = 1;

uint8_t type = MAV_TYPE_QUADROTOR;

uint8_t autopilot = MAV_AUTOPILOT_GENERIC;
uint8_t base_mode;
uint8_t system_status;

void mavlink_setup();
void mavlink_loop();
void mav_send_heartbeat();
void mav_receive_heartbeat();
void mav_arm();
void mav_disarm();
void mav_mode_update(uint8_t mode);
void mav_manual_control(float roll, float pitch, float yaw, float thrust);
void mav_guided_control(float latitude, float longitude, float altitude);


//state
/*
void mavlink_setup(){
  system_status = MAV_STATE_BOOT;
  MAVLINK_SERIAL.begin(57600);
  system_status = MAV_STATE_CALIBRATING;

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_QUADROTOR, autopilot, MAV_MODE_STABILIZE_DISARMED, 0, MAV_STATE_UNINIT);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVLINK_SERIAL.write(buf, len);
}

void mavlink_loop(RawData _data)
{
  if (manual_control)
    {
      // Roll
      _data.roll1 = constrain(_data.roll1, -45, 45);
      _data.roll1 = map(_data.roll1, -45, 45, 1200, 1800);
        
      // Pitch
      _data.pitch1 = constrain(_data.pitch1, -45, 45);
      _data.pitch1 = map(_data.pitch1, -45, 45, 1200, 1800);
        
      // Yaw (* temporarily, use the second roll for yaw)
      _data.yaw1 = constrain(_data.roll2, -45, 45);
      _data.yaw1 = map(_data.yaw1, -45, 45, 1200, 1800);
        
      // Throttle (for manual control use aux1 for throttle)
      _data.aux1 = map(_data.aux1, 0, 4095, 0, 2000);
      _data.aux1 = constrain(_data.aux1, 0, 1700);

      mav_manual_control(_data.roll1, _data.pitch1, _data.yaw1, _data.aux1);
    }
}

void mav_arm(){
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(system_id, component_id, &msg, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
}

void mav_disarm(){
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(system_id, component_id, &msg, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
}

void mav_mode_update(uint8_t mode){}

void mav_send_heartbeat(){
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_QUADROTOR, autopilot, MAV_MODE_STABILIZE_DISARMED, 0, MAV_STATE_UNINIT);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MAVLINK_SERIAL.write(buf, len);
}

void mav_manual_control(float roll, float pitch, float yaw, float thrust){
    mavlink_message_t msg;
    mavlink_msg_manual_control_pack(system_id, component_id, &msg, 0, roll, pitch, yaw, thrust, 1, 1, 1, 1);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MAVLINK_SERIAL.write(buf, len);
}

void mav_override_rc(int roll, int pitch, int throttle, int yaw) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_rc_channels_override_pack(0xFF, 0xBE, &msg, 1, 1, roll, pitch, throttle, yaw, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVLINK_SERIAL.write(buf, len);
}

void mavlink_update(RawData _data)
{
  (_data.arm == 0) ? mav_arm() : mav_disarm();
  mav_send_heartbeat();
  float altitude;
  if (manual_control)
  {
    //roll
    _data.roll1 = constrain(_data.roll1, -45, 45);
    _data.roll1 = map(_data.roll1, -45, 45 , 1200, 1800);
    //pitch
    _data.pitch1 = constrain(_data.pitch1, -45, 45);
    _data.pitch1 = map(_data.pitch1, -45, 45 , 1200, 1800);
    //yaw (* temporarily, use the second roll for yaw)
    _data.yaw1 = constrain(_data.roll2, -45, 45);
    _data.yaw1 = map(_data.yaw1, -45, 45 , 1200, 1800);
    //throttle (for manual control use aux1 for throtle), constrain to 
    _data.aux1 = map(_data.aux1, 0, 4095, 0, 2000);
    _data.aux1 = constrain(_data.aux1, 0, 1700);

    // throttleValue = _data.aux1;  // t : throttle / altitude

    mav_override_rc(_data.roll1, _data.pitch1, _data.aux1, _data.yaw1);
    ESP_LOGI("MAV", "rc, manual");
  } 
  else 
  {
 //throttle (* use the second pitch for throtle), constrain to 
    if (_data.heightControl.ALT_DOWN) pwm_throttle -= 50;
    if (_data.heightControl.ALT_UP) pwm_throttle += 50;
    if (_data.heightControl.ALT_IDLE) pwm_throttle += 0;
    if (_data.move.LEFT) _data.roll1 = 1200;
    if (_data.move.RIGHT) _data.roll1 = 1780;
    if (_data.move.NOSEDOWN) _data.pitch1 = 1200;
    if (_data.move.NOSEUP) _data.pitch1 = 1800;
    if (_data.move.YAW_LEFT) _data.yaw1 = 1200;
    if (_data.move.YAW_RIGHT) _data.yaw1 = 1800;
    if (_data.move.LEVEL) {
      _data.roll1 = 1500;
      _data.pitch1 = 1500;
    }

    mav_override_rc(_data.roll1, _data.pitch1, pwm_throttle, _data.yaw1);
    ESP_LOGI("MAV", "rc, semi");
  }
}
*/