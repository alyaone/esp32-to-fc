#pragma once
#include <Arduino.h>

Stream * _stream;
uint32_t _timeout;

//--- crc
uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void msp_begin(Stream & stream, uint32_t timeout)
{
  _stream   = &stream;
  _timeout  = timeout;
}


void msp_reset()
{
  _stream->flush();
  while (_stream->available() > 0)
    _stream->read();
}

void msp_send(uint16_t messageID, void * payload, uint16_t size)
{
  uint8_t flag = 0;
    uint8_t crc = 0;
    uint8_t tmp_buf[2];

    _stream->write('$');
    _stream->write('X');
    _stream->write('<');

    crc = crc8_dvb_s2(crc, flag);
    _stream->write(flag);

    memcpy(tmp_buf, &messageID, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    memcpy(tmp_buf, &size, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    uint8_t * payloadPtr = (uint8_t*)payload;
    for (uint8_t i = 0; i < size; ++i) {
        uint8_t b = *(payloadPtr++);
        crc = crc8_dvb_s2(crc, b);
        _stream->write(b);
    }
    _stream->write(crc);
}

void msp_send_v1(uint8_t messageID, void * payload, uint16_t size)
{
    uint8_t flag = 0;
    _stream->write('$');
    _stream->write('X');
    _stream->write('<');
    _stream->write(flag);
    _stream->write(messageID);
    _stream->write(size);
    uint8_t checksum = 0;
    uint8_t * payloadPtr = (uint8_t*)payload;
    for (uint8_t i = 0; i < size; ++i) {
        uint8_t b = *(payloadPtr++);
        checksum = crc8_dvb_s2(checksum, b);
        _stream->write(b);
    }
    _stream->write(checksum);
}

void msp_send_v2(uint16_t messageID, void * payload, uint8_t size) // 255 chars max, out of V2 specs
{
  uint8_t _crc = 0;
  uint8_t message[size + 9];
  message[0] = '$';
  message[1] = 'X';
  message[2] = '<';
  message[3] = 0; //flag
  message[4] = messageID; //function
  message[5] = messageID >> 8;
  message[6] = size; //payload size
  message[7] = size >> 8;
  for(uint8_t i = 3; i < 8; i++) {
    _crc = crc8_dvb_s2(_crc, message[i]);
  }
  //Start of Payload
  uint8_t * payloadPtr = (uint8_t*)payload;
  for (uint16_t i = 0; i < size; ++i) {
    message[i + 8] = *(payloadPtr++);
    _crc = crc8_dvb_s2(_crc, message[i + 8]);
  }
  message[size + 8] = _crc;
  _stream->write(message, sizeof(message));
}


bool msp_recv(uint8_t * messageID, void * payload, uint8_t maxSize, uint8_t * recvSize)
{
  uint32_t t0 = millis();

  while (1) {

    // read header
    while (_stream->available() < 6)
      if (millis() - t0 >= _timeout)
        return false;
    char header[3];
    _stream->readBytes((char*)header, 3);

    // check header
    if (header[0] == '$' && header[1] == 'M' && header[2] == '>') {
      // header ok, read payload size
      *recvSize = _stream->read();

      // read message ID (type)
      *messageID = _stream->read();

      uint8_t checksumCalc = *recvSize ^ *messageID;

      // read payload
      uint8_t * payloadPtr = (uint8_t*)payload;
      uint8_t idx = 0;
      while (idx < *recvSize) {
        if (millis() - t0 >= _timeout)
          return false;
        if (_stream->available() > 0) {
          uint8_t b = _stream->read();
          checksumCalc ^= b;
          if (idx < maxSize)
            *(payloadPtr++) = b;
          ++idx;
        }
      }
      // zero remaining bytes if *size < maxSize
      for (; idx < maxSize; ++idx)
        *(payloadPtr++) = 0;

      // read and check checksum
      while (_stream->available() == 0)
        if (millis() - t0 >= _timeout)
          return false;
      uint8_t checksum = _stream->read();
      if (checksumCalc == checksum) {
        return true;
      }

    }
  }

}

bool msp_recv_v2(uint16_t * messageID, void * payload, uint8_t maxSize, uint8_t * recvSize)
{
  uint32_t t0 = millis();

  while (1) {

    // read header
    while (_stream->available() < 6)
      if (millis() - t0 >= _timeout)
        return false;
    char header[4];
    _stream->readBytes((char*)header, 4);

    // check header
    if (header[0] == '$' && header[1] == 'X' && header[2] == '>') {

      // read message ID (type)
      *messageID = _stream->read();


    // header ok, read payload size
      *recvSize = _stream->read();



      // read payload
      uint8_t * payloadPtr = (uint8_t*)payload;
      uint8_t idx = 0;
      while (idx < *recvSize) {
        if (millis() - t0 >= _timeout)
          return false;
        if (_stream->available() > 0) {
          uint8_t b = _stream->read();

          if (idx < maxSize)
            *(payloadPtr++) = b;
          ++idx;
        }
      }
      // zero remaining bytes if *size < maxSize
      for (; idx < maxSize; ++idx)
        *(payloadPtr++) = 0;



        return true;



    }
  }

}

bool msp_waitFor(uint8_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize)
{
  uint8_t recvMessageID;
  uint8_t recvSizeValue;
  uint32_t t0 = millis();
  while (millis() - t0 < _timeout)
    if (msp_recv(&recvMessageID, payload, maxSize, (recvSize ? recvSize : &recvSizeValue)) && messageID == recvMessageID)
      return true;

  // timeout
  return false;
}

bool msp_waitFor2(uint16_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize)
{
  uint16_t recvMessageID;
  uint8_t recvSizeValue;
  uint32_t t0 = millis();
  while (millis() - t0 < _timeout)
    if (msp_recv_v2(&recvMessageID, payload, maxSize, (recvSize ? recvSize : &recvSizeValue)) && messageID == recvMessageID)
      return true;

  return false;
}

bool msp_req(uint8_t messageID, void * payload, uint8_t maxSize, uint8_t * recvSize)
{
  msp_send_v1(messageID, NULL, 0);
  return msp_waitFor(messageID, payload, maxSize, recvSize);
}


// send message and wait for ack
bool msp_command(uint8_t messageID, void * payload, uint8_t size, bool waitACK)
{
  msp_send_v1(messageID, payload, size);

  // ack required
  // if (waitACK)
  //   return msp_waitFor(messageID, NULL, 0);

  return true;
}

bool msp_command2(uint16_t messageID, void * payload, uint8_t size, bool waitACK)
{
  msp_send_v2(messageID, payload, size);

  // ack required
  // if (waitACK)
  //   return msp_waitFor2(messageID, NULL, 0);

  return true;
}