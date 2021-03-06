/*
    Kingswood Monitoring System
    Websocket/Serial Bridge

    Receives a Protobuf enecoded packet by Serial and broadcasts data via websocket.

    Hardware: Adafruit feather32u4 RFM95
    OS:       Arduino

 */
#include <Arduino.h>

#include <pb_decode.h>
#include "packet.pb.h"

#include "socket.h"
#include "util.h"
#include "serial.h"

#define REFRESH_MILLIS 1000

void setup()
{
  if (!init_device())
    Serial.println("ERROR: Failed to start device");

  if (!init_serial())
    Serial.println("ERROR: Failed to start Serial1");

  if (!init_display())
    Serial.println("ERROR: Failed to start display");

  if (!init_socket())
  {
    Serial.println("ERROR: Failed to connect socket. Freezing...");
    while (1)
      ;
  }
}

uint16_t packet_id = 0;
uint8_t packet_buffer[255];

void loop()
{
  poll_serial();

  // delay(REFRESH_MILLIS);
}
