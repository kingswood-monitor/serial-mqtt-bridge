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

#include "util.h"
#include "serial.h"

void setup()
{
  if (!init_device())
    Serial.println("ERROR: Failed to start device");

  if (!init_serial())
    Serial.println("ERROR: Failed to start Serial1");

  if (!init_display())
    Serial.println("ERROR: Failed to start display");
}

void loop()
{
  delay(1000);
}
