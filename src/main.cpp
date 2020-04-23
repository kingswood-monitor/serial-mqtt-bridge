/*
    Kingswood Monitoring System
    MQTT/Serial Bridge

    Receives a Protobuf enecoded packet by Serial and broadcasts data via MQTT topics.

    Hardware: Adafruit feather32u4 RFM95
    OS:       Arduino

 */

#include <pb_decode.h>
#include "packet.pb.h"

#include "util.h"
#include "mqtt.h"
#include "serial.h"

void setup()
{
  if (init_device())
    Serial.println("Device started [OK]");

  if (init_serial())
    Serial.println("");

  if (init_display())
    Serial.println("Display started [OK]");

  init_wifi();
  if (init_mqtt())
    Serial.println("");
}

void loop()
{
  if (!loop_mqtt())
    reconnect_mqtt();

  poll_packet();
}
