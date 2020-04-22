/*
    Kingswood Monitoring System
    MQTT/Serial Bridge

    Receives a Protobuf enecoded packet by Serial and broadcasts data via MQTT topics.

    Hardware: Adafruit feather32u4 RFM95
    OS:       Arduino

 * 
 *  ESP8266         32U4RFM95
 *  ------------------------------
 *  RXD2 [13]  <--  TXD1 [1]
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

  WiFiClient wifiClient = init_wifi();
  if (init_mqtt(wifiClient))
    Serial.println("");
}

void loop()
{
  if (!loop_mqtt())
    reconnect_mqtt();

  poll_packet();
}
