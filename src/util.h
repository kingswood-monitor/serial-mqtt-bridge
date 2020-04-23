#pragma once

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// firmware info
#define FIRMWARE_NAME "LoRa/Wifi Bridge "
#define FIRMWARE_VERSION "1.0.0"
#define DEVICE_TYPE "HUZZAHESP8266"

// Hardware configuration
#define LED_RED 0
#define LED_BLUE 2

bool init_device();
bool init_display();
WiFiClient init_wifi();

void logo(char *title, char *version, char *type);
void reconnect();
void flash_LED(int times, int millis);