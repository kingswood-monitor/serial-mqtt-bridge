#pragma once

// firmware info
#define FIRMWARE_NAME "LoRa/Wifi Bridge "
#define FIRMWARE_VERSION "0.3"
#define DEVICE_TYPE "HUZZAHESP8266"

// Hardware configuration
#define LED_RED 0
#define LED_BLUE 2

// SoftwareSerial
#define SERIAL_PIN 13

// Transmission packet start and end characters
#define KW_START_MARKER 0x3C     // "<"
#define KW_END_MARKER 0x3E       // ">"
#define KW_SEPARATOR_MARKER 0x3B // ";"
#define KW_MAX_PACKET_LENGTH 255

// MQTT
#define MQTT_SERVER_IP IPAddress(192, 168, 1, 30)
#define KW_MQTT_MAX_TOPIC_LENGTH 30
IPAddress mqtt_server(192, 168, 1, 30);

// Blynk
#define BLINK_AUTH "zMkMyutAQEtOG6aU8Y-ELB4AI4PkxGav" // Project "Outside"

// the incoming byte when deserialising a LoRa packet
uint8_t incomingByte;

// buffer for storing a received packet
char buffer[KW_MAX_PACKET_LENGTH] = "";
int char_ptr;
bool reading_data = false;

struct SensorReadings
{
    int client_id;
    int packet_id;
    float temperature;
    float humidity;
    float co2;
    float light;
    float pressure;
    float battery;
    int lora_rssi;
    int lora_snr;
    int lora_frequency_error;
};

void logo(char *title, char *version, char *type);
void setup_wifi();
void reconnect();
void publish_float(int client, const char *topic, float val);
void publish_int(int client, const char *topic, int val);
void process_buffer_data(char *buf, SensorReadings *rdgs);
void publish_readings(SensorReadings *readings);
void flashLED(int times, int millis);
void display_oled(SensorReadings *readings);