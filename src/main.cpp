/** 
 *  MQTT Client firmware for HUZZAH ESP8266.
 *  
 *  Implements a Reliable Datagram client for receiving LoRa packets over Serial1 from a 
 *  LoRan server. Topics are decoded assuming "<" and ">" start/end and ";" separator characters.
 *  Relays data onto the WiFi via MQTT. MQTT topics are constructed as [clientID]/data/temperature 
 *  etc. from a clientID provided in the packet. The onboard LED flashes the Client ID number on 
 *  successful transmission, and one long flash for fail. 
 *  
 *  PINOUT
 * 
 *  ESP8266         32U4RFM95
 *  ------------------------------
 *  RXD2 [13]  <--  TXD1 [1]
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <pb_decode.h>
#include "packet.pb.h"

#include "config.h"
#include "secrets.h"

#define DEBUG true // prints debug into for received serial data

SoftwareSerial SoftSerial(SERIAL_PIN, -1); // RX, TX
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

void setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH); // Off
  digitalWrite(LED_BLUE, HIGH);

  Serial.begin(57600);
  SoftSerial.begin(9600);
  delay(2000);

  logo(FIRMWARE_NAME, FIRMWARE_VERSION, DEVICE_TYPE);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  delay(1000);

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Kingswood");

  display.setCursor(0, 10);
  display.print(FIRMWARE_NAME);

  char buf[30];
  sprintf(buf, "Version %s", FIRMWARE_VERSION);
  display.setCursor(0, 20);
  display.print(buf);

  display.display();

  delay(5000);

  setup_wifi();
  mqttClient.setServer(mqtt_server, 1883);
}

bool read_data(pb_istream_t *stream, const pb_field_iter_t *field, void **arg)
{
  Meta *meta = *(Meta **)arg;

  Measurement measurement;
  if (!pb_decode(stream, Measurement_fields, &measurement))
  {
    Serial.println("read_data.pb_decode = Decode error, false");
    return false;
  }

  switch (measurement.which_type)
  {
  case Measurement_temperature_tag:
    publish_float(meta->device_id, "measurement/temperature", measurement.type.temperature);
    break;
  case Measurement_humidity_tag:
    publish_float(meta->device_id, "measurement/humidity", measurement.type.humidity);
    break;
  case Measurement_pressure_tag:
    publish_float(meta->device_id, "measurement/pressure", measurement.type.pressure);
    break;
  case Measurement_co2_tag:
    publish_float(meta->device_id, "measurement/co2", measurement.type.co2);
    break;
  case Measurement_light_tag:
    publish_float(meta->device_id, "measurement/light", measurement.type.light);
    break;
  case Measurement_power_tag:
    publish_float(meta->device_id, "measurement/power", measurement.type.power);
    break;
  case Measurement_voltage_tag:
    publish_float(meta->device_id, "measurement/voltage", measurement.type.voltage);
  default:
    break;
  }

  return true;
}

bool handle_packet(uint8_t packet_length)
{
  uint8_t buf[255] = {};
  SoftSerial.readBytes(buf, packet_length);

  // Decode packet
  Packet packet = Packet_init_zero;
  packet.measurements.funcs.decode = read_data;
  packet.measurements.arg = &packet.meta;
  pb_istream_t stream = pb_istream_from_buffer(buf, packet_length);
  bool success = pb_decode(&stream, Packet_fields, &packet);

  if (!success)
  {
    Serial.print("DECODING ERROR: ");
    Serial.println(stream.errmsg);
    return false;
  }

  return true;
}

typedef enum states
{
  START,
  GET_LENGTH,
  READ,
} state_machine;

uint8_t sentinel = '\0';
uint8_t sentinel_count = 4;
uint8_t count = 0;
uint8_t bytes_to_read = 0;
state_machine state = START;

// \0\0\0\0${LEN}${PACKET}
void loop()
{
  mqttClient.loop();

  while (SoftSerial.available() > 0)
  {
    digitalWrite(LED_RED, LOW); // inverted output
    switch (state)
    {
    case START:
      if (SoftSerial.read() == sentinel)
        count += 1;
      else
        count = 0;

      if (count == sentinel_count)
      {
        state = GET_LENGTH;
        count = 0;
      }
      break;
    case GET_LENGTH:
      bytes_to_read = SoftSerial.read();
      state = READ;

      Serial.print(F("[->] "));
      Serial.print(bytes_to_read);
      Serial.println(F(" bytes : "));

      break;
    case READ:
      if (!handle_packet(bytes_to_read))
        Serial.println("Malformed packet");

      state = START;
      digitalWrite(LED_RED, HIGH); // inverted output
      break;
    }
  }
}
/**
 *  Publishes readings to MQTT topics derived from client_id
 */
void publish_readings(SensorReadings *readings)
{
  if (!mqttClient.connected())
    reconnect();

  publish_float(readings->client_id, "data/temperature", readings->temperature);
  publish_float(readings->client_id, "data/humidity", readings->humidity);
  publish_float(readings->client_id, "data/co2", readings->co2);
  publish_float(readings->client_id, "data/light", readings->light);
  publish_float(readings->client_id, "data/pressure", readings->pressure);
  publish_float(readings->client_id, "info/vbat", readings->battery);
  publish_int(readings->client_id, "info/packet_id", readings->packet_id);
  publish_int(readings->client_id, "info/lora_rssi", readings->lora_rssi);
  publish_int(readings->client_id, "info/lora_snr", readings->lora_snr);
  publish_int(readings->client_id, "info/lora_frequency_error", readings->lora_frequency_error);
}

void publish_float(int client, const char *topic, float val)
{
  char val_buf[10];
  char topic_buf[KW_MQTT_MAX_TOPIC_LENGTH];

  sprintf(topic_buf, "%d/%s", client, topic);
  sprintf(val_buf, "%.2f", val);

  Serial.print(topic_buf);
  Serial.print(" ");
  Serial.println(val_buf);

  mqttClient.publish(topic_buf, val_buf);
}

void publish_int(int client, const char *topic, int val)
{
  char val_buf[10];
  char topic_buf[KW_MQTT_MAX_TOPIC_LENGTH];

  sprintf(topic_buf, "%d/%s", client, topic);
  sprintf(val_buf, "%d", val);

  mqttClient.publish(topic_buf, val_buf);
}

/* UTILITIES ********************/

void setup_wifi()
{
  char line_0[32], line_1[32], line_2[32];

  // update OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  sprintf(line_0, "SSID: %s", SSID_NAME);
  display.print(line_0);
  display.display();

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID_NAME);

  WiFi.begin(SSID_NAME, SSID_PASS);

  bool led_state = false;
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_BLUE, led_state);
    delay(200);
    led_state = !led_state;
    digitalWrite(LED_BLUE, led_state);
    delay(200);

    Serial.print(".");
  }

  digitalWrite(LED_BLUE, LOW);

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.RSSI());

  uint32_t ip = (uint32_t)WiFi.localIP();
  sprintf(line_1, "IP  : %u.%u.%u.%u", ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF);
  display.setCursor(0, 10);
  display.print(line_1);

  sprintf(line_2, "RSSI: %idB", WiFi.RSSI());
  display.setCursor(0, 20);
  display.print(line_2);

  display.display();
}

void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("arduinoClient"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void flashLED(int times, int millis)
{
  for (int i = 0; i < times; ++i)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(millis);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(millis);
  }
}

void display_oled(SensorReadings *readings)
{
#define LINE_LENGTH 20
#define LINE_HEIGHT 10

  char line_0[LINE_LENGTH], line_1[LINE_LENGTH], line_2[LINE_LENGTH];
  sprintf(line_0, "RX : %i/%i (%idB)", readings->client_id, readings->packet_id, readings->lora_rssi);
  sprintf(line_1, "BAT: %.3fV", readings->battery);
  sprintf(line_2, "TMP: %.1fC", readings->temperature);

  display.clearDisplay();

  display.setCursor(0, 0);
  display.print(line_0);

  display.setCursor(0, 1 * LINE_HEIGHT);
  display.print(line_1);

  display.setCursor(0, 2.5 * LINE_HEIGHT);
  display.print(line_2);

  display.display();
}

void logo(char *title, char *version, char *type)
{
  char strap_line[200];
  sprintf(strap_line, "                  |___/  %s v%s on %s", title, version, type);

  Serial.println("  _  __ _                                                _ ");
  Serial.println(" | |/ /(_) _ __    __ _  ___ __      __ ___    ___    __| |");
  Serial.println(" | ' / | || '_ \\  / _` |/ __|\\ \\ /\\ / // _ \\  / _ \\  / _` |");
  Serial.println(" | . \\ | || | | || (_| |\\__ \\ \\ V  V /| (_) || (_) || (_| |");
  Serial.println(" |_|\\_\\|_||_| |_| \\__, ||___/  \\_/\\_/  \\___/  \\___/  \\__,_|");
  Serial.println(strap_line);
  Serial.println();
}