#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PubSubClient.h>

#include "util.h"
#include "secrets.h"

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
WiFiClient wifiClient;

bool init_device()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED, HIGH); // Off
    digitalWrite(LED_BLUE, HIGH);

    Serial.begin(57600);
    delay(2000);

    logo(FIRMWARE_NAME, FIRMWARE_VERSION, DEVICE_TYPE);

    return true;
}

bool init_display()
{
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

    return true;
}

WiFiClient init_wifi()
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

    return wifiClient;
}

void flash_LED(int times, int millis)
{
    for (int i = 0; i < times; ++i)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(millis);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(millis);
    }
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