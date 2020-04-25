#include <Arduino.h>

#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DigitalOut.h"

#include "config.h"
#include "util.h"
#include "secrets.h"

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
Kingswood::Pin::DigitalOut util_red_led(RED_LED_PIN);

char chip_id[8];

void display_logo(char *title, char *version);
void generate_chip_id();
void reconnect();
void flash_LED(int times, int millis);

bool init_device()
{
    Serial.begin(57600);
    delay(2000);

    util_red_led.begin();
    util_red_led.activeLow();
    util_red_led.turnOff();

    display_logo(FIRMWARE_NAME, FIRMWARE_VERSION);

    generate_chip_id();

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

void generate_chip_id()
{
    sprintf(chip_id, "%08X\0", ESP.getChipId());
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

void display_logo(char *title, char *version)
{
    char strap_line[200];
    sprintf(strap_line, "                  |___/  %s v%s", title, version);

    Serial.println("  _  __ _                                                _ ");
    Serial.println(" | |/ /(_) _ __    __ _  ___ __      __ ___    ___    __| |");
    Serial.println(" | ' / | || '_ \\  / _` |/ __|\\ \\ /\\ / // _ \\  / _ \\  / _` |");
    Serial.println(" | . \\ | || | | || (_| |\\__ \\ \\ V  V /| (_) || (_) || (_| |");
    Serial.println(" |_|\\_\\|_||_| |_| \\__, ||___/  \\_/\\_/  \\___/  \\___/  \\__,_|");
    Serial.println(strap_line);
    Serial.println();
}