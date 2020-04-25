#include <ArduinoWebsockets.h>
#include <ESP8266WiFi.h>
#include "DigitalOut.h"

#include "config.h"
#include "secrets.h"

Kingswood::Pin::DigitalOut blue_led(BLUE_LED_PIN);
Kingswood::Pin::DigitalOut red_led(RED_LED_PIN);

const char *ssid = SSID_NAME;                        //Enter SSID
const char *password = SSID_PASS;                    //Enter Password
const char *websockets_server_host = SERVER_HOST;    //Enter server adress
const uint16_t websockets_server_port = SERVER_PORT; // Enter server port

using namespace websockets;
WebsocketsClient client;

void onMessageCallback(WebsocketsMessage message)
{
    Serial.print("Got Message: ");
    Serial.println(message.data());
}

void onEventsCallback(WebsocketsEvent event, String data)
{
    if (event == WebsocketsEvent::ConnectionOpened)
    {
        red_led.turnOff();
        Serial.println("INFO: Connnection Opened");
    }

    else if (event == WebsocketsEvent::ConnectionClosed)
    {
        red_led.turnOn();
        Serial.println("INFO: Connnection Closed");
        client.connect(websockets_server_host, websockets_server_port, "/");
    }

    else if (event == WebsocketsEvent::GotPing)
    {
        Serial.println("INFO: Got a Ping!");
    }

    else if (event == WebsocketsEvent::GotPong)
    {
        Serial.println("INFO: Got a Pong!");
    }
}

bool init_socket()
{
    red_led.begin();
    red_led.activeLow();
    red_led.turnOn();

    blue_led.begin();
    blue_led.activeLow();
    blue_led.turnOff();

    // Connect to wifi
    Serial.print("INFO: Connecting to WiFi");
    WiFi.begin(ssid, password);

    // Wait some time to connect to wifi
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++)
    {
        Serial.print(".");
        delay(1000);
        blue_led.toggle();
    }
    blue_led.turnOn();
    Serial.print("Connected, IP: ");
    Serial.println(WiFi.localIP());

    // run callback when messages are received
    client.onMessage(onMessageCallback);

    // run callback when events are occuring
    client.onEvent(onEventsCallback);

    // Connect to server
    Serial.print("INFO: Connecting to websocket");
    while (!client.available())
    {
        client.connect(websockets_server_host, websockets_server_port, "/");
        Serial.print(".");
        delay(1000);
    }
    Serial.println();

    // Send a message
    client.send("Hello Server");

    // Send a ping
    client.ping();

    return client.available();
}

void socket_send_measurement(const uint8_t *data, const size_t len)
{
    while (!client.available())
    {
        red_led.toggle();
        client.connect(websockets_server_host, websockets_server_port, "/");
        delay(1000);
    }

    client.sendBinary((char *)data, len);
    red_led.blink(1, 100);
}

void loop_socket()
{
    client.poll();
}