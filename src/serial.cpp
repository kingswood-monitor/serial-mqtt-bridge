#include <Arduino.h>
#include <pb_decode.h>
#include <SoftwareSerial.h>

#include "DigitalOut.h"
#include "serial.h"
#include "socket.h"
#include "packet.pb.h"
#include "config.h"
#include "util.h"

#define SERIAL_PIN 13

SoftwareSerial SoftSerial(SERIAL_PIN, -1); // RX, TX
Kingswood::Pin::DigitalOut serial_red_led(RED_LED_PIN);

bool handle_packet(uint8_t bytes_to_read);

bool init_serial()
{
    SoftSerial.begin(9600);
    serial_red_led.begin();
    serial_red_led.activeLow();
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
void poll_serial()
{
    while (SoftSerial.available() > 0)
    {
        serial_red_led.turnOn();
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
            serial_red_led.turnOff();
            break;
        }
    }
}

bool handle_packet(uint8_t bytes_to_read)
{

    // Read packet from Serial1
    uint8_t buf[255] = {};
    SoftSerial.readBytes(buf, bytes_to_read);

    // Send to websocket
    socket_send_measurement(buf, bytes_to_read);

    return true;
}