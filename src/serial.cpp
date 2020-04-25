#include <Arduino.h>
#include <pb_decode.h>
#include <SoftwareSerial.h>

#include "serial.h"
#include "packet.pb.h"
#include "util.h"

#define SERIAL_PIN 13

SoftwareSerial SoftSerial(SERIAL_PIN, -1); // RX, TX

bool init_serial()
{
    SoftSerial.begin(9600);
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
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "temperature", measurement.type.temperature);
        break;
    case Measurement_humidity_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "humidity", measurement.type.humidity);
        break;
    case Measurement_pressure_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "pressure", measurement.type.pressure);
        break;
    case Measurement_co2_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "co2", measurement.type.co2);
        break;
    case Measurement_light_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "light", measurement.type.light);
        break;
    case Measurement_electricity_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "electricity", measurement.type.electricity);
        break;
    case Measurement_gas_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "gas", measurement.type.gas);
        break;
    case Measurement_voltage_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "voltage", measurement.type.voltage);
        break;
    case Measurement_frequency_error_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "frequency_error", measurement.type.frequency_error);
        break;
    case Measurement_rssi_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "rssi", measurement.type.rssi);
        break;
    case Measurement_snr_tag:
        publish_measurement_float(meta->location, meta->sensor_type, measurement.sensor, "snr", measurement.type.snr);
        break;

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

    publish_status(packet.meta.location, "firmware", packet.meta.firmware_version);

    if (!success)
    {
        Serial.print("DECODING ERROR: ");
        Serial.println(stream.errmsg);
        return false;
    }

    return true;
}

// \0\0\0\0${LEN}${PACKET}
void poll_packet()
{
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