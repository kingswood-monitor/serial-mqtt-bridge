# Kingswood Monitoring System WiFi gateway firmware.

Firmware for a feather Huzzah ESP8288 to relay data from a LoRa hub onto WiFi via MQTT.

Huzzah ESP8266 / Arduino.

## Protobuf

To regenerate the schema, run this command:

```
mkdir -p lib/proto
python .pio/libdeps/huzzah/Nanopb/generator/nanopb_generator.py -I proto -D lib/proto proto/packet.proto
```


device/2/sensor/*/temp = 20
device/2/sensor/*/temp = 22

2/measurement/temperature = 21
