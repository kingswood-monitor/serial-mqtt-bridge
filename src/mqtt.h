
#include <ESP8266WiFi.h>

bool init_mqtt();
bool loop_mqtt();
void reconnect_mqtt();

void publish_float(int device_type, int device_id, int sensor_id, const char *topic, float val);
void publish_int(int client, const char *topic, int val);
