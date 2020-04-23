
#include <ESP8266WiFi.h>

bool init_mqtt();
bool loop_mqtt();
void reconnect_mqtt();

void publish_measurement_float(int location_id, int sensor_type, int sensor_id, const char *topic, float val);
void publish_status(int location_id, const char *topic, const char *data);
