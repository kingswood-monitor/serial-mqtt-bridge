
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// MQTT
#define MQTT_SERVER_IP IPAddress(192, 168, 1, 30)
#define KW_MQTT_MAX_TOPIC_LENGTH 40

WiFiClient w;
PubSubClient mqttClient(w);

bool init_mqtt()
{
    mqttClient.setServer(MQTT_SERVER_IP, 1883);
    return true;
}

bool loop_mqtt()
{
    return mqttClient.loop();
}

void reconnect_mqtt()
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

void publish_measurement_float(int location_id, int sensor_type, int sensor_id, const char *topic, float val)
{
    const char measurement_topic[] = "sensor/measurement";

    char val_buf[10];
    char topic_buf[KW_MQTT_MAX_TOPIC_LENGTH];

    sprintf(topic_buf, "%s/%d/%d/%d/%s", measurement_topic, location_id, sensor_type, sensor_id, topic);
    sprintf(val_buf, "%.3f", val);

    Serial.print(topic_buf);
    Serial.print(" ");
    Serial.println(val_buf);

    mqttClient.publish(topic_buf, val_buf);
}

void publish_status(int location_id, const char *topic, const char *data)
{
    const char status_topic[] = "sensor/status";

    char topic_buf[KW_MQTT_MAX_TOPIC_LENGTH];
    sprintf(topic_buf, "%s/%d/%s", status_topic, location_id, topic);

    Serial.print(topic_buf);
    Serial.print(" ");
    Serial.println(data);

    mqttClient.publish(topic_buf, data);
}
