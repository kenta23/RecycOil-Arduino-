#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* mqttServer = "test.mosquitto.org"; // Use your MQTT broker
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// Callback function to handle received MQTT messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

   if (message == "ON") {
        Serial.println("ON"); //turn on all of the components to arduino mega
    } else if (message == "OFF") {
        Serial.println("OFF");
    }

  Serial.println("Received: " + message);
  Serial.println(message);
  
  // Send command to Arduino Mega
  Serial.println(message);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");

    if (client.connect("ESP8266Client")) {
      Serial.println("Connected!");
      client.publish("iot/machine", "OFF");
    } else {
      delay(2000);
    }
  }
}

void loop() {
  if (Serial.available()) {
    String sensorData = Serial.readStringUntil('\n');
    sensorData.trim();

    int commaIndex = receivedData.indexOf(',');

    if (commaIndex != -1) {
      String flowRate = receivedData.substring(0, commaIndex);
      String temperature = receivedData.substring(commaIndex + 1);

      Serial.println("Flow Rate: " + flowRate);
      Serial.println("Temperature: " + temperature);

      // Publish to MQTT
      client.publish("recycoil/flow", flowRate.c_str());
      client.publish("recycoil/temp", temperature.c_str());


    }

  }

  client.loop();
}
