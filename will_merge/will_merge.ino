#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>



// Pin Definitions
#define PUMP_ONE 8
#define PUMP_TWO 22 
#define MOTOR 26
#define HEATER 24
#define SV 28
#define BUTTON_ONE 12 //pump one
#define BUTTON_TWO 7
#define BUTTON_THREE 8 // New button for additional control
#define FLOW_SENSOR_PIN 2  // Flow sensor signal pin



//machine state
bool machineStatus = false;

// LCD Setup (Assuming I2C LCD at 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

//struct for flow rate and 
struct OilFLowTemp {
  float oil;
  float temp;
}


/*void printFlowTemp(OilFlowTemp data) {
   //pass the data to esp32 
   Serial.println(data.oil);
   //Serial1 Esp32
   Serial1.println(data.oil);

   Serial1.print(data.temp);
} */


// Temperature Sensor Setup
OneWire oneWire(52);
DallasTemperature sensors(oneWire);

// Flow Sensor Variables
volatile int pulseCount = 0;
float totalLiters = 0.0;
bool pumpOneState = false;
bool pumpTwoState = false;
bool heaterState = false;
bool mixingCompleted = false;
unsigned long heaterStartTime = 0;
bool heaterStarted = false;
unsigned long pumpStartTime = 0;
bool flowMeasurementStarted = false;
const unsigned long flowSensorDelay = 3000; // Delay before measuring flow (3 seconds)

// MPC Variables for Heater
float targetTemp = 35.0;
float currentTemp = 0.0;
unsigned long lastHeaterUpdate = 0;
const unsigned long heaterInterval = 5000; // 5 sec update interval

void pulseCounter() {
    if (flowMeasurementStarted) {
        pulseCount++;
    }
}


//WIFI AND PUBSUBCLIENT

// ✅ WiFi Credentials
const char* ssid = "PLDTHOMEFIBR7Fx93";
const char* password = "@ApolinarioFamily29";

// ✅ MQTT Broker Details
const char* mqttServer = "broker.hivemq.com";  // Use your broker's IP/URL
const int mqttPort = 1883;  // Use 8883 for SSL (if needed)
const char* mqttTopicPub = "recycoil/test";  // Topic for publishing
const char* mqttTopicSub = "recycoil/command";  // Topic for subscribing

WiFiClient espClient;
PubSubClient client(espClient);


// 📌 Callback function when an MQTT message is received
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Received message on topic: ");
    Serial.println(topic);

    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("Message: ");
    Serial.println(message);

    // ✅ Example: If "ON" is received, do something
    if (message == "ON") {
        Serial.println("Turning ON LED!");
        digitalWrite(2, HIGH);  // Example: Turn ON an LED on GPIO 2
    }
    if (message == "OFF") {
        Serial.println("Turning OFF LED!");
        digitalWrite(2, LOW);  // Turn OFF LED
    }
}

// 📌 Connect to WiFi
void setupWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi!");
}


void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client")) {  // Set a unique client ID
            Serial.println("Connected!");
            client.subscribe(mqttTopicSub);  // Subscribe to a topic
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Trying again in 5s...");
            delay(5000);
        }
    }
}


void setup() {
    Serial.begin(9600);
    
    pinMode(PUMP_ONE, OUTPUT);
    pinMode(PUMP_TWO, OUTPUT);
    pinMode(MOTOR, OUTPUT);
    pinMode(HEATER, OUTPUT);
    pinMode(SV, OUTPUT);
    pinMode(BUTTON_ONE, INPUT_PULLUP);
    pinMode(BUTTON_TWO, INPUT_PULLUP);
    pinMode(BUTTON_THREE, INPUT_PULLUP);
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    
    digitalWrite(PUMP_ONE, HIGH);
    digitalWrite(PUMP_TWO, HIGH);
    digitalWrite(MOTOR, HIGH);
    digitalWrite(HEATER, HIGH);
    digitalWrite(SV, HIGH);

    sensors.begin();
    lcd.init();
    lcd.backlight(); 


    // setupWiFi();
    // client.setServer(mqttServer, mqttPort);
    // client.setCallback(callback);  // Set function to handle received messages 
    
    // attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
    
    Serial.println("Machine Ready");
    lcd.print("Machine Ready");
}



void loop() {
  //  if (!client.connected()) {
  //       reconnectMQTT();
  //   }
  //   client.loop();  // Keep MQTT connection alive

    // ✅ Example: Publish a message every 5 seconds
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 5000) {
        lastMsg = millis();
        String payload = "Hello from ESP32! Time: " + String(millis());
        client.publish(mqttTopicPub, payload.c_str());
        Serial.println("Published: " + payload);
    }

    if (digitalRead(BUTTON_ONE) == LOW) {
        pumpOneState = !pumpOneState;
        pulseCount = 0;
        flowMeasurementStarted = false;
        
        if (pumpOneState) {
            digitalWrite(PUMP_ONE, LOW);
            Serial.println("Pumping Oil - Waiting for Flow Stabilization");
            pumpStartTime = millis(); // Start time when pump turns on
        } else {
            digitalWrite(PUMP_ONE, HIGH);
            Serial.println("Pump One OFF - Waiting 10s before heating");
            delay(10000);
            heaterState = true;
            heaterStartTime = millis();
            heaterStarted = true;
        }
        delay(300);
    }

    // Start flow measurement after initial delay
    if (pumpOneState && !flowMeasurementStarted && (millis() - pumpStartTime >= flowSensorDelay)) {
        flowMeasurementStarted = true;
        Serial.println("Flow Sensor Activated - Measuring Volume");
    }

    if (flowMeasurementStarted) {
        totalLiters = ((pulseCount * 2.0) * 1.8) / 1000.0; // Adjusted calibration
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Flow: ");
        lcd.print(totalLiters, 2);
        lcd.print(" L");
        Serial.print("Total Volume Transferred: ");
        Serial.print(totalLiters, 2);
        Serial.println(" L");
    }

    if (heaterState) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastHeaterUpdate >= heaterInterval) {
            lastHeaterUpdate = currentMillis;
            sensors.requestTemperatures();
            currentTemp = sensors.getTempCByIndex(0);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Heating Oil...");
            lcd.setCursor(0, 1);
            // lcd.print("Temp: ");
            // lcd.print(currentTemp);
            lcd.print("C");

            Serial.print("Current Temp: ");
            Serial.print(currentTemp);
            Serial.println(" C");

            // float controlSignal = targetTemp - currentTemp;

            if (controlSignal > 1.0) {
                digitalWrite(HEATER, LOW);
            } else {
                digitalWrite(HEATER, HIGH);
                heaterState = false;
                Serial.println("Heater OFF - Reached Target Temp");
                unsigned long heatingDuration = (millis() - heaterStartTime) / 60000;
                Serial.print("Heating Duration: ");
                Serial.print(heatingDuration);
                Serial.println(" min");
                delay(20000);
                Serial.println("Starting Mixing Process");
                digitalWrite(MOTOR, LOW);
                for (int i = 20; i > 0; i--) {
                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("Mixing Oil...");
                    lcd.setCursor(0, 1);
                    lcd.print("Time Left: ");
                    lcd.print(i);
                    lcd.print(" sec");
                    Serial.print("Mixing Time Left: ");
                    Serial.println(i);
                    delay(1000);
                }
                digitalWrite(MOTOR, HIGH);
                Serial.println("Mixing Complete - Motor OFF");
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Process Complete");
                mixingCompleted = true;
            }
        }
    }

    if (digitalRead(BUTTON_TWO) == LOW) {
        pumpTwoState = !pumpTwoState;
        if (pumpTwoState) {
            digitalWrite(PUMP_TWO, LOW);
            digitalWrite(SV, LOW);
            Serial.println("Pump Two ON - Transferring biodiesel");
            pulseCount = 0;
        } else {
            digitalWrite(PUMP_TWO, HIGH);
            digitalWrite(SV, HIGH);
            Serial.println("Pump Two OFF - Transfer stopped");
            Serial.println("Process Complete");
        }
        delay(300);
    } 
    delay(1000); 
}
