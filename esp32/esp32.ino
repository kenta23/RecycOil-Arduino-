#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>               
#include <LiquidCrystal_I2C.h>  
#include <OneWire.h>            
#include <DallasTemperature.h>  
#include <HX711_ADC.h>

#define WIFI_SSID "PLDTHOMEFIBR7Fx93"
#define WIFI_PASSWORD "@ApolinarioFamily29"

#define MQTT_BROKER "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_USERNAME "Your_MQTT_Username"
#define MQTT_PASSWORD "Your_MQTT_Password"

#define TOPIC_START "recycoil/buttonStart"
#define TOPIC_STOP  "recycoil/buttonStop"
#define TOPIC_TEMP "recycoil/temperature"
#define TOPIC_FLOW "recycoil/flowRate"
#define TOPIC_LITERS "recycoil/liters"
#define TOPIC_STATUS "recycoil/status"
#define TOPIC_BIODIESEL "recycoil/biodiesel"
#define TOPIC_CARBONFOOTPRINT "recycoil/carbonFootprint"


#define PUMP_ONE 23    
#define PUMP_TWO 19    
#define PUMP_THREE 18  
#define HEATER 25      
#define DCMOTOR 17     
#define SV_METHANOL 16 
#define SV_BIODIESEL 26

// Buttons
#define BUTTON_ONE 14 
#define BUTTON_TWO 27 

// Flow Sensor
#define FLOW_SENSOR_PIN 13  

// DS18B20 OneWire
#define ONE_WIRE_BUS 32  

// Flow Sensor Variables
volatile int pulseCount = 0;
float flowRate;
const float pulsesPerLiter = 5880.0;
float totalLiters = 0;
unsigned long lastSensorUpdate = 0;


// Load Cell Pins
#define LOADCELL_DT 33  // Data pin (DOUT)
#define LOADCELL_SCK 4 // Clock pin (SCK)

// Create HX711 object
HX711_ADC loadCell(LOADCELL_DT, LOADCELL_SCK);

// Load Cell Variables
float weight = 0.0;
const float CALIBRATION_FACTOR = 2280.0; // Adjust based on calibration

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

WiFiClient espClient;
PubSubClient client(espClient);

// Interrupt Service Routine for Flow Sensor
void IRAM_ATTR flowISR() {  
    pulseCount++;
}

// Machine State
bool machineRunning = false;
bool heaterRunning = false;
bool heaterStoppedByTemp = false;
bool solenoidActive = false;

float targetTemp = 28.0;
float currentTemp = 0.0;

// WiFi Connection
void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
}

// MQTT Callback
void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Message received on topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    Serial.println(message);

    if (String(topic) == TOPIC_START && message == "1") {
        machineRunning = true;
    } 
    if (String(topic) == TOPIC_STOP && message == "1") {
        machineRunning = false;
    }
}

// Connect to MQTT Broker
void connectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client")) {
            Serial.println("Connected!");
            client.subscribe(TOPIC_START);
            client.subscribe(TOPIC_STOP);
        } else {
            Serial.print("Failed (");
            Serial.print(client.state());
            Serial.println("), retrying...");
            delay(2000);
        }
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(PUMP_ONE, OUTPUT);
    pinMode(PUMP_TWO, OUTPUT);
    pinMode(PUMP_THREE, OUTPUT);
    pinMode(HEATER, OUTPUT);
    pinMode(DCMOTOR, OUTPUT);
    pinMode(SV_METHANOL, OUTPUT);
    pinMode(SV_BIODIESEL, OUTPUT);

    // Set all relays OFF (HIGH if active LOW)
    digitalWrite(PUMP_ONE, HIGH);
    digitalWrite(PUMP_TWO, HIGH);
    digitalWrite(PUMP_THREE, HIGH);
    digitalWrite(HEATER, HIGH);
    digitalWrite(DCMOTOR, HIGH);
    digitalWrite(SV_METHANOL, HIGH);
    digitalWrite(SV_BIODIESEL, HIGH);

    pinMode(BUTTON_ONE, INPUT_PULLUP);
    pinMode(BUTTON_TWO, INPUT_PULLUP);
    pinMode(FLOW_SENSOR_PIN, INPUT);

    attachInterrupt(FLOW_SENSOR_PIN, flowISR, RISING);

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Machine is open!");
    sensors.begin();

    connectWiFi();
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(callback);
    connectMQTT();
}



void updateSensors() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastSensorUpdate >= 2000) {
        lastSensorUpdate = currentMillis;

        // Read Temperature
        sensors.requestTemperatures();
        currentTemp = sensors.getTempCByIndex(0);
        client.publish(TOPIC_TEMP, String(currentTemp, 2).c_str());
        Serial.print("Temperature: ");
        Serial.print(currentTemp);
        Serial.println(" °C");

        // Read Flow Rate
        noInterrupts();
        float pulses = pulseCount;
        pulseCount = 0;
        interrupts();

        flowRate = (pulses / pulsesPerLiter) * 60.0;
        totalLiters += (pulses / pulsesPerLiter);
        client.publish(TOPIC_FLOW, String(flowRate, 2).c_str());
        client.publish(TOPIC_LITERS, String(totalLiters, 2).c_str());

        // Read Load Cell (Weight)
        loadCell.update();
        weight = loadCell.getData();
        Serial.print("Weight: ");
        Serial.print(weight, 2);
        Serial.println(" g");

        client.publish("recycoil/weight", String(weight, 2).c_str());  // Publish weight to MQTT
    }
}


void runMachine() {
    unsigned long startTime = millis();  // Record start time

    if (!heaterRunning && !solenoidActive) {
        digitalWrite(PUMP_ONE, LOW);
        lcd.setCursor(0, 1);
        lcd.print("Transferring Oil");
        delay(6000);
    }

    if (flowRate < 2.00 && !heaterRunning && !heaterStoppedByTemp) {
        digitalWrite(PUMP_ONE, HIGH);
        digitalWrite(HEATER, LOW);
        heaterRunning = true;
        lcd.setCursor(0, 1);
        lcd.print("Heating oil");
    }

    if (heaterRunning && currentTemp >= targetTemp) {
        digitalWrite(HEATER, HIGH);
        heaterRunning = false;
        heaterStoppedByTemp = true;
        lcd.setCursor(0, 1);
        lcd.print("Heater Stopped");
        delay(30000);
        solenoidActive = true;
    }

    if (solenoidActive) {
        digitalWrite(SV_METHANOL, LOW);
        digitalWrite(PUMP_THREE, LOW);
        delay(30000);
        digitalWrite(SV_METHANOL, HIGH);
        digitalWrite(PUMP_THREE, HIGH);
        lcd.setCursor(0, 1);
        lcd.print("Pouring methanol");
        solenoidActive = false;
        delay(10000);
    }

     digitalWrite(DCMOTOR, LOW);
     lcd.print("Mixing oil");
     delay(20000);
     digitalWrite(DCMOTOR, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Mixing Done");

    digitalWrite(SV_BIODIESEL, LOW);
    digitalWrite(PUMP_TWO, LOW);
    delay(15000);
    digitalWrite(SV_BIODIESEL, HIGH);
    digitalWrite(PUMP_TWO, HIGH);
    machineRunning = false;

    unsigned long endTime = millis();
    unsigned long producingTime = (endTime - startTime) / 1000;  // Convert to seconds

    // Convert to hours, minutes, seconds
    int hours = producingTime / 3600;
    int minutes = (producingTime % 3600) / 60;
    int seconds = producingTime % 60;

    char formattedTime[20];
    snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d", hours, minutes, seconds);

    client.publish("recycoil/producingTime", formattedTime);
    client.publish("recycoil/status", "SUCCESSFUL");
    Serial.println("STATUS successful");
    Serial.print("Producing Time: ");
    Serial.println(formattedTime);
}


bool buttonPressed(int pin) {
    if (digitalRead(pin) == LOW) {
        delay(200); // Debounce
        return digitalRead(pin) == LOW;
    }
    return false;
}

void loop() {
    client.loop();
    updateSensors();

    if (buttonPressed(BUTTON_ONE)) {
        machineRunning = true;
    }

    if (buttonPressed(BUTTON_TWO)) {
        machineRunning = false;
    }

    if (machineRunning) {
        runMachine();
    }
}

