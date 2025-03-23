#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>               
#include <LiquidCrystal_I2C.h>  
#include <OneWire.h>            
#include <DallasTemperature.h>  
#include <HX711_ADC.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>


#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

#define WIFI_SSID "UCC-WIFI"
#define WIFI_PASSWORD ""

#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT 1883 //or 1883 if using ssl

#define TOPIC_START "recycoil/buttonStart"
#define TOPIC_TEMP "recycoil/temperature"
#define TOPIC_FLOWRATE "recycoil/flowRate"
#define TOPIC_LITERS "recycoil/liters"
#define TOPIC_STATUS "recycoil/status"
#define TOPIC_BIODIESEL "recycoil/biodiesel"
#define TOPIC_CARBONFOOTPRINT "recycoil/carbonFootprint"
#define TOPIC_ENERGYCONSUMPTION "recycoil/energyConsumption"
#define TOPIC_PRODUCINGTIME "recycoil/producingTime"


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
#define FLOW_SENSOR_PIN 34  //changed from 13

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
bool machineRunning = true;
bool heaterRunning = false;
bool heaterStoppedByTemp = false;
bool solenoidActive = false;

float targetTemp = 28;
float currentTemp = 0.0;


bool runMotor = false;
bool lastStep = false;
bool finished = false;


enum MachineState {
    IDLE, TRANSFERRING_OIL, HEATING, POURING_METHANOL, MIXING, PRODUCING_BIODIESEL, DONE
};
MachineState machineState = IDLE;

//bluetooth
// class MyServerCallbacks : public BLEServerCallbacks {
//     void onConnect(BLEServer* pServer) {
//         deviceConnected = true;
//         Serial.println(F("✅ BLE Device Connected!"));
//     }

//     void onDisconnect(BLEServer* pServer) {
//         deviceConnected = false;
//         Serial.println(F("🔴 BLE Device Disconnected!"));
//         BLEAdvertising *pAdvertising = pServer->getAdvertising();
//         pAdvertising->start();  // Restart advertising
//     }
// };

// WiFi Connection
void connectWiFi() {
     const unsigned int maxAttempts = 5;
    unsigned int attempt = 0;

    Serial.print("Connecting to WiFi...");
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
     while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts) {
        delay(1000);
        Serial.print(".");
        attempt++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi after 5 attempts.");
    }
}

String lastMessage = "";  // Global variable to track last displayed message

void updateLCD(const String& message) {
    if (lastMessage != message) {  // Only update if the message is different
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(message);
        lastMessage = message;
        
    }
}

void reconnect() {
    static unsigned long lastReconnectAttempt = 0;
    unsigned long now = millis();

    if (!client.connected() && (now - lastReconnectAttempt > 500)) {  // Retry every 1 second
        lastReconnectAttempt = now;
        Serial.print(F("Attempting MQTT connection..."));
        
        if (client.connect("ESP32_Client")) {
            Serial.println(F("Connected to MQTT!"));
            client.subscribe(TOPIC_START);
            client.subscribe(TOPIC_TEMP);
            client.subscribe(TOPIC_ENERGYCONSUMPTION);
            client.subscribe(TOPIC_BIODIESEL);
            client.subscribe(TOPIC_FLOWRATE);
            client.subscribe(TOPIC_PRODUCINGTIME);
        } else {
            Serial.print(F("Failed, rc="));
            Serial.println(client.state());
        }
    }
}


void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Callback triggered!");

    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print(F("Received topic: "));
    Serial.println(topic);
    Serial.print(F("Message: "));
    Serial.println(message);

    
    if (String(topic) == TOPIC_START && message == "true") {
       client.publish("recycoil/machineStatus", "ready", true);
        machineRunning = true;
        machineState = TRANSFERRING_OIL;
    }
}

void resetMachine() {
    heaterRunning = false;
    heaterStoppedByTemp = false;
    solenoidActive = false;
    runMotor = false;
    lastStep = false;
    finished = false;
    machineRunning = false;
    machineState = IDLE;  // Reset state to IDLE

    digitalWrite(PUMP_ONE, HIGH);
    digitalWrite(PUMP_TWO, HIGH);
    digitalWrite(PUMP_THREE, HIGH);
    digitalWrite(HEATER, HIGH);
    digitalWrite(SV_METHANOL, HIGH);
    digitalWrite(SV_BIODIESEL, HIGH);
    digitalWrite(DCMOTOR, HIGH);

    updateLCD("Machine is ready");  
    Serial.println(F("Machine Reset Complete!"));
}


// Connect to MQTT Broker
void connectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client")) {
            Serial.println("Connected!");
            client.subscribe(TOPIC_START);
            client.subscribe(TOPIC_TEMP);
            client.subscribe("recycoil/energyConsumption");
        } else {
            Serial.print(F("Failed ("));
            Serial.print(F(client.state()));
            Serial.println("), retrying...");
            delay(2000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.setSleep(true);

    //   // Initialize BLE
    // BLEDevice::init("ESP32");

    // // Create BLE Server
    // BLEServer *pServer = BLEDevice::createServer();
    // pServer->setCallbacks(new MyServerCallbacks());

    // // Create BLE Service
    // BLEService *pService = pServer->createService(SERVICE_UUID);

    // // Create BLE Characteristic
    // pCharacteristic = pService->createCharacteristic(
    //                     CHARACTERISTIC_UUID,
    //                     BLECharacteristic::PROPERTY_READ   |
    //                     BLECharacteristic::PROPERTY_WRITE  |
    //                     BLECharacteristic::PROPERTY_NOTIFY 
    //                 );

    // // Set initial value
    // pCharacteristic->setValue("Hello from ESP32!");
    // // Start Service
    // pService->start();

    // // Start Advertising
    // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    // pAdvertising->addServiceUUID(SERVICE_UUID);
    // pAdvertising->setScanResponse(true);
    // pAdvertising->setMinPreferred(0x06);
    // BLEDevice::startAdvertising();

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


    loadCell.begin();
    loadCell.start(2000); 

    
    if (loadCell.getTareTimeoutFlag()) {
        Serial.println(F("Tare timeout, check wiring!"));
    } else {
        loadCell.setCalFactor(2280.0); // Adjust based on calibration
        Serial.println("Load Cell Ready!");
    }


    pinMode(BUTTON_ONE, INPUT_PULLUP);
    pinMode(BUTTON_TWO, INPUT_PULLUP);
    pinMode(FLOW_SENSOR_PIN, INPUT);

    attachInterrupt(FLOW_SENSOR_PIN, flowISR, RISING);

    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Machine is open!");
    sensors.begin();

    // connectWiFi();
    // client.setServer(MQTT_BROKER, MQTT_PORT);
    // client.setCallback(callback);
    // connectMQTT();

}

// Track producing time
unsigned long startTime = 0;
unsigned long producingTime = 0;

// Power consumption constants
const float voltage = 12.0;  // Example voltage (update as needed)
const float current = 10.0;  // Example current (update as needed)

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
        client.publish(TOPIC_FLOWRATE, String(flowRate, 2).c_str());

        // Read Load Cell (Weight)
        weight = loadCell.getData();
        Serial.print("Weight: ");
        Serial.print(weight, 2);
        Serial.println(" g");

        //publish biodiesel
        client.publish(TOPIC_BIODIESEL, String(flowRate, 2).c_str());

        // ✅ Publish Producing Time in Real-Time
        if (machineRunning) {
            producingTime = (millis() - startTime) / 1000;
            client.publish("recycoil/producingTime", String(producingTime).c_str());

            // ✅ Publish Real-Time Energy Consumption
            float timeHours = producingTime / 3600.0;
            float energyConsumption = voltage * current * timeHours;
            client.publish(TOPIC_ENERGYCONSUMPTION, String(energyConsumption, 2).c_str());
            Serial.print("Energy Consumption: ");
            Serial.println(energyConsumption);

            // ✅ Publish Real-Time CO₂ Savings
            float carbonfootprint = flowRate * 2.7 * 0.8;
            client.publish(TOPIC_CARBONFOOTPRINT, String(carbonfootprint, 2).c_str());
        }
    }
}


void runMachine() {
    static unsigned long previousMillis = 0;
    static bool processStarted = false;
    static unsigned long startTime = 0; // Track process start time

    unsigned long currentMillis = millis();

    switch (machineState) {
        case TRANSFERRING_OIL:
            if (!processStarted) {
                updateLCD("Transferring Oil");
                Serial.println("Transferring oil");
                digitalWrite(PUMP_ONE, LOW);
                startTime = millis();
                previousMillis = currentMillis;
                processStarted = true;
            }
            if (currentMillis - previousMillis >= 10000) {
                digitalWrite(PUMP_ONE, HIGH);
                machineState = HEATING;
                processStarted = false;  // Reset for the next state
            }
            break;

        case HEATING:
            if (!heaterRunning && !heaterStoppedByTemp) {
                heaterRunning = true;
                digitalWrite(HEATER, LOW);
                updateLCD("Heating Oil");
                Serial.println("Heating oil");
            }

            if (currentTemp > targetTemp) {
                digitalWrite(HEATER, HIGH);
                Serial.println("Heater Stopped");
                updateLCD("Heater Stopped");
                heaterRunning = false;
                heaterStoppedByTemp = true;
                machineState = POURING_METHANOL;
            }
            break;

        case POURING_METHANOL:
            if (!processStarted) {
                updateLCD("Adding Catalysts");
                Serial.println("Adding Catalysts");
                digitalWrite(SV_METHANOL, LOW);
                digitalWrite(PUMP_THREE, LOW);
                previousMillis = currentMillis;
                processStarted = true;
            }
            if (currentMillis - previousMillis >= 30000) {
                digitalWrite(SV_METHANOL, HIGH);
                digitalWrite(PUMP_THREE, HIGH);
                machineState = MIXING;
                processStarted = false;
            }
            break;

        case MIXING:
            if (!processStarted) {
                updateLCD("Mixing Oil");
                Serial.println("Mixing oil");
                digitalWrite(DCMOTOR, LOW);
                previousMillis = currentMillis;
                processStarted = true;
            }
            if (currentMillis - previousMillis >= 20000) {
                digitalWrite(DCMOTOR, HIGH);
                updateLCD("Mixing Done");
                Serial.println("Mixing done");
                machineState = PRODUCING_BIODIESEL;
                processStarted = false;
            }
            break;

        case PRODUCING_BIODIESEL:
            if (!processStarted) {
                updateLCD("Producing Biodiesel");
                Serial.println(F("Producing Biodiesel"));
                digitalWrite(SV_BIODIESEL, LOW);
                digitalWrite(PUMP_TWO, LOW);
                previousMillis = currentMillis;
                processStarted = true;
            }
            if (currentMillis - previousMillis >= 15000) {
                digitalWrite(SV_BIODIESEL, HIGH);
                digitalWrite(PUMP_TWO, HIGH);
                machineState = DONE;
                processStarted = false;
            }
            break;

        case DONE:
            if (!processStarted) {
                updateLCD("Finished!");
                Serial.println(F("STATUS successful"));
            

                unsigned long endTime = millis();
                unsigned long producingTime = (endTime - startTime) / 1000;


                Serial.print(F("Producing Time: "));
                Serial.println(producingTime);
                client.publish("recycoil/producingTime", String(producingTime).c_str());

                float carbonfootprint = weight * 2.7 * 0.8; 
                client.publish(TOPIC_CARBONFOOTPRINT, String(carbonfootprint, 2).c_str());

                client.publish(TOPIC_BIODIESEL, String(weight, 2).c_str());
                client.publish("recycoil/machineStatus", "idle");

                float timeHours = producingTime / 3600.0;
                float voltage = 12.0;  
                float current = 10.0;   
                float energyConsumption = voltage * current * timeHours;  
                client.publish(TOPIC_ENERGYCONSUMPTION, String(energyConsumption, 2).c_str());

                Serial.print(F("Energy Consumption: "));
                Serial.println(energyConsumption);

                client.publish("recycoil/status", "SUCCESSFUL");

                processStarted = true;
            }

            processStarted = false;
            resetMachine();  
            break;

        default:
            break;
    }
}

bool buttonPressed(int pin) {
    if (digitalRead(pin) == LOW) {
        delay(200); // Debounce
        Serial.println(F("Button one pressed"));
        return digitalRead(pin) == LOW;
    }
    return false;
}

void loop() {
    updateSensors();  

   if (deviceConnected) {
        String message = "Data from ESP32: " + String(millis() / 1000) + " sec";
        pCharacteristic->setValue(message.c_str());
        pCharacteristic->notify();  // Send data to the app
        Serial.println("📡 Sent: " + message);
        delay(5000);
    }

    if (buttonPressed(BUTTON_ONE)) {
        machineRunning = true;
        machineState = TRANSFERRING_OIL;
    }

    if (buttonPressed(BUTTON_TWO)) {
        machineRunning = false;
        resetMachine();
    }

    if (machineRunning) {
        runMachine();  
    }

    if (!client.connected()) {
         reconnect();
     }

    client.loop(); // Ensures MQTT messages are processed
}

