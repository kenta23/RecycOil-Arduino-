#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>               
#include <LiquidCrystal_I2C.h>  
#include <OneWire.h>            
#include <DallasTemperature.h>  
#include <HX711_ADC.h>

#define WIFI_SSID "PLDTHOMEFIBR7Fx93"
#define WIFI_PASSWORD "@ApolinarioFamily29"

#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT 1883 //or 1883 if using ssl

#define TOPIC_START "recycoil/buttonStart"
#define TOPIC_TEMP "recycoil/temperature"
#define TOPIC_FLOW "recycoil/flowRate"
#define TOPIC_LITERS "recycoil/liters"
#define TOPIC_STATUS "recycoil/status"
#define TOPIC_BIODIESEL "recycoil/biodiesel"
#define TOPIC_CARBONFOOTPRINT "recycoil/carbonFootprint"
#define TOPIC_ENERGYCONSUMPTION "recycoil/energyConsumption"


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
bool machineRunning = false;
bool heaterRunning = false;
bool heaterStoppedByTemp = false;
bool solenoidActive = false;

float targetTemp = 28.0;
float currentTemp = 0.0;


bool runMotor = false;
bool lastStep = false;
bool finished = false;

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

String lastMessage = "";  // Global variable to track last displayed message

void updateLCD(const String& message) {
    if (lastMessage != message) {  // Only update if the message is different
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(message);
        lastMessage = message;
        
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Callback triggered!");

    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("Received topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    Serial.println(message);

    if (String(topic) == TOPIC_START && message == "true") {
        Serial.println("MQTT Start command received!");
        machineRunning = true;
    }
}


// Connect to MQTT Broker
void connectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client")) {
            Serial.println("Connected!");
            client.subscribe(TOPIC_START);
            client.subscribe(TOPIC_TEMP);
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
        client.publish(TOPIC_TEMP, String(currentTemp, 2).c_str());  //publish temp data 
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
        client.publish(TOPIC_LITERS, String(weight, 2).c_str());  // Publish oil volume to MQTT
    }
}



void runMachine() {
    unsigned long startTime = millis();  // Record start time
    lcd.clear();

    static unsigned long stepStartTime = 0;
    static int step = 0;
  
    if (digitalRead(PUMP_ONE) == HIGH) {
        digitalWrite(PUMP_ONE, LOW);
        updateLCD("Transferring Oil");
        delay(6000);
    }

    if (!heaterRunning && !heaterStoppedByTemp) {
        heaterRunning = true;
        
        if (heaterRunning) { 
           digitalWrite(PUMP_ONE, HIGH); //off the pump one
           digitalWrite(HEATER, LOW);  //turn on the heater

        }
        updateLCD("Heating oil");
    }



    if (heaterRunning && currentTemp >= targetTemp) {
        updateLCD("Heater Stopped");
        digitalWrite(HEATER, HIGH); //off the heater
        heaterRunning = false;
        heaterStoppedByTemp = true;
        delay(30000); //after 30 seconds
        solenoidActive = true;
    }


    if (solenoidActive && heaterStoppedByTemp) {
        updateLCD("Pouring methanol");
        digitalWrite(SV_METHANOL, LOW); //turn on
        digitalWrite(PUMP_THREE, LOW); //turn on

        delay(30000); //after 300000 ms
        digitalWrite(SV_METHANOL, HIGH);
        digitalWrite(PUMP_THREE, HIGH);
        updateLCD("Stopped");
        solenoidActive = false;
        delay(10000); //delay 10 seconds for the next step
        runMotor = true;
    }

    if(!solenoidActive && runMotor) { 
       digitalWrite(DCMOTOR, LOW);
       updateLCD("Mixing oil");
       delay(20000);

       digitalWrite(DCMOTOR, HIGH);
       updateLCD("Mixing done");
       delay(3000);
       runMotor = false;
       lastStep = true;
    }
  

    if(!runMotor && lastStep) { 
        digitalWrite(SV_BIODIESEL, LOW);
        digitalWrite(PUMP_TWO, LOW);
        delay(15000); //15 seconds 
        digitalWrite(SV_BIODIESEL, HIGH);
        digitalWrite(PUMP_TWO, HIGH);

        delay(2000);
        finished = true;
    }

   if(finished) { 
      //stop all the components
      digitalWrite(PUMP_ONE, HIGH);
      digitalWrite(PUMP_TWO, HIGH);
      digitalWrite(PUMP_THREE, HIGH);
      digitalWrite(HEATER, HIGH);
      digitalWrite(DCMOTOR, HIGH);
      digitalWrite(SV_METHANOL, HIGH);
      digitalWrite(SV_BIODIESEL, HIGH);

      unsigned long endTime = millis();
      unsigned long producingTime = (endTime - startTime) / 1000;  // Convert to seconds

      // Convert producingTime to a string then publish
    client.publish("recycoil/producingTime", String(producingTime).c_str());

     //calculate the saved Co2 
    float carbonfootprint = weight * 2.7 * 0.8; //CO₂ Saved = Liters of Biodiesel Produced × CO₂ Emission Factor of Diesel × Emission Reduction Factor
     // Convert carbonfootprint (float) to a string
    client.publish(TOPIC_CARBONFOOTPRINT, String(carbonfootprint, 2).c_str());

    //energy consumption
    float timeHours = producingTime / (1000.0 * 3600.0);     // Convert milliseconds to hours
    float voltage = 12.0;  // Supply Voltage in Volts
    float current = 10.0;   // Current in Amps
    float energyConsumption = voltage * current * timeHours;  // E = V * A * (producingTime / 1000)
    client.publish(TOPIC_ENERGYCONSUMPTION, String(energyConsumption, 2).c_str());

    Serial.println("STATUS successful");
    client.publish("recycoil/status", "SUCCESSFUL");
    Serial.print("Producing Time:");
    Serial.println(producingTime);

    updateLCD("Done!");
     delay(1000);
       finished = false;
       machineRunning = false;

        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Machine is ready");
     }   

}

bool buttonPressed(int pin) {
    if (digitalRead(pin) == LOW) {
        delay(200); // Debounce
        Serial.println("Button one pressed");
        return digitalRead(pin) == LOW;
    }
    return false;
}

void loop() {
   updateSensors();

    if (buttonPressed(BUTTON_ONE)) {
        machineRunning = true;
    }

    if (buttonPressed(BUTTON_TWO)) {
        machineRunning = false;
        // lastStep = false;
        finished = false;
    }

    if (machineRunning) {
        runMachine();
    }


    client.loop();

    client.publish("recycoil/temperature", "34.5");
    delay(1000);
}