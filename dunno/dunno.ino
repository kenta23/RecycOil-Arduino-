#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

#define TEMP_SENSOR_PIN 6
#define PUMP_PIN 3
#define HEATER_PIN 11
#define MOTOR_PIN 8
#define FLOW_SENSOR_PIN 2
#define BUTTON_PIN 10
#define LED_PIN 9

//wifi 
#define ESP8266 Serial1


LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

const float TEMP_THRESHOLD = 60.0;
const int TARGET_VOLUME = 100; // 2 liters

volatile int pulseCount = 0;
float totalPulses = 0.0;
float totalVolume = 0.0;
bool pumpRunning = false;
bool heaterOff = false;
bool motorStarted = false;
bool buttonPressed = false;
bool heatingStarted = false;

//machine state 
bool machineOn = false;


void pulseCounter() {
    pulseCount++;
    delayMicroseconds(10); // Debounce to reduce noise
}

void setup() {
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();

    pinMode(PUMP_PIN, OUTPUT);
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

    digitalWrite(PUMP_PIN, HIGH);
    digitalWrite(HEATER_PIN, HIGH);
    digitalWrite(MOTOR_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);

    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

    sensors.begin();
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    Serial.print("System Ready");
}

void loop() {
   machineState();

   if (machineOn || checkButtonPress()) {
        totalVolume = 0.0;
        totalPulses = 0.0;
        pulseCount = 0;
        pumpRunning = true;
        heatingStarted = false;

     Serial.println("Button Pressed: Starting Pump");
     readTemperature();
     controlHeater();
     controlMotor();
     controlPump();
   }
}

void machineState() {
  if(ESP8266.available()) {
        String command = Serial1.readStringUntil('\n');
        command.trim();
        //receiving data from esp8266 


        Serial.println(command);
        

        if (command == "ON") {
            machineOn = true;
            Serial.println("Machine Started");
        } else if (command == "OFF") {
            machineOn = false;
            Serial.println("Machine Stopped");
        }
  }  
}


bool checkButtonPress() {
    // if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
    //     delay(200); // Debounce
    //     buttonPressed = true;
    //     totalVolume = 0.0;
    //     totalPulses = 0.0;
    //     pulseCount = 0;
    //     pumpRunning = true;
    //     heatingStarted = false;

    //     Serial.println("Button Pressed: Starting Pump");
    //     lcd.clear();
    //     lcd.setCursor(0, 0);
    //     lcd.print("Pump ON");
    //     digitalWrite(PUMP_PIN, LOW);
    // }

    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
        delay(200);
        buttonPressed = true;
        return true;
    }

    if (digitalRead(BUTTON_PIN) == HIGH) {
        buttonPressed = false;
        return false;
    }

}

void readTemperature() {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    if (tempC != DEVICE_DISCONNECTED_C) {
        Serial.print("Temperature: ");
        ESP8266.println(tempC);
        lcd.setCursor(0, 1);
        lcd.print("Temp: ");
        lcd.print(tempC);
        lcd.print("C ");
    }
}

void controlHeater() {
    if (totalVolume >= TARGET_VOLUME && !heatingStarted) {
        digitalWrite(PUMP_PIN, HIGH); // Stop the pump
        pumpRunning = false;
        Serial.println("Pump OFF");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Pump OFF");
        
        delay(10000); // Wait 10 seconds before starting heater
        
        digitalWrite(HEATER_PIN, LOW);
        Serial.println("Heater ON");
        heatingStarted = true;
    }
    
    float tempC = sensors.getTempCByIndex(0);
    if (heatingStarted && !heaterOff && tempC >= TEMP_THRESHOLD) {
        digitalWrite(HEATER_PIN, HIGH);
        heaterOff = true;
        Serial.println("Heater OFF");
    }
}

void controlMotor() {
    if (heaterOff && !motorStarted) {
        delay(30000); // 30 sec delay after heater turns off
        digitalWrite(MOTOR_PIN, HIGH);
        Serial.println("Motor Started");
        delay(1800000); // Run for 30 min
        digitalWrite(MOTOR_PIN, LOW);
        Serial.println("Motor Stopped");
        motorStarted = true;
        digitalWrite(LED_PIN, LOW); // Turn LED on to indicate next process
    }
}

void controlPump() {
    if (pumpRunning) {
        unsigned long currentMillis = millis();
        static unsigned long previousMillis = 0;
        const unsigned long interval = 500;

        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            
            detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
            
            float flowRate = ((pulseCount * 2.5 * 2.3) / 1000.0) * 12;
            float pulsesThisCycle = pulseCount; // Store before resetting
            pulseCount = 0;
            totalPulses += pulsesThisCycle;
            totalVolume = ((totalPulses * 2.5 * 2.3) / 1000.0);
            
            attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
            
            Serial.print("Pulse Count: ");
            Serial.print(pulseCount);
            Serial.print(" | Flow Rate: ");
            ESP8266.print(flowRate, 2); //flow rate 
            Serial.print(" L/min | Total Volume: ");
            Serial.print(totalVolume, 2);
            Serial.println(" L");
            
            if (totalVolume >= TARGET_VOLUME) {
                pumpRunning = false;
                digitalWrite(PUMP_PIN, HIGH); // Stop Pump
                Serial.println("Target Volume Reached - Pump Stopped");
            }
        }
    }
}