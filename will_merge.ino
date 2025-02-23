#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin Definitions
#define PUMP_ONE 36
#define PUMP_TWO 22 
#define MOTOR 26
#define HEATER 24
#define SV 28
#define BUTTON_ONE 12
#define BUTTON_TWO 7
#define BUTTON_THREE 8 // New button for additional control
#define FLOW_SENSOR_PIN 2  // Flow sensor signal pin

// LCD Setup (Assuming I2C LCD at 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Temperature Sensor Setup
OneWire oneWire(52);
DallasTemperature sensors(&oneWire);

// Flow Sensor Variables
volatile int pulseCount = 0;
float totalLiters = 0.0;
bool pumpOneState = false;
bool pumpTwoState = false;
bool heaterState = false;
bool mixingCompleted = false;
unsigned long heaterStartTime = 0;
bool heaterStarted = false;

// MPC Variables for Heater
float targetTemp = 35.0;
float currentTemp = 0.0;
unsigned long lastHeaterUpdate = 0;
const unsigned long heaterInterval = 5000; // 5 sec update interval

void pulseCounter() {
    pulseCount++;
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
    
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
    
    Serial.println("Machine Ready");
    lcd.print("Machine Ready");
}

void loop() {
    if (digitalRead(BUTTON_ONE) == LOW) {
        pumpOneState = !pumpOneState;
        pulseCount = 0;
        
        if (pumpOneState) {
            digitalWrite(PUMP_ONE, LOW);
            Serial.println("Pumping Oil - Measuring Volume");
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

    if (pumpOneState) {
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
            lcd.print("Temp: ");
            lcd.print(currentTemp);
            lcd.print("C");

            Serial.print("Current Temp: ");
            Serial.print(currentTemp);
            Serial.println(" C");

            float controlSignal = targetTemp - currentTemp;
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
