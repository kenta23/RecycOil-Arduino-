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
#define BUTTON_THREE 8
#define FLOW_SENSOR_PIN 2  // Flow sensor signal pin

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Temperature Sensor Setup
OneWire oneWire(52);
DallasTemperature sensors(&oneWire);

// Flow Sensor Variables
volatile int pulseCount = 0;
float totalLiters = 0.0;
float totalPulses = 0.0;
float litersPerPulse = (2.6 * 2.5) / 1000.0;  // 🔧 Adjust as needed

unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // 1 second interval

bool pumpOneState = false;
bool heaterState = false;
unsigned long pumpStartTime = 0;
bool flowMeasurementStarted = false;
const unsigned long flowSensorDelay = 2000;  // 01-second delay before measurement

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
    unsigned long currentMillis = millis();
    
    // ✅ Pump One Start/Stop Button
    if (digitalRead(BUTTON_ONE) == LOW) {
        delay(200);
        while (digitalRead(BUTTON_ONE) == LOW);

        pumpOneState = !pumpOneState;
        pulseCount = 0;
        flowMeasurementStarted = false;

        if (pumpOneState) {
            digitalWrite(PUMP_ONE, LOW);
            Serial.println("Pumping Oil - Waiting for Flow Stabilization");
            pumpStartTime = millis();
        }
    }

    // ✅ Start flow measurement after 3-second delay
    if (pumpOneState && !flowMeasurementStarted && (millis() - pumpStartTime >= flowSensorDelay)) {
        flowMeasurementStarted = true;
        Serial.println("Flow Sensor Activated - Measuring Volume");
    }

    // ✅ Flow Measurement (Accurate version)
    if (flowMeasurementStarted && (currentMillis - previousMillis >= interval)) {
        previousMillis = currentMillis;

        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

        float litersPerMinute = (pulseCount * litersPerPulse) * 12;  // 🔧 Scaled up to L/min
        totalPulses += pulseCount;
        totalLiters = totalPulses * litersPerPulse;

        pulseCount = 0;  // Reset for next measurement
        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

        // ✅ LCD Optimization (Only update when value changes)
        static float lastDisplayedVolume = -1;
        if (totalLiters != lastDisplayedVolume) { 
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Flow: ");
            lcd.print(totalLiters, 2);
            lcd.print(" L");
            lastDisplayedVolume = totalLiters;
        }

        Serial.print("Flow Rate: ");
        Serial.print(litersPerMinute, 2);
        Serial.print(" L/min | Total Volume: ");
        Serial.print(totalLiters, 2);
        Serial.println(" L");

        // ✅ Stop Pump at 1L and proceed to heating
        if (totalLiters >= 1.0) {
            digitalWrite(PUMP_ONE, HIGH);
            pumpOneState = false;
            flowMeasurementStarted = false;
            Serial.println("Target Volume Reached - Pump One OFF");
            
            Serial.println("Waiting 10s before heating...");
            delay(10000);
            heaterState = true;
        }
    }

    // ✅ Heater Logic
    if (heaterState) {
        sensors.requestTemperatures();
        float currentTemp = sensors.getTempCByIndex(0);

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

        if (currentTemp < 35.0) {
            digitalWrite(HEATER, LOW);
        } else {
            digitalWrite(HEATER, HIGH);
            heaterState = false;
            Serial.println("Heater OFF - Reached Target Temp");

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
        }
    }

    // ✅ Pump Two Button
    if (digitalRead(BUTTON_TWO) == LOW) {
        delay(200);
        while (digitalRead(BUTTON_TWO) == LOW);
        
        bool pumpTwoState = digitalRead(PUMP_TWO) == HIGH;
        digitalWrite(PUMP_TWO, pumpTwoState ? LOW : HIGH);
        digitalWrite(SV, pumpTwoState ? LOW : HIGH);

        Serial.println(pumpTwoState ? "Pump Two ON - Transferring biodiesel" : "Pump Two OFF - Transfer stopped");
    }
}
