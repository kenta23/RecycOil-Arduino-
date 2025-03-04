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
#define PUMP_THREE 30
#define SV_2 32
#define BUTTON_ONE 12
#define BUTTON_TWO 7
#define BUTTON_THREE 32
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
bool pumpTwoState = false;
bool heaterState = false;
bool pumpThreeState = false;
unsigned long pumpStartTime = 0;
unsigned long heaterStartTime = 0;
unsigned long heaterStopTime = 0;  // ✅ Add this line
bool flowMeasurementStarted = false;
const unsigned long flowSensorDelay = 1000;  // 2.5-second delay before measurement


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
    pinMode(PUMP_THREE, OUTPUT);
    pinMode(SV_2, OUTPUT);
    pinMode(BUTTON_ONE, INPUT_PULLUP);
    pinMode(BUTTON_TWO, INPUT_PULLUP);
    pinMode(BUTTON_THREE, INPUT_PULLUP);
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

    digitalWrite(PUMP_ONE, HIGH);
    digitalWrite(PUMP_TWO, HIGH);
    digitalWrite(MOTOR, HIGH);
    digitalWrite(HEATER, HIGH);
    digitalWrite(SV, HIGH);
    digitalWrite(PUMP_THREE, HIGH);
    digitalWrite(SV_2, HIGH);

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
        while (digitalRead(BUTTON_ONE) == LOW);  // Wait for button release

        pumpOneState = !pumpOneState;  // Toggle pump state

        if (pumpOneState) {
            digitalWrite(PUMP_ONE, LOW);
            Serial.println("Pumping Oil - Waiting for Flow Stabilization");
            pumpStartTime = millis();
            flowMeasurementStarted = false;  // Reset flow measurement delay
        } else {
            digitalWrite(PUMP_ONE, HIGH);
            Serial.println("Pump One Stopped Manually");
            flowMeasurementStarted = false;  // Stop measuring flow
            pulseCount = 0;
        }
    }

    // ✅ Start flow measurement after 3-second delay
    if (pumpOneState && !flowMeasurementStarted && (millis() - pumpStartTime >= flowSensorDelay)) {
        flowMeasurementStarted = true;
        Serial.println("Flow Sensor Activated - Measuring Volume");
    }

    // ✅ Flow Measurement
    if (flowMeasurementStarted && (currentMillis - previousMillis >= interval)) {
        previousMillis = currentMillis;

        detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

        float litersPerMinute = (pulseCount * litersPerPulse) * 12;  // 🔧 Scaled up to L/min
        totalPulses += pulseCount;
        totalLiters = totalPulses * litersPerPulse;

        pulseCount = 0;
        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

        static float lastDisplayedVolume = -1;
        if (abs(totalLiters - lastDisplayedVolume) >= 0.01) {  // ✅ Only update LCD if value changes
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Volume: ");
            lcd.print(totalLiters, 2);
            lcd.print(" L");
            lastDisplayedVolume = totalLiters;
        }

        Serial.print("Flow Rate: ");
        Serial.print(litersPerMinute, 2);
        Serial.print(" L/min | Total Volume: ");
        Serial.print(totalLiters, 2);
        Serial.println(" L");

        // ✅ Stop Pump One at 2.5L and start heating
        if (totalLiters >= 0.01) {
            digitalWrite(PUMP_ONE, HIGH);
            pumpOneState = false;
            flowMeasurementStarted = false;
            Serial.println("Target Volume Reached - Pump One OFF");

            Serial.println("Waiting 10s before heating...");
            delay(10000);  // ⏳ Wait 10 seconds before starting the heater
            heaterState = true;
            digitalWrite(HEATER, LOW);
            heaterStartTime = millis();
        }
    }

    // ✅ Heater Control Logic
    if (heaterState) {
        sensors.requestTemperatures();
        float currentTemp = sensors.getTempCByIndex(0);

        Serial.print("Current Temp: ");
        Serial.println(currentTemp);

        // 🛑 If the temperature sensor fails, show an error
        if (currentTemp == -127.00) {
            Serial.println("ERROR: Temperature sensor not detected!");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Temp Sensor Err");
            return;  // Skip further processing
        }

        // ✅ Display Temperature on LCD (only update when value changes)
        static float lastDisplayedTemp = -100;
        if (abs(currentTemp - lastDisplayedTemp) >= 1000) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Heating Oil...");
            lcd.setCursor(0, 1);
            lcd.print("Temp: ");
            lcd.print(currentTemp, 1);
            lcd.print("C");
            lastDisplayedTemp = currentTemp;
        }

        // ✅ Control Heater
        if (currentTemp < 30.00) {
            digitalWrite(HEATER, LOW);  // Heater ON
            
        } else {
            digitalWrite(HEATER, HIGH);  // Heater OFF
            heaterState = false;
            heaterStopTime = millis();
            Serial.println("Heater OFF - Reached Target Temp");
        }
    }

    // ✅ Turn on PUMP_THREE and SV_2 **40 seconds after heater stops**
    if (!pumpThreeState && heaterStopTime > 0 && (millis() - heaterStopTime >= 3000)) {
        pumpThreeState = true;
        pumpStartTime = millis();  
        digitalWrite(PUMP_THREE, LOW);
        digitalWrite(SV_2, LOW);
        Serial.println("Pump Three & SV_2 ON - Running for 30 seconds");
        lcd.print("Adding Catalyst");
    }

    // ✅ Turn off PUMP_THREE and SV_2 after **30 seconds**
    if (pumpThreeState && (millis() - pumpStartTime >= 2000)) { // 30 seconds
        pumpThreeState = false;
        digitalWrite(PUMP_THREE, HIGH);
        digitalWrite(SV_2, HIGH);
        Serial.println("Pump Three & SV_2 OFF - Proceeding to Mixing");
        lcd.print("Mixing");

        delay(10000); // Small delay before mixing
        Serial.println("Starting Mixing Process");
        digitalWrite(MOTOR, LOW);

        for (int i = 1800; i > 0; i--) {  // ⏳ 30 minutes (1800 seconds)
            int minutes = i / 60;
            int seconds = i % 60;

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Mixing Oil...");
            lcd.setCursor(0, 1);
            lcd.print("Time Left: ");
            if (minutes < 10) lcd.print("0");
            lcd.print(minutes);
            lcd.print(":");
            if (seconds < 10) lcd.print("0");
            lcd.print(seconds);

            Serial.print("Mixing Time Left: ");
            if (minutes < 10) Serial.print("0");
            Serial.print(minutes);
            Serial.print(":");
            if (seconds < 10) Serial.print("0");
            Serial.println(seconds);

            delay(1000);
        } 

        digitalWrite(MOTOR, HIGH);
        Serial.println("Mixing Complete - Motor OFF");
        heaterStopTime = 0;  
    }

    // ✅ Pump Two Button
    if (digitalRead(BUTTON_TWO) == LOW) {
        delay(200);
        while (digitalRead(BUTTON_TWO) == LOW);

        pumpTwoState = !pumpTwoState;
        digitalWrite(PUMP_TWO, pumpTwoState ? LOW : HIGH);
        digitalWrite(SV, pumpTwoState ? LOW : HIGH);

        Serial.println(pumpTwoState ? "Pump Two ON - Transferring biodiesel" : "Pump Two OFF - Transfer stopped");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Transferring");

        if (!pumpTwoState) {
            delay(5000);  
            resetMachine();
        }
    }
}

// ✅ Machine Reset Function
void resetMachine() {
    pumpOneState = false;
    pumpTwoState = false;
    heaterState = false;
    pumpThreeState = false;
    flowMeasurementStarted = false;
    totalLiters = 0.0;
    totalPulses = 0.0;
    pulseCount = 0;
    heaterStopTime = 0;

    digitalWrite(PUMP_ONE, HIGH);
    digitalWrite(PUMP_TWO, HIGH);
    digitalWrite(MOTOR, HIGH);
    digitalWrite(HEATER, HIGH);
    digitalWrite(SV, HIGH);
    digitalWrite(PUMP_THREE, HIGH);
    digitalWrite(SV_2, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Machine Ready");

    Serial.println("Machine Reset - Ready for Next Batch");

    delay(2000);
    lcd.clear();
}
 
