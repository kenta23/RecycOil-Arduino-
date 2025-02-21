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
#define FLOW_SENSOR_PIN 14  

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
bool mixingCompleted = false;  // NEW FLAG: Wait for user input after mixing

// Interrupt for Flow Sensor
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
    
    digitalWrite(PUMP_ONE, HIGH);
    digitalWrite(PUMP_TWO, HIGH);
    digitalWrite(MOTOR, HIGH);
    digitalWrite(HEATER, HIGH);
    digitalWrite(SV, HIGH);

    sensors.begin();
    lcd.init();
    lcd.backlight();
    
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
    
    Serial.println("System Ready");
    lcd.print("System Ready");
}

void loop() {
    // Button 1 Press - Toggle Pump One
    if (digitalRead(BUTTON_ONE) == LOW) {
        delay(200); // Debounce
        pumpOneState = !pumpOneState; // Toggle state
        
        if (pumpOneState) {
            digitalWrite(PUMP_ONE, LOW);  // Turn ON Pump One
            Serial.println("Pump One ON - Oil is being pumped...");
        } else {
            digitalWrite(PUMP_ONE, HIGH); // Turn OFF Pump One
            Serial.println("Pump One OFF - Starting heater...");
            heaterState = true; // Start heater after Pump One stops
        }
        
        delay(300); // Prevent accidental double press
    }

    // **Wait for Mixing Completion Before Allowing Pump Two**
    if (mixingCompleted) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Mixing Done!");
        lcd.setCursor(0, 1);
        lcd.print("Press Button 2");

        Serial.println("Mixing Done! Waiting for BUTTON_TWO to start Pump Two.");

        // Only allow Pump Two after mixing is completed
        if (digitalRead(BUTTON_TWO) == LOW) {
            delay(200); // Debounce
            pumpTwoState = !pumpTwoState;

            if (pumpTwoState) {
                digitalWrite(PUMP_TWO, LOW);  // Turn ON Pump Two
                digitalWrite(SV, LOW);        // Open Solenoid
                Serial.println("Pump Two ON - Transferring biodiesel...");
            } else {
                digitalWrite(PUMP_TWO, HIGH); // Turn OFF Pump Two
                digitalWrite(SV, HIGH);       // Close Solenoid
                Serial.println("Pump Two OFF - Transfer stopped.");
            }
            
            delay(300); // Prevent accidental double press
        }
        return; // Stop execution here so LCD does not reset
    }

    // Update LCD and Serial Monitor
    lcd.clear();

    if (pumpOneState) {
        totalLiters = (pulseCount * 2.5 * 2.3) / 1000.0;
        
        lcd.setCursor(0, 0);
        lcd.print("Pumping Oil...");
        lcd.setCursor(0, 1);
        lcd.print("Flow: ");
        lcd.print(totalLiters, 2);
        lcd.print(" L");

        Serial.print("Flow Rate: ");
        Serial.print(totalLiters, 2);
        Serial.println(" L pumped");
    } 
    else if (heaterState) {
        sensors.requestTemperatures();
        float temperature = sensors.getTempCByIndex(0);
        
        lcd.setCursor(0, 0);
        lcd.print("Heating Oil...");
        lcd.setCursor(0, 1);
        lcd.print("Temp: ");
        lcd.print(temperature);
        lcd.print("C");

        Serial.print("Heating... Current Temp: ");
        Serial.print(temperature);
        Serial.println("°C");

        // Stop heater at 30°C
        if (temperature >= 30.0) {
            digitalWrite(HEATER, HIGH);
            heaterState = false;
            Serial.println("Heater OFF - Reached 30°C");

            delay(5000); // Wait 5 seconds before mixing

            digitalWrite(MOTOR, LOW); 
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Mixing Oil...");
            lcd.setCursor(0, 1);
            lcd.print("Please Wait");
            Serial.println("Mixing Oil - Motor ON for 30 sec");

            delay(30000); // Mix for 30 seconds
            
            digitalWrite(MOTOR, HIGH);
            Serial.println("Mixing Complete - Motor OFF");
            
            mixingCompleted = true;  // Enable Pump Two activation
        } else {
            digitalWrite(HEATER, LOW); 
        }
    } 
    else {
        lcd.setCursor(0, 0);
        lcd.print("System Ready...");
        lcd.setCursor(0, 1);
        lcd.print("Press a Button");

        Serial.println("System Ready - Waiting for input...");
    }

    delay(500);
}
