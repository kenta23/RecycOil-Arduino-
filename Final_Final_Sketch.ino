#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin definitions
#define ONE_WIRE_BUS 2
#define BUTTON_START 3
#define RELAY_PUMP 22
#define RELAY_MOTOR 23
#define RELAY_HEATER 24

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Temperature sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variables
float temperatureC;
bool processRunning = false;
#define ESP8266 Serial1

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  ESP8266.begin(9600); //CHANGE THIS TO 115200 IF THIS DOESNT WORK 

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize temperature sensor
  sensors.begin();

  // Pin modes
  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_MOTOR, OUTPUT);
  pinMode(RELAY_HEATER, OUTPUT);
  pinMode(BUTTON_START, INPUT_PULLUP);

  // Ensure relays are off at startup
  digitalWrite(RELAY_PUMP, HIGH);
  digitalWrite(RELAY_MOTOR, HIGH);
  digitalWrite(RELAY_HEATER, HIGH);

  // Display startup message
  Serial.println("Machine is Ready");
  lcd.setCursor(0, 0);
  lcd.print("Machine is Ready");
}

void loop() {
  // Start button: Begin process (active LOW)
  if (digitalRead(BUTTON_START) == LOW && !processRunning) {
    delay(50); // Debounce delay
    if (digitalRead(BUTTON_START) == LOW) { // Check again to confirm press
      processRunning = true;

      // Step 1: Start pump for 30 seconds
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Pump ON (30s)");
      Serial.println("Pump ON (30s)");
      digitalWrite(RELAY_PUMP, LOW);
      delay(30000);
      digitalWrite(RELAY_PUMP, HIGH);
      Serial.println("Pump Stopped");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Pump Stopped");

      // Step 2: Pause for 5 seconds
      delay(5000);

      // Step 3: Turn on heater until temperature reaches 60C
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Heating...");
      Serial.println("Heating...");
      digitalWrite(RELAY_HEATER, LOW);
      while (true) {
        sensors.requestTemperatures();
        temperatureC = sensors.getTempCByIndex(0);
        lcd.setCursor(0, 1);
        lcd.print("Temp: ");
        lcd.print(temperatureC);
        lcd.print((char)223);
        lcd.print("C ");
        Serial.print("Temperature: ");
        Serial.print(temperatureC);
        Serial.println(" C");

        if (temperatureC >= 35.0) {
          break;
        }
        delay(1000);
      }
      digitalWrite(RELAY_HEATER, HIGH);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Heating Done!");
      Serial.println("Heating Done!");

      // Step 4: Pause for 10 seconds
      delay(10000);

      // Step 5: Start DC motor for 30 minutes
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Mixing...");
      Serial.println("Mixing...");
      digitalWrite(RELAY_MOTOR, LOW);
      countdown(30000); // 30 minutes countdown
      digitalWrite(RELAY_MOTOR, HIGH);

      // Process finished
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Process Complete"); 
      Serial.println("Process Complete");
      processRunning = false;
    }
  }
}

void 

void countdown(int minutes) {
  for (int i = minutes; i > 0; i--) {
    lcd.setCursor(0, 1);
    lcd.print("Countdown: ");
    lcd.print(i);
    lcd.print(" min ");
    Serial.print("Countdown: ");
    Serial.print(i);
    Serial.println(" min");
    delay(1000);
  }
}
