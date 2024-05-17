#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 4                          // D2 pin of NodeMCU

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);            // Pass the oneWire reference to Dallas Temperature.

// Define the I2C address of the LCD display
#define I2C_ADDR 0x27

// Define the number of columns and rows of the LCD display
#define LCD_COLS 16
#define LCD_ROWS 2

// Initialize the LCD object with the I2C address
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLS, LCD_ROWS);

const int triggerPin = D6;
const int echoPin = D7;
const int motor1IN1 = D4; // Connected to IN1
const int motor1IN2 = D3; // Connected to IN2
const int motor2IN3 = D5; // Connected to IN3
const int motor2IN4 = D8; // Connected to IN4
bool welcomeDisplayed = false;
bool motionSensorEnabled = true;
unsigned long welcomeDisplayTime = 0;
unsigned long fingertipDisplayTime = 0;
bool readingsDisplayed = false; // Flag to track if readings have been displayed
bool followMeDisplayed = false; // Flag to track if "Follow me" has been displayed
bool motorsRotated = false; // Flag to track if motors have been rotated

MAX30105 particleSensor;
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE]; // Infrared LED sensor data
uint32_t redBuffer[BUFFER_SIZE]; // Red LED sensor data

void setup() {
  Serial.begin(115200);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motor1IN1, OUTPUT);
  pinMode(motor1IN2, OUTPUT);
  pinMode(motor2IN3, OUTPUT);
  pinMode(motor2IN4, OUTPUT);

  // Initialize the LCD display
  lcd.init();
  lcd.backlight();

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  // Setup sensor
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED

  Serial.println("Place your index finger on the sensor with steady pressure.");

  sensors.begin(); // Start the temperature sensor
}

// Function to enable motion sensor and restart all operations
void enableMotionSensorAndRestart() {
  // Enable motion sensor
  motionSensorEnabled = true;

  // Reset flags
  welcomeDisplayed = false;
  readingsDisplayed = false;
  followMeDisplayed = false;
  motorsRotated = false;

  // Clear LCD
  lcd.clear();

  // Reset timers
  welcomeDisplayTime = 0;
  fingertipDisplayTime = 0;
}

void loop() {
  if (motionSensorEnabled) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;

    if (distance < 100 && distance > 0) {
      if (!welcomeDisplayed) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("WELCOME");
        welcomeDisplayTime = millis();
        welcomeDisplayed = true;
      }
    } else {
      // Check if "WELCOME" has been displayed and 5 seconds have passed
      if (welcomeDisplayed && millis() - welcomeDisplayTime >= 10000) {
        lcd.clear();
        welcomeDisplayed = false;
        motionSensorEnabled = false; // Disable the motion sensor
        fingertipDisplayTime = millis(); // Record the time when "WELCOME" is removed
      }
    }
  }

  // Display "keep your fingertip" for 10 seconds after "WELCOME" is removed
  if (!welcomeDisplayed && !motionSensorEnabled && millis() - fingertipDisplayTime < 10000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Keep your");
    lcd.setCursor(0, 1);
    lcd.print("fingertip");
    delay(10000); // Keep "keep your fingertip" for 10 seconds
    lcd.clear(); // Clear the LCD display after 10 seconds

    // Read raw data from sensor
    bool fingertipDisplayed = false;
    while (!fingertipDisplayed) {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        while (!particleSensor.available()) {
          particleSensor.check();
        }

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
      }

      // Calculate heart rate and SpO2
      int32_t spo2;
      int8_t validSPO2;
      int32_t heartRate;
      int8_t validHeartRate;

      maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

      // Read temperature
      sensors.requestTemperatures(); // Send the command to get temperatures
      float temperature = sensors.getTempCByIndex(0); // Get the temperature in Celsius

      // Display HR, SpO2, and temperature readings on the LCD
      lcd.setCursor(0, 0);
      if (validHeartRate) {
        lcd.print("HR: ");
        lcd.print(heartRate);
        lcd.print(" bpm");
      } else {
        lcd.print("HR: -- bpm");
      }
      lcd.setCursor(0, 1);
      if (validSPO2) {
        lcd.print("SpO2:");
        lcd.print(spo2);
        lcd.print("%");
      } else {
        lcd.print("SpO2: -- %");
      }

      lcd.print("Temp:");
      lcd.print(temperature);
      lcd.print(" C");

      fingertipDisplayed = true;
    }

    readingsDisplayed = true; // Mark that readings have been displayed
    delay(10000); // Display the values for 10 seconds
    lcd.clear(); // Clear the LCD display after 10 seconds
  } else {
    // If readings are removed and readings have been displayed, display "Follow me" for 5 seconds
    if (readingsDisplayed && !followMeDisplayed) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Follow me");
      delay(5000); // Display "follow me" for 5 seconds
      lcd.clear(); // Clear the LCD display after 5 seconds
      followMeDisplayed = true; // Mark that "Follow me" has been displayed
    }

    // If "Follow me" has been displayed and motors have not yet rotated
    if (followMeDisplayed && !motorsRotated) {
      // Start both motors rotating for 10 seconds
      analogWrite(motor1IN1, 100);
      digitalWrite(motor1IN2, LOW);
      analogWrite(motor2IN3, 100);
      digitalWrite(motor2IN4, LOW);
      delay(5000); // Rotate motors for 10 seconds
      digitalWrite(motor1IN1, LOW); // Stop motor 1
      digitalWrite(motor1IN2, LOW);
      digitalWrite(motor2IN3, LOW); // Stop motor 2
      digitalWrite(motor2IN4, LOW);
      motorsRotated = true; // Mark that motors have been rotated
    }

    // If motors have been rotated and "Wait" has not yet been displayed
    else if (motorsRotated && !welcomeDisplayed) {
      // Display "Wait" for 5 seconds
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Wait");
      delay(5000); // Display "Wait" for 5 seconds
      lcd.clear(); // Clear the LCD display after 5 seconds
      welcomeDisplayed = true; // Mark that "Wait" has been displayed

      // Rotate motors again for next 10 seconds
      digitalWrite(motor1IN1, LOW);
      analogWrite(motor1IN2, 100);
      digitalWrite(motor2IN3, LOW);
      analogWrite(motor2IN4, 100);
      delay(5000); // Rotate motors for 10 seconds
      digitalWrite(motor1IN1, LOW); // Stop motor 1
      digitalWrite(motor1IN2, LOW);
      digitalWrite(motor2IN3, LOW); // Stop motor 2
      digitalWrite(motor2IN4, LOW);
    }
  }

  // After completing all operations
  if (welcomeDisplayed && motorsRotated) {
    // Call the custom function to enable motion sensor and restart
    enableMotionSensorAndRestart();

    // Restart loop by jumping to the beginning
    return;
  }
}