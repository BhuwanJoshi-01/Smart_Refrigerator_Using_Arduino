#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Goal 12: Ensure Sustainable Consumption and Production Patterns.
// responsible consumption and production

// Pin Definitions
#define TMP36_PIN A0
#define BUZZER_PIN 8
#define ULTRASONIC_TRIG 9
#define ULTRASONIC_ECHO 10
#define PIR_PIN 2
#define GAS_SENSOR_PIN A1
#define FORCE1_PIN A2  // Daily Usage Food (Egg/Milk)
#define FORCE2_PIN A3  // Vegetables
#define SERVO_DOOR_PIN 3
#define SERVO_COOLING_PIN 4 // New servo for cooling switch

// LCD Setup
LiquidCrystal_I2C lcd(0x20, 16, 2);

// Servo Setup
Servo doorServo;
Servo coolingServo;

// Variables
float temperature;
long duration;
int distance;
int pirMotion;
int gasLevel;
float weightFood1, weightFood2;
bool doorOpenTimerActive = false;
unsigned long doorOpenStartTime = 0;

// Thresholds
#define TEMP_LOW -5
#define TEMP_HIGH 10
#define DOOR_DISTANCE_THRESHOLD 10
#define GAS_THRESHOLD 500
#define FORCE_THRESHOLD 2.0 // Assume 2N as low stock threshold
#define DOOR_OPEN_DELAY 15000 // 15 seconds

void setup() {
  // Initialize LCD and Servo
  lcd.init();
  lcd.backlight();
  doorServo.attach(SERVO_DOOR_PIN);
  coolingServo.attach(SERVO_COOLING_PIN);
  doorServo.write(0); // Ensure door is closed initially
  coolingServo.write(0); // Ensure cooling switch is off initially

  // Pin Modes
  pinMode(TMP36_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(GAS_SENSOR_PIN, INPUT);
  pinMode(FORCE1_PIN, INPUT);
  pinMode(FORCE2_PIN, INPUT);

  // Welcome Message
  lcd.setCursor(0, 0);
  lcd.print("Smart ");
  lcd.setCursor(0, 1);
  lcd.print("Refrigerator");
  delay(2000);
  lcd.clear();
  
  Serial.begin(9600);
}

void loop() {

  checkTemperature();
  checkDoorAndPIR();
  checkGasSensor();
  delay(1000);
  checkStock();
  delay(500);
  lcd.clear();
}

void checkTemperature() {
  int sensorValue = analogRead(TMP36_PIN);
  temperature = (sensorValue * 5.0 / 1023.0 - 0.5) * 100;

  Serial.print("TEMPERATURE: ");
  Serial.println(temperature);

  if (temperature < TEMP_LOW) {
    lcd.clear();
    lcd.print("Temp Too Low");
    tone(BUZZER_PIN, 255);
    delay(1000);
    noTone(BUZZER_PIN);
    
    // Open cooling switch
    coolingServo.write(0); // Move servo to "open" position
    lcd.clear();
    lcd.print("Cooling OFF");
    delay(1000);
  } 
  else if (temperature > TEMP_HIGH) {
    lcd.clear();
    lcd.print("Temp Too High");
    tone(BUZZER_PIN, 255);
    delay(1000);
    noTone(BUZZER_PIN);
    
    // Close cooling switch
    coolingServo.write(90); // Move servo to "closed" position
    lcd.clear();
    lcd.print("Cooling ON");
    delay(1000);
  }
}

void checkDoorAndPIR() {
  // Measure Distance
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  duration = pulseIn(ULTRASONIC_ECHO, HIGH);
  distance = duration * 0.034 / 2;

  // PIR Motion Detection
  pirMotion = digitalRead(PIR_PIN);

  if (distance < DOOR_DISTANCE_THRESHOLD) {
    if (!doorOpenTimerActive) {
      doorOpenStartTime = millis();
      doorOpenTimerActive = true;
      lcd.clear();
      lcd.print("Door Open!");
    }

    // Reset timer if motion is detected
    if (pirMotion == HIGH) {
      doorOpenStartTime = millis();
      lcd.setCursor(0, 1);
      lcd.print("Motion Detected");
    }

    // Check if timer exceeds 15 seconds
    if (millis() - doorOpenStartTime > DOOR_OPEN_DELAY) {
      doorServo.write(90); // Close door
      lcd.clear();
      lcd.print("Door Closing");
      tone(BUZZER_PIN, 1000, 500);
      delay(2000);
      lcd.clear();
      doorServo.write(0); // Reset servo
      doorOpenTimerActive = false;
    }
  } else {
    doorOpenTimerActive = false;
  }
}

void checkGasSensor() {
  gasLevel = analogRead(GAS_SENSOR_PIN); // Read raw sensor value
  Serial.print(gasLevel);
  // Debugging: Print gas level to Serial Monitor
  Serial.print("Gas Level: ");
  Serial.println(gasLevel);

  // Check if gas level exceeds threshold
  if (gasLevel > GAS_THRESHOLD) {
    lcd.clear();
    lcd.print("Spoiled Food!");
    
    // Alert with buzzer
    for (int i = 0; i < 4; i++) {
      tone(BUZZER_PIN, 1000); // Beep
      delay(500);            // Wait
      noTone(BUZZER_PIN);    // Silence
      delay(500);
    }
  }
}

void checkStock() {
  // Calculate weight in Newtons
  weightFood1 = analogRead(FORCE1_PIN) * 10.0 / 1023.0;
  weightFood2 = analogRead(FORCE2_PIN) * 10.0 / 1023.0;

  // Display warnings for low stock
  if (weightFood1 < FORCE_THRESHOLD) {
    Serial.print(weightFood1);
    lcd.clear();
    lcd.print("Egg/Milk Low");
    delay(2000);
  }
  if (weightFood2 < FORCE_THRESHOLD) {
    lcd.clear();
    lcd.print("Veg Low");
    delay(2000);
  }
}
