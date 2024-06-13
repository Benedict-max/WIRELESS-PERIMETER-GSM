#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Define pins for LDRs
const int sensorPins[4] = {A0, A1, A2, A3}; // LDRs connected to these pins
const int ledPins[4] = {4, 5, 6, 7}; // LEDs connected to these pins
const int buzzerPin = 8; // Buzzer connected to this pin
const int threshold = 500; // Threshold for detecting interruption (tune as necessary)

// Define SoftwareSerial for GSM communication
SoftwareSerial gsmSerial(2, 3); // Rx (D2), Tx (D3)

// Initialize the I2C LCD (address 0x27 for a 16x2 LCD)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialize serial communication for GSM
  gsmSerial.begin(9600); 
  Serial.begin(9600); // For debugging
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Perimeter Security");

  // Set pins for LEDs and buzzer as output
  for (int i = 0; i < 4; i++) {
    pinMode(sensorPins[i], INPUT);
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
}

void loop() {
  bool intrusionDetected = false;
  for (int i = 0; i < 4; i++) {
    int sensorValue = analogRead(sensorPins[i]);
    Serial.print("Sensor "); Serial.print(i+1); Serial.print(": "); Serial.println(sensorValue);
    
    if (sensorValue > threshold) {
      digitalWrite(ledPins[i], HIGH); // Turn on corresponding LED
      digitalWrite(buzzerPin, HIGH);  // Turn on buzzer
      sendSMS(i + 1);                 // Send SMS indicating which side
      lcd.setCursor(0, 1);
      lcd.print("Intrusion: Side "); lcd.print(i + 1);
      intrusionDetected = true;
    } else {
      digitalWrite(ledPins[i], LOW);  // Turn off corresponding LED
    }
  }
  if (!intrusionDetected) {
    lcd.setCursor(0, 1);
    lcd.print("Status: Secure   ");
  }
  delay(100); // Small delay to debounce
  digitalWrite(buzzerPin, LOW); // Turn off buzzer after the delay
}

void sendSMS(int side) {
  gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(100);
  gsmSerial.print("AT+CMGS=\"+1234567890\""); // Replace with your phone number
  delay(100);
  gsmSerial.print("Intrusion Detected on Side "); // Compose the message
  gsmSerial.println(side);
  gsmSerial.write(26); // Send Ctrl+Z to send SMS
  delay(1000); // Give time for the SMS to send
}
