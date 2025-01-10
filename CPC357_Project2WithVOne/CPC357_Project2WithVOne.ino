// Include necessary libraries
#include "VOneMqttClient.h"
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h> 

// Define device id based on the V-One platform
const char* IRSensor1 = "e86f6f33-c7c2-481b-9e27-3d83df2ce7c7";
const char* IRSensor2 = "5c0bb7d0-6175-473e-a80d-6618e7c568ed";

// Pin configuration for RFID (Parking Lot 1 and 2 - Using SPI Interface)
#define SS_PIN_1 7    // SDA/SS pin for Parking Lot 1
#define RST_PIN_1 21  // RST pin for Parking Lot 1
#define SS_PIN_2 42   // SDA/SS pin for Parking Lot 2
#define RST_PIN_2 5   // RST pin for Parking Lot 2

// Pin configuration for IR sensors and Servos
#define IR_SENSOR_PIN_1 6   // IR sensor for Parking Lot 1
#define SERVO_PIN_1 14      // Servo for Parking Lot 1
#define IR_SENSOR_PIN_2 4   // IR sensor for Parking Lot 2
#define SERVO_PIN_2 9       // Servo for Parking Lot 2

// Instantiate RFID readers for SPI
MFRC522 rfid1(SS_PIN_1, RST_PIN_1);
MFRC522 rfid2(SS_PIN_2, RST_PIN_2);

Servo barrierServo1, barrierServo2;

// Variables for tracking time and car presence
unsigned long carEntryTime1 = 0, carExitTime1 = 0, totalUsageTime1 = 0;
unsigned long carEntryTime2 = 0, carExitTime2 = 0, totalUsageTime2 = 0;
bool carDetected1 = false, carDetected2 = false;

// Create an instance of VOneMqttClient
VOneMqttClient voneClient;

// Setup wifi to transmit data to V-One platform
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // Initialize Serial Monitor
  Serial.println(F("Initializing Parking System..."));

  // Setup the wifi
  setup_wifi();
  voneClient.setup();

  // Initialize SPI and RFID readers
  SPI.begin(); // Shared SPI bus
  rfid1.PCD_Init();
  Serial.println(F("RFID Reader 1 Initialized (SPI)."));
  rfid2.PCD_Init();
  Serial.println(F("RFID Reader 2 Initialized (SPI)."));

  // Initialize Servos
  barrierServo1.attach(SERVO_PIN_1);
  barrierServo1.write(0); // Set barrier 1 to default position (open)
  barrierServo2.attach(SERVO_PIN_2);
  barrierServo2.write(0); // Set barrier 2 to default position (open)

  // Initialize IR sensors
  pinMode(IR_SENSOR_PIN_1, INPUT);
  pinMode(IR_SENSOR_PIN_2, INPUT);
}

// Continuously checking if there is a car that would like to use the parking lot
void loop() {
  // Handle Parking Lot 1 (RFID1)
  handleParkingLotSPI(1, rfid1, IR_SENSOR_PIN_1, barrierServo1, carDetected1, carEntryTime1, carExitTime1, totalUsageTime1, IRSensor1);

  // Handle Parking Lot 2 (RFID2)
  handleParkingLotSPI(2, rfid2, IR_SENSOR_PIN_2, barrierServo2, carDetected2, carEntryTime2, carExitTime2, totalUsageTime2, IRSensor2);

  delay(100); // Small delay to avoid rapid sensor readings
}

// Check if a RFID tag is scanned which shows that there is a authorized user that would like to use the lot
void handleParkingLotSPI(
  int lotNumber,
  MFRC522 &rfid,
  int irSensorPin,
  Servo &barrierServo,
  bool &carDetected,
  unsigned long &carEntryTime,
  unsigned long &carExitTime,
  unsigned long &totalUsageTime,
  const char* irSensorKey
) {
  // Check if a new RFID card is present
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print(F("RFID Tag Detected for Parking Lot "));
    Serial.println(lotNumber);

    // Print RFID UID
    Serial.print(F("UID: "));
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(rfid.uid.uidByte[i], HEX);
    }
    Serial.println();

    barrierServo.write(90); // Lower barrier
    voneClient.publishTelemetryData(irSensorKey, "InUse", 1); // Publish "InUse" = 1, to show the lot is occupied in monitoring dashboard
    carEntryTime = millis(); // Start timing to calculate the lot usage time

    rfid.PICC_HaltA(); // Halt communication with the card
  }

  // Check whether the car has entered the parking lot
  checkIRSensor(irSensorPin, carDetected, barrierServo, carEntryTime, carExitTime, totalUsageTime, lotNumber, irSensorKey);
}

// Check whether the car is in parking lot, if it is in parking lot, the barrier cannot be lifted
void checkIRSensor(
  int irSensorPin,
  bool &carDetected,
  Servo &barrierServo,
  unsigned long &carEntryTime,
  unsigned long &carExitTime,
  unsigned long &totalUsageTime,
  int lotNumber,
  const char* irSensorKey
) {
  if (digitalRead(irSensorPin) == LOW) { // Car is detected
    if (!carDetected) {
      Serial.print(F("Car Detected in Parking Lot "));
      Serial.println(lotNumber);
      carDetected = true;
    }
  } else { // No car detected
    if (carDetected) {
      Serial.print(F("Car Left Parking Lot "));
      Serial.println(lotNumber);
      carDetected = false;

      barrierServo.write(0); // Raise barrier
      voneClient.publishTelemetryData(irSensorKey, "InUse", 0); // Publish "InUse" = 0
      carExitTime = millis(); // Stop timing
      unsigned long usageTime = carExitTime - carEntryTime; // Calculate time in milliseconds
      totalUsageTime += usageTime;

      // Convert to minutes (1 min = 60,000 ms)
      float usageTimeMinutes = usageTime / 60000.0;

      Serial.print(F("Usage Time (minutes) for Parking Lot "));
      Serial.print(lotNumber);
      Serial.print(F(": "));
      Serial.println(usageTimeMinutes, 2); // Print with 2 decimal points

      if (!voneClient.connected()) {
        voneClient.reconnect();
        voneClient.publishDeviceStatusEvent(irSensorKey, true);
      }
      voneClient.loop();

      // Publish usage time in minutes as a float
      voneClient.publishTelemetryData(irSensorKey, "Obstacle", usageTimeMinutes);
    }
  }
}
