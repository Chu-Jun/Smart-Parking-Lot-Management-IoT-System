#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h> // Use ESP32Servo instead of Servo

// Pin configuration for RFID (Parking Lot 1 and 2 - SPI)
#define SS_PIN_1 7    // SDA/SS pin for Parking Lot 1
#define RST_PIN_1 21  // RST pin for Parking Lot 1
#define SS_PIN_2 42    // SDA/SS pin for Parking Lot 2
#define RST_PIN_2 5  // RST pin for Parking Lot 2

// Pin configuration for IR sensors and Servos
#define IR_SENSOR_PIN_1 6   // IR sensor for Parking Lot 1
#define SERVO_PIN_1 14       // Servo for Parking Lot 1
#define IR_SENSOR_PIN_2 4    // IR sensor for Parking Lot 2
#define SERVO_PIN_2 9        // Servo for Parking Lot 2

// Instantiate RFID readers for SPI
MFRC522 rfid1(SS_PIN_1, RST_PIN_1);
MFRC522 rfid2(SS_PIN_2, RST_PIN_2);
Servo barrierServo1, barrierServo2;

// Variables for tracking time and car presence
unsigned long carEntryTime1 = 0, carExitTime1 = 0, totalUsageTime1 = 0;
unsigned long carEntryTime2 = 0, carExitTime2 = 0, totalUsageTime2 = 0;
bool carDetected1 = false, carDetected2 = false;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println(F("Initializing Parking System..."));

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

void loop() {
  // Handle Parking Lot 1 (RFID1)
  handleParkingLotSPI(1, rfid1, IR_SENSOR_PIN_1, barrierServo1, carDetected1, carEntryTime1, carExitTime1, totalUsageTime1);

  // Handle Parking Lot 2 (RFID2)
  handleParkingLotSPI(2, rfid2, IR_SENSOR_PIN_2, barrierServo2, carDetected2, carEntryTime2, carExitTime2, totalUsageTime2);

  delay(100); // Small delay to avoid rapid sensor readings
}

void handleParkingLotSPI(
  int lotNumber,
  MFRC522 &rfid,
  int irSensorPin,
  Servo &barrierServo,
  bool &carDetected,
  unsigned long &carEntryTime,
  unsigned long &carExitTime,
  unsigned long &totalUsageTime
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
    carEntryTime = millis(); // Start timing

    rfid.PICC_HaltA(); // Halt communication with the card
  }

  checkIRSensor(irSensorPin, carDetected, barrierServo, carEntryTime, carExitTime, totalUsageTime, lotNumber);
}

void checkIRSensor(
  int irSensorPin,
  bool &carDetected,
  Servo &barrierServo,
  unsigned long &carEntryTime,
  unsigned long &carExitTime,
  unsigned long &totalUsageTime,
  int lotNumber
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
      carExitTime = millis(); // Stop timing
      unsigned long usageTime = carExitTime - carEntryTime;
      totalUsageTime += usageTime;

      Serial.print(F("Usage Time (ms) for Parking Lot "));
      Serial.print(lotNumber);
      Serial.print(F(": "));
      Serial.println(usageTime);
    }
  }
}
