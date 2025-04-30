
// multi_pmw3901.ino
//
// Script to interface four PMW3901 optical flow sensors with an Arduino Uno R3
//
// External Code Attribution:
// - This script uses the Bitcraze_PMW3901 library from https://github.com/bitcraze/Bitcraze_PMW3901
// - Bitcraze's library handles low-level SPI communication with PMW3901 sensors
// - All code for initialising and reading multiple sensors is original work
//
// Hardware:
// - 4 PMW3901 sensors connected via SPI (MOSI, MISO, SCK)
// - Chip Select pins: Sensor 1 (Pin 10), Sensor 2 (Pin 9), Sensor 3 (Pin 8), Sensor 4 (Pin 7)
// - Power: 3.3V and GND shared across sensors

#include <Bitcraze_PMW3901.h>


// Initialise the four PMW3901 sensor instances with their respective CS pins
Bitcraze_PMW3901 sensor1(10); // Sensor 1 on pin 10
Bitcraze_PMW3901 sensor2(9);  // Sensor 2 on pin 9
Bitcraze_PMW3901 sensor3(8);  // Sensor 3 on pin 8
Bitcraze_PMW3901 sensor4(7);  // Sensor 4 on pin 7

void setup() {
  // Using Bitcraze_PMW3901 library function begin()
  Serial.begin(9600);

  // Initialise each sensor and check for connection
  if (!sensor1.begin()) {
    Serial.println("S1 failed");
    while (1) {}
  }
  if (!sensor2.begin()) {
    Serial.println("S2 failed");
    while (1) {}
  }
  if (!sensor3.begin()) {
    Serial.println("S3 failed");
    while (1) {}
  }
  if (!sensor4.begin()) {
    Serial.println("S4 failed");
    while (1) {}
  }
  Serial.println("All sensors initialised");
}

void loop() {
  // Variables to store motion data for each sensor
  int16_t deltaX1, deltaY1; // Sensor 1 (Down)
  int16_t deltaX2, deltaY2; // Sensor 2 (Forward)
  int16_t deltaX3, deltaY3; // Sensor 3 (Left)
  int16_t deltaX4, deltaY4; // Sensor 4 (Right)

  // Read motion data from each sensor
  // Using Bitcraze_PMW3901 library function readMotionCount()
  sensor1.readMotionCount(&deltaX1, &deltaY1);
  sensor2.readMotionCount(&deltaX2, &deltaY2);
  sensor3.readMotionCount(&deltaX3, &deltaY3);
  sensor4.readMotionCount(&deltaX4, &deltaY4);

  Serial.println("--- Motion Data ---");
  Serial.print("Down - X: "); Serial.print(deltaX1); Serial.print(", Y: "); Serial.println(deltaY1);
  Serial.print("Forward - X: "); Serial.print(deltaX2); Serial.print(", Y: "); Serial.println(deltaY2);
  Serial.print("Left - X: "); Serial.print(deltaX3); Serial.print(", Y: "); Serial.println(deltaY3);
  Serial.print("Right - X: "); Serial.print(deltaX4); Serial.print(", Y: "); Serial.println(deltaY4);

  delay(100);
}
