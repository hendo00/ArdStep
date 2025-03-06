#include <Arduino.h>

// Pin Definitions
#define EN_PIN 8      // LOW: Driver enabled, HIGH: Driver disabled
#define STEP_PIN 9    // Step on the rising edge
#define DIR_PIN 10    // Set stepping direction
#define POT_PIN A0    // Potentiometer input (analog)

// Motor Settings
int microSecondsDelay = 500;  // Speed control
const int maxSteps = 1700;     // Max mapped steps
int currentPosition = 0;       // Track current step position
int lastPotValue = -1;         // Store last potentiometer value
const int tolerance = 5;       // Hysteresis tolerance (Increase this to reduce bouncing)
const int numSamples = 10;     // Number of samples for moving average filter
int potReadings[numSamples];   // Array to store previous readings
int readIndex = 0;             // Current index in the array

// Moving Average Filter for Potentiometer Readings
int readPotentiometer() {
  int sum = 0;
  
  // Read new potentiometer value
  potReadings[readIndex] = analogRead(POT_PIN);
  
  // Sum all stored values
  for (int i = 0; i < numSamples; i++) {
    sum += potReadings[i];
  }
  
  // Move to next index (circular buffer)
  readIndex = (readIndex + 1) % numSamples;
  
  // Return the smoothed average value
  return sum / numSamples;
}
void stepMotor(int targetPos) {
  while (currentPosition != targetPos) {
    // Update potentiometer continuously with smoothed value
    int potValue = readPotentiometer(); // Get stabilized reading
    int newTarget = map(potValue, 0, 1023, 0, maxSteps);
    
    if (abs(newTarget - targetPos) > tolerance) {
      targetPos = newTarget; // Update target if knob moved fast
    }

    // Set direction
    if (currentPosition < targetPos) {
      digitalWrite(DIR_PIN, HIGH);
      currentPosition++;
    } else if (currentPosition > targetPos) {
      digitalWrite(DIR_PIN, LOW);
      currentPosition--;
    }

    // Step pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(microSecondsDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(microSecondsDelay);
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  
  digitalWrite(EN_PIN, LOW);

  // Initialize smoothing array with first potentiometer value
  int initialReading = analogRead(POT_PIN);
  for (int i = 0; i < numSamples; i++) {
    potReadings[i] = initialReading;
  }
}

void loop() {
  int potValue = readPotentiometer();
  int targetPosition = map(potValue, 0, 1023, 0, maxSteps);

  if (abs(potValue - lastPotValue) > tolerance) {
    lastPotValue = potValue;
    stepMotor(targetPosition); // Move in real-time
  }

  Serial.println(potValue);
}
