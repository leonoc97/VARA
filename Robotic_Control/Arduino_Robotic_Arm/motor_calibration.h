#include <Servo.h>

// Declare servo objects for each motor
Servo servoBase;   // Base motor
Servo servoElbow;  // Elbow
Servo servoShoulder; // Shoulder

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  servoBase.attach(3);       // Base motor uses Arduino Pin 3
  servoElbow.attach(9);      // Elbow uses Arduino Pin 9
  servoShoulder.attach(11);  // Shoulder uses Arduino Pin 11

  // Initialize servos to a middle position (usually a safe start point)
  servoBase.write(90);
  servoElbow.write(90);
  servoShoulder.write(90);

  Serial.println("Calibration Mode: Use 'b', 'e', 's' followed by angle to set base, elbow, and shoulder.");
  Serial.println("Example: b90 -> Moves the base to 90 degrees");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();  // Read the command character
    int angle = Serial.parseInt(); // Read the angle

    switch (command) {
      case 'b':
        servoBase.write(angle);
        Serial.print("Base moved to ");
        Serial.println(angle);
        break;
      case 'e':
        servoElbow.write(angle);
        Serial.print("Elbow moved to ");
        Serial.println(angle);
        break;
      case 's':
        servoShoulder.write(angle);
        Serial.print("Shoulder moved to ");
        Serial.println(angle);
        break;
      default:
        Serial.println("Invalid command. Use 'b' for base, 'e' for elbow, 's' for shoulder.");
        break;
    }
  }

  delay(100); // Slight delay to prevent overwhelming the serial buffer
}
