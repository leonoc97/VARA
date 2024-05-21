#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define INPUT_BUFFER_SIZE 50
#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)

// Define servo channels on the PCA9685
#define BASE_CHANNEL 0
#define SHOULDER_CHANNEL 1
#define ELBOW_CHANNEL 2
#define WRIST_ROT_CHANNEL 3
#define WRIST_PITCH_CHANNEL 4
#define GRIPPER_CHANNEL 5

// Angle ranges for each servo
#define BASE_MIN_ANGLE 0
#define BASE_MAX_ANGLE 180
#define SHOULDER_MIN_ANGLE 15
#define SHOULDER_MAX_ANGLE 165
#define ELBOW_MIN_ANGLE 0
#define ELBOW_MAX_ANGLE 180
#define WRIST_ROT_MIN_ANGLE 0
#define WRIST_ROT_MAX_ANGLE 180
#define WRIST_PITCH_MIN_ANGLE 0
#define WRIST_PITCH_MAX_ANGLE 180
#define GRIPPER_MIN_ANGLE 10
#define GRIPPER_MAX_ANGLE 73

static char inputBuffer[INPUT_BUFFER_SIZE];
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct Position {
  int base;
  int shoulder;
  int elbow;
  int wrist_rot;
  int wrist_pitch;
  int gripper;
  int speed;

  Position() : base(90), shoulder(90), elbow(90), wrist_rot(90), wrist_pitch(90), gripper(73), speed(150) {}

  int setFromString(char* input) {
    int parsed = sscanf(input, "P%d,%d,%d,%d,%d,%d,%d", &base, &shoulder, &elbow, &wrist_rot, &wrist_pitch, &gripper, &speed);
    return (parsed == 7) ? 1 : -1;
  }

  void set(int b, int s, int e, int wr, int wp, int g) {
    base = b;
    shoulder = s;
    elbow = e;
    wrist_rot = wr;
    wrist_pitch = wp;
    gripper = g;
    speed = 150;  // Default speed
  }
};

Position armPosition;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz
  delay(10);
}

void loop() {
    handleInput();
}

void handleInput() {
  if (Serial.available() > 0) {
    byte result = Serial.readBytesUntil('\n', inputBuffer, INPUT_BUFFER_SIZE);
    inputBuffer[result] = 0;
    interpretCommand(inputBuffer, result);
  }
}

void interpretCommand(char* inputBuffer, byte commandLength) {
  if (inputBuffer[0] == 'P') {
    positionArm(&inputBuffer[0]);
  } else if (inputBuffer[0] == 'H') {
    homePositionArm();
  } else if (inputBuffer[0] == '0') {
    powerOff();
    Serial.println("OK");
  }  else if (inputBuffer[0] == '1') {
    powerOn();
    Serial.println("OK");
  } else {
    Serial.println("E0");
  }
  Serial.flush();
}

void setServoAngle(uint8_t channel, int angle, int minAngle, int maxAngle) {
  angle = constrain(angle, minAngle, maxAngle);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void moveToPosition(Position &pos) {
  setServoAngle(BASE_CHANNEL, pos.base, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
  delay(pos.speed);
  setServoAngle(SHOULDER_CHANNEL, pos.shoulder, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE);
  delay(pos.speed);
  setServoAngle(ELBOW_CHANNEL, pos.elbow, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
  delay(pos.speed);
  setServoAngle(WRIST_ROT_CHANNEL, pos.wrist_rot, WRIST_ROT_MIN_ANGLE, WRIST_ROT_MAX_ANGLE);
  delay(pos.speed);
  setServoAngle(WRIST_PITCH_CHANNEL, pos.wrist_pitch, WRIST_PITCH_MIN_ANGLE, WRIST_PITCH_MAX_ANGLE);
  delay(pos.speed);
  setServoAngle(GRIPPER_CHANNEL, pos.gripper, GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);
  delay(pos.speed);
}

void positionArm(char *in) {
  int result = armPosition.setFromString(in);
  if (result > 0) {
    moveToPosition(armPosition);
    Serial.println("OK");
  } else {
    Serial.println("E1");
  }
}

void homePositionArm() {
  armPosition.set(90, 90, 90, 90, 90, 73);
  moveToPosition(armPosition);
  Serial.println("OK");
}

void powerOff() {
  pwm.setPWM(BASE_CHANNEL, 0, 0);
  pwm.setPWM(SHOULDER_CHANNEL, 0, 0);
  pwm.setPWM(ELBOW_CHANNEL, 0, 0);
  pwm.setPWM(WRIST_ROT_CHANNEL, 0, 0);
  pwm.setPWM(WRIST_PITCH_CHANNEL, 0, 0);
  pwm.setPWM(GRIPPER_CHANNEL, 0, 0);
}

void powerOn() {
  moveToPosition(armPosition);
}
