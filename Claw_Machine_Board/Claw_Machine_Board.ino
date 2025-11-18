// Last Edit by Tim Norris 3/3/25
// This version is applied to the RAMBo 1.3L Board

#include <AccelStepper.h>
#include <Servo.h>
#include <SPI.h>              // <— Added for digipot control

// ====== Digipot (AD5206) on RAMBo 1.3L ======
#define DIGIPOT_CS 38         // Chip Select for the digipot (RAMBo)
const uint8_t CH_X  = 4;      // Channel mapping on RAMBo: {4,5,3,0,1} = X,Y,Z,E0,E1
const uint8_t CH_Y  = 5;
const uint8_t CH_Z  = 3;
const uint8_t CH_E0 = 0;
const uint8_t CH_E1 = 1;

static inline void digiWrite(uint8_t channel, uint8_t value) {
  digitalWrite(DIGIPOT_CS, LOW);
  SPI.transfer(channel & 0x07);  // AD5206 expects channel 0..5
  SPI.transfer(value);           // 0..255 (lower = less current)
  digitalWrite(DIGIPOT_CS, HIGH);
}

// Adjust these to taste; start cool, nudge up if you get skips
#define DIGI_X   90
#define DIGI_Y   130
#define DIGI_Z   90
#define DIGI_E0  90
#define DIGI_E1  90

static inline void initDigipots() {
  pinMode(DIGIPOT_CS, OUTPUT);
  digitalWrite(DIGIPOT_CS, HIGH);
  SPI.begin();
  // Write all channels
  digiWrite(CH_X,  DIGI_X);
  digiWrite(CH_Y,  DIGI_Y);
  digiWrite(CH_Z,  DIGI_Z);
  digiWrite(CH_E0, DIGI_E0);
  digiWrite(CH_E1, DIGI_E1);
}

// ====== Stepper IO ======
// Enable pins (HIGH = off on RAMBo)
#define XEN 29
#define YEN 28
#define ZEN 26

// Step/dir pins
#define XPUL 37
#define XDIR 48
#define YPUL 36
#define YDIR 49
#define ZPUL 34
#define ZDIR 43

// Inputs / servo
#define UP 45
#define DN 31
#define LT 46
#define RT 30
#define CLAW 23
#define SERVO_PIN 44

// Motor interface
#define MotorInterfaceType 1

// Steppers
AccelStepper stepperX(MotorInterfaceType, XPUL, XDIR);
AccelStepper stepperY(MotorInterfaceType, YPUL, YDIR);
AccelStepper stepperZ(MotorInterfaceType, ZPUL, ZDIR);

bool claw_ready = true;
Servo claw_servo;

// Limits
const long X_MIN = 0;
const long X_MAX = 10000;
const long Y_MIN = 0;
const long Y_MAX = 10000;

// Targets
long xTarget = 0;
long yTarget = 0;

unsigned long delayMil = 1;

void setup() {
  Serial.begin(9600);

  // Inputs
  pinMode(UP, INPUT_PULLUP);
  pinMode(DN, INPUT_PULLUP);
  pinMode(LT, INPUT_PULLUP);
  pinMode(RT, INPUT_PULLUP);
  pinMode(CLAW, INPUT_PULLUP);

  // Enables
  pinMode(XEN, OUTPUT);
  pinMode(YEN, OUTPUT);
  pinMode(ZEN, OUTPUT);

  // Keep drivers disabled while we set current (HIGH = disabled)
  digitalWrite(XEN, HIGH);
  digitalWrite(YEN, HIGH);
  digitalWrite(ZEN, HIGH);

  // Init digipots BEFORE enabling drivers
  initDigipots();

  // Now enable drivers (LOW = enabled)
  digitalWrite(XEN, LOW);
  digitalWrite(YEN, LOW);
  digitalWrite(ZEN, LOW);

  // Servo
  claw_servo.attach(SERVO_PIN);
  claw_servo.write(0);

  // Motion settings
  float maxSpeed = 750;
  float acceleration = 10000;
  stepperX.setMaxSpeed(maxSpeed);
  stepperX.setAcceleration(acceleration);
  stepperX.setCurrentPosition(0);

  stepperY.setMaxSpeed(maxSpeed);
  stepperY.setAcceleration(acceleration);
  stepperY.setCurrentPosition(0);

  float zMaxSpeed = 2000;
  float zAcceleration = 5000;
  stepperZ.setMaxSpeed(zMaxSpeed);
  stepperZ.setAcceleration(zAcceleration);
  stepperZ.setCurrentPosition(0);

  Serial.println("Setup complete");
}

void loop() {
  bool upPressed = (digitalRead(UP) == LOW);
  bool dnPressed = (digitalRead(DN) == LOW);
  bool ltPressed = (digitalRead(LT) == LOW);
  bool rtPressed = (digitalRead(RT) == LOW);

  int joystickIncrement = 1;

  if (ltPressed) xTarget -= joystickIncrement;
  if (rtPressed) xTarget += joystickIncrement;
  if (xTarget < X_MIN) xTarget = X_MIN;
  if (xTarget > X_MAX) xTarget = X_MAX;
  stepperX.moveTo(xTarget);

  if (upPressed) yTarget += joystickIncrement;
  if (dnPressed) yTarget -= joystickIncrement;
  if (yTarget < Y_MIN) yTarget = Y_MIN;
  if (yTarget > Y_MAX) yTarget = Y_MAX;
  stepperY.moveTo(yTarget);

  stepperX.run();
  stepperY.run();
  delay(delayMil);

  if (digitalRead(CLAW) == LOW && claw_ready) {
    claw_ready = false;
    Serial.println("Dropping Claw");

    xTarget = stepperX.currentPosition();
    yTarget = stepperY.currentPosition();

    int zDropDistance = -3600;
    stepperZ.moveTo(zDropDistance);
    while (stepperZ.isRunning()) stepperZ.run();

    delay(500);
    claw_servo.write(75);
    Serial.println("Grabbing");
    delay(500);

    Serial.println("Raising Claw");
    stepperZ.moveTo(0);
    while (stepperZ.isRunning()) stepperZ.run();

    xTarget = X_MIN;
    yTarget = Y_MIN;
    stepperX.moveTo(xTarget);
    stepperY.moveTo(yTarget);
    while (stepperX.isRunning() || stepperY.isRunning()) {
      stepperX.run();
      stepperY.run();
    }

    delay(500);
    claw_servo.write(0);
    Serial.println("Releasing");
    claw_ready = true;
  }
}
