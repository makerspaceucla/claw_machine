//This version is applied to the Creality 1.1.5 Board
//1.1.5 Board has 1/16 Microstepping on on all motors

#include <AccelStepper.h>
#include <Servo.h>

// Define stepper motor connections and motor interface type
#define XYZEN 14  //Enable Pin for all motors, High = off
#define ZEN 26    //Enable Pin for Z Stepper Driver, High = off
#define XPUL 3   // X Pulse pin
#define XDIR 2   // X Direction pin
#define YPUL 22   // Y Pulse pin
#define YDIR 23   // Y Direction pin
#define ZPUL 0    // Z Pulse pin
#define ZDIR 1    // Z Direction pin
#define UP 18     // Up Joystick Pin (X-Stop Connector)
#define DN 19     // Down Joystick Pin (Y-Stop Connector)
#define LT 20     // Left Joystick Pin (Z-Stop Connector)
#define RT 25     // Right Joystick Pin (B-Mot Connector)
#define CLAW 24   // Claw Button Pin (ETemp Connector)
#define SERVO_PIN 4  // Servo control pin (PWM-capable) (Fan Connector)

// Define motor interface type
#define MotorInterfaceType 1  // 1 = Driver with Step and Direction pins

// Create instances of AccelStepper for each axis
AccelStepper stepperX(MotorInterfaceType, XPUL, XDIR);
AccelStepper stepperY(MotorInterfaceType, YPUL, YDIR);
AccelStepper stepperZ(MotorInterfaceType, ZPUL, ZDIR);

bool claw_ready = true;  // Debounce boolean for claw operation
Servo claw_servo;        // Servo motor for the claw

// Define position limits (replace with your machine's actual limits)
const long X_MIN = 0;
const long X_MAX = 10000;  // TODO: Replace with your machine's maximum X position
const long Y_MIN = 0;
const long Y_MAX = 10000;  // TODO: Replace with your machine's maximum Y position

// Global variables for target positions
long xTarget = 0;
long yTarget = 0;

unsigned long delayMil = 1;

void setup() {
  Serial.begin(9600);

  // Initialize joystick pins
  pinMode(UP, INPUT_PULLUP);
  pinMode(DN, INPUT_PULLUP);
  pinMode(LT, INPUT_PULLUP);
  pinMode(RT, INPUT_PULLUP);
  pinMode(CLAW, INPUT_PULLUP);
  pinMode(XYZEN, OUTPUT);
  pinMode(ZEN, OUTPUT);

  digitalWrite(ZEN, LOW);

  // Initialize the servo
  claw_servo.attach(SERVO_PIN);
  claw_servo.write(0);  // Set Servo Start Angle to 0 degrees (open claw)

  // Set reduced speed and acceleration for safety during testing
  float maxSpeed = 750;       // Maximum speed in steps per second (adjust as needed)
  float acceleration = 10000;   // Acceleration in steps per second squared (adjust as needed)

  stepperX.setMaxSpeed(maxSpeed);
  stepperX.setAcceleration(acceleration);
  stepperX.setCurrentPosition(0);

  stepperY.setMaxSpeed(maxSpeed);
  stepperY.setAcceleration(acceleration);
  stepperY.setCurrentPosition(0);

  // Set speed and acceleration for Z-axis
  float zMaxSpeed = 500;        // Adjust as needed
  float zAcceleration = 500;    // Adjust as needed

  stepperZ.setMaxSpeed(zMaxSpeed);
  stepperZ.setAcceleration(zAcceleration);
  stepperZ.setCurrentPosition(0);

  Serial.println("Setup complete");
}

void loop() {
  // Read joystick inputs
  bool upPressed = (digitalRead(UP) == LOW);
  bool dnPressed = (digitalRead(DN) == LOW);
  bool ltPressed = (digitalRead(LT) == LOW);
  bool rtPressed = (digitalRead(RT) == LOW);

  // Movement increment per joystick input
  int joystickIncrement = 1;  // Adjust for sensitivity

  // X-axis control
  if (ltPressed) {
    Serial.println("Going Left");
    xTarget -= joystickIncrement;
  }
  if (rtPressed) {
    xTarget += joystickIncrement;
  }
  // Enforce X-axis limits
  if (xTarget < X_MIN) xTarget = X_MIN;
  if (xTarget > X_MAX) xTarget = X_MAX;

  Serial.println(xTarget);
  stepperX.moveTo(xTarget);

  // Y-axis control
  if (upPressed) {
    yTarget += joystickIncrement;
  }
  if (dnPressed) {
    yTarget -= joystickIncrement;
  }
  // Enforce Y-axis limits
  if (yTarget < Y_MIN) yTarget = Y_MIN;
  if (yTarget > Y_MAX) yTarget = Y_MAX;

  stepperY.moveTo(yTarget);

  // Run the steppers to execute movement
  stepperX.run();
  stepperY.run();
  delay(delayMil);
  
  // Claw operation
  if (digitalRead(CLAW) == LOW && claw_ready) {
    claw_ready = false;  // Debounce claw button to prevent repeated activation
    Serial.println("Dropping Claw");

    // Stop X and Y movement by setting targets to current positions
    xTarget = stepperX.currentPosition();
    yTarget = stepperY.currentPosition();

    // Move Z-axis down
    int zDropDistance = -900;  // Negative value to move down 900 steps
    stepperZ.moveTo(zDropDistance);

    while (stepperZ.isRunning()) {
      stepperZ.run();
    }

    delay(500);
    claw_servo.write(75);  // Close claw (adjust angle as needed)
    Serial.println("Grabbing");
    delay(500);

    Serial.println("Raising Claw");

    // Move Z-axis up
    stepperZ.moveTo(0);

    while (stepperZ.isRunning()) {
      stepperZ.run();
    }

    // Return X and Y axes to origin within limits
    xTarget = X_MIN;
    yTarget = Y_MIN;
    stepperX.moveTo(xTarget);
    stepperY.moveTo(yTarget);

    while (stepperX.isRunning() || stepperY.isRunning()) {
      stepperX.run();
      stepperY.run();
    }

    delay(500);
    claw_servo.write(0);  // Open claw (adjust angle as needed)
    Serial.println("Releasing");
    claw_ready = true;  // Reset claw ready state
  }
}
