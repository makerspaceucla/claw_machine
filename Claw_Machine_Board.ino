//Last Edit by Tim Norris 3/3/25
//This version is applied to the RAMBo 1.3L Board

#include <AccelStepper.h>
#include <Servo.h>

// Define stepper motor connections and motor interface type
#define XEN 29    //Enable Pin for X Stepper Driver, High = off
#define YEN 28    //Enable Pin for Y Stepper Driver, High = off
#define ZEN 27    //Enable Pin for Z Stepper Driver, High = off
#define XPUL 37   // X Pulse/Step pin
#define XDIR 48   // X Direction pin
#define YPUL 36   // Y Pulse/Step pin
#define YDIR 49   // Y Direction pin
#define ZPUL 35   // Z Pulse/Step pin
#define ZDIR 47   // Z Direction pin
#define UP 45     // Up Joystick Pin (MX2 - Pin 5)
#define DN 31     // Down Joystick Pin (MX2 - Pin 4)
#define LT 46     // Left Joystick Pin (MX3 - Pin 5)
#define RT 30     // Right Joystick Pin (MX3 - Pin 4)
#define CLAW 23   // Claw Button Pin (MX2 - Pin 3)
#define SERVO_PIN 44  // Servo control pin (MX1 - Pin 5) (PWM-Capable)

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
  pinMode(XEN, OUTPUT);
  pinMode(YEN, OUTPUT);
  pinMode(ZEN, OUTPUT);

  //digitalWrite(ZEN, LOW);

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

  digitalWrite(XEN, LOW);
  digitalWrite(YEN, LOW);
  digitalWrite(ZEN, LOW);

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
  //int timer = 0;

  // X-axis control
  if (ltPressed) {
    //Serial.println("Going Left");
    xTarget -= joystickIncrement;
    //timer = 0;
  }
  if (rtPressed) {
    //Serial.println("Going Right");
    xTarget += joystickIncrement;
    //timer = 0;
  }
  // Enforce X-axis limits
  if (xTarget < X_MIN) xTarget = X_MIN;
  if (xTarget > X_MAX) xTarget = X_MAX;

  //Serial.println(xTarget);
  stepperX.moveTo(xTarget);

  // Y-axis control
  if (upPressed) {
    //Serial.println("Going Up");
    yTarget += joystickIncrement;
    //timer = 0;
  }
  if (dnPressed) {
    //Serial.println("Going Down");
    yTarget -= joystickIncrement;
    //timer = 0;
  }
  // Enforce Y-axis limits
  if (yTarget < Y_MIN) yTarget = Y_MIN;
  if (yTarget > Y_MAX) yTarget = Y_MAX;

  stepperY.moveTo(yTarget);

  // Run the steppers to execute movement
  stepperX.run();
  stepperY.run();
  // timer++;
  // if(timer == 10000){

  // }
  //Serial.println(timer);
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
