/*
UNPHAYZED ELECTRIC CHAMPION KIT 2025-2026

MAIN VEHICLE CODE
*/

#include <RobojaxBTS7960.h>  //Download this library from the instructions page
#include <math.h>


//MOTOR DRIVER PINS+SETUP------------------------------------
#define RPWM 10  // define pin for RPWM pin (output)
#define R_EN 11  // define pin for R_EN pin (input)
#define R_IS 13  // define pin for R_IS pin (output)

#define LPWM 9   // define pin for LPWM pin (output)
#define L_EN 8   // define pin for L_EN pin (input)
#define L_IS 12  // define pin for L_IS pin (output)

#define CW 1   //defines CW motor movement
#define CCW 0  //defines CCW motor movement

#define debug 1  //change to 0 to hide serial monitor debugging infornmation or set to 1 to view

RobojaxBTS7960 motor(R_EN, RPWM, R_IS, L_EN, LPWM, L_IS, debug);


//ENCODER PINS------------------------------------
#define ENCA 2  //Vehicle encoder pin
#define ENCB 3  //Vehicle encoder pin


//PUSH BUTTON PINS------------------------------------
#define StartButtonPin 6  //Start vehicle


//ENCODER VARIABLES------------------------------------
volatile unsigned long counter = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile uint8_t encState;

const int8_t encArr[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};


//MOVEMENT VARIABLES------------------------------------
double TargetDistanceInM = 10.000;                //Target Distance in m
double TargetDistance = TargetDistanceInM * 100;  //Target Distance in cm
double ArcLength;                                 //Actual travel distance
double TargetTime = 20.00;                        //Target Time in s
double wheelDiameter = 6.0325;                    //Wheel Diameter is 2.875in = 7.3025 cm
double wheelCircumfrence = wheelDiameter * 3.14159;
double slowDownDistance;    //Distance that initial slowing is initiated
double snailDistance;       //Distance that secondary slowing is initiated
double CPR = 480;  //Encoder CPR

bool SBpressed = false;
bool moved = false;
bool slowed = false;
bool snail = false;
bool reachedTargetDistance = false;

double targetEncoderValue;
double slowDownEncoderValue;
double maxEncoderValue;
double encoderChange;
double finalDist;


//TIMER VARIABLES------------------------------------
long startTime;
long currentTime;
long endTime;

double timeDiff;
double milisTime;
double runTime;


void setup() {

  Serial.begin(9600);
  Serial.println("Entered Setup");


  //MOTOR SETUP----------------------------------------------------
  motor.begin();
  Serial.println("Motor Setup Complete");


  //BUTTON SETUP----------------------------------------------------
  pinMode(StartButtonPin, INPUT_PULLUP);

  Serial.println("Button Setup Complete");


  //ENCODER SETUP----------------------------------------------------
  pinMode(ENCA, INPUT_PULLUP);  // set pin to input w/ pullup resistors
  pinMode(ENCB, INPUT_PULLUP);  // set pin to input w/ pullup resistors

  //setting the current encoder state
  encState = (digitalRead(ENCA) << 1) | digitalRead(ENCB);

  //Setting up interrupt
  //Activates whenever theres a change in either of the encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCA), encInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), encInterrupt, CHANGE);

  Serial.println("Encoder Setup Complete");


  //TARGET ENCODER VALUES SETUP----------------------------------------------------
  targetEncoderValue = getEncoderValue(TargetDistance, wheelDiameter);
  maxEncoderValue = targetEncoderValue * 1.5;

  Serial.println(targetEncoderValue);
  Serial.println(maxEncoderValue);

  Serial.println("Target Values Set");
  Serial.println("Target Values Set");


  //VARIABLE SETUP----------------------------------------------------
  SBpressed = false;
  moved = false;
  slowed = false;
  snail = false;
  reachedTargetDistance = false;

  Serial.println("Variable Reset Complete");

  Serial.println("Setup Complete");
}

void loop() {

  if (digitalRead(StartButtonPin) == LOW) {
    //This section of code resets the movement bools and counter and records the starting time
    Serial.println("Start pressed");
    moved = false;
    slowed = false;
    snail = false;
    reachedTargetDistance = false;
    counter = 0;
    startTime = millis();
    Serial.println(startTime);
    SBpressed = true;
  }

  if (SBpressed) {
    //This section of code turns the motor on
    motor.rotate(50, CW);  // run motor with 30% speed in CW direction
    moved = true;
    SBpressed = false;
  }

  if (moved) {
    //This section of code keeps the motor running until the vehicle reaches the slow down dist
    if (counter >= slowDownEncoderValue && counter < maxEncoderValue) {
      Serial.println("Reached Decel Distance");
      //This section of code stops the motor and checks run time
      motor.stop();  // stop the motor
      motor.rotate(30, CCW);
      delay(100);
      motor.stop();
      delay(500);

      currentTime = millis();
      Serial.println(currentTime);
      milisTime = currentTime - startTime;
      runTime = milisTime / 1000.00;
      timeDiff = TargetTime - runTime;

      Serial.println(timeDiff);

      slowed = true;
      moved = false;
    }
  }

  if (slowed) {
    /*
    This section of code initiates a set of small pulses.
    These pulses are averaged and used to determine the total number needed to reach the target dist.
    By knowing the amount of pulses needed, the interval between pulses can be adjusted to achieve the target time.
    */
    encoderChange = 0;
    double initalEncoderVal;
    double finalEncoderVal;

    //Performs 5 pulses at 0.4 secs per pulse
    for (int i = 0; i < 5; i++) {
      initalEncoderVal = counter;
      motor.rotate(10, CW);
      delay(200);
      motor.stop();
      delay(200);
      finalEncoderVal = counter;

      encoderChange += (finalEncoderVal - initalEncoderVal);
    }

    //average distance change per pulse
    encoderChange = encoderChange / 5;

    //check run time
    currentTime = millis();
    Serial.println(currentTime);
    milisTime = currentTime - startTime;
    runTime = milisTime / 1000.00;
    timeDiff = TargetTime - runTime;

    Serial.println(timeDiff);

    snail = true;
    slowed = false;
  }


  if (snail) {
    //This sections of code determines number of pulses needed to reach TD
    //It also determines the interval length needed to reach the target time
    double remainingDist = targetEncoderValue - counter;
    double pulses = remainingDist / encoderChange;
    double pulseTime = pulses * 0.2;
    double nonPulseTime = timeDiff - pulseTime;
    if (nonPulseTime < 0) {
      nonPulseTime = 0;
    }
    double pulseDelay = 1000 * (nonPulseTime / (pulses - 1));  //delay in ms

    //initiate pulses
    for (int i = 0; i < pulses; i++) {
      motor.rotate(10, CW);
      delay(200);
      motor.stop();
      delay(pulseDelay);
    }

    //check run time
    endTime = millis();
    Serial.println(endTime);
    milisTime = endTime - startTime - pulseDelay;
    runTime = milisTime / 1000.00;
    Serial.println(runTime);

    snail = false;
    reachedTargetDistance = true;
  }

  if (reachedTargetDistance) {
    //This section of code records actual dist and time for debugging purposes

    delay(1000);  //wait 1 sec to make sure all movement has stopped

    Serial.println("Reached Target Distance Loop");
    Serial.println(counter);

    //determine distance error from run
    finalDist = TargetDistance * (counter / targetEncoderValue);

    //reset all bools + counter
    SBpressed = false;
    moved = false;
    slowed = false;
    snail = false;
    reachedTargetDistance = false;
    counter = 0;
  }
}


void encInterrupt() {
  uint8_t A = digitalRead(ENCA);
  uint8_t B = digitalRead(ENCB);
  
  uint8_t newEncState = (A << 1) | B;
  uint8_t index = (encState << 2) | newEncState;

  counter += encArr[index];

  encState = newEncState;
}

double getEncoderValue(double TD, double WD) {
  double WheelCircumfrence = 3.14159 * WD;
  double tgtENCval = (TD / WheelCircumfrence) * CPR;
  return tgtENCval;
}

double getArcLength(double TargetDist) {
  double vehicleWidth = 13.00;
  double canBonusError = 2.00;
  double arcHeight = 100.00 - (vehicleWidth + canBonusError) / 2;
  double arcRadius = ((arcHeight / 2) + (sq(TargetDist) / (8 * arcHeight)));
  double arcAngle = 2 * asin(TargetDist / (2 * arcRadius));
  double arcLength = arcRadius * arcAngle;
  return arcLength;
}
