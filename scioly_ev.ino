/*
hehe I love PID trust

also once the esp32 is on for like 71 min the ev will prob start tweaking out bc
i think micros() overflows and will probably have to restart the esp32

TODO: see if u can change pos PID clamping to the calculated trapezoidal max velocity
later in the script when distance and time are set
TODO: also change the encoder used for controlling the vehicle to use ESP32Encoder
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <BTS7960.h>
#include <PID_v1.h>

#include "trajectory.h"
#include "controls.h"

// rotary encoder pins
#define MTR_ENC_A 32
#define MTR_ENC_B 33

#define CTRL_ENC_A 17
#define CTRL_ENC_B 16
#define CTRL_ENC_BTN 4

// motor controller pins
#define R_EN 25
#define L_EN 14
#define R_PWM 26
#define L_PWM 27

// button pins
#define START_BTN_PIN 18

// screen pins
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

// movement vars?
constexpr int CPR = 445;

constexpr int motorDeadzone = 35; // is a pwm value

constexpr double wheelDia = 6.0325; // in cm
constexpr double wheelCircumference = wheelDia * 3.14159265;

double targetDInM = 8.0;
long targetD; // target dist in counts, set right after onboard setting is done
double targetT = 12.0;

constexpr double accelInM = 0.7;                                       // m/s^2
extern const double accel = accelInM * 100 / wheelCircumference * CPR; // in counts/s^2
constexpr double maxSpdInM = 1.5;                                      // m/s
constexpr double maxSpd = maxSpdInM * 100 / wheelCircumference * CPR;  // in counts/s

// PID pain
constexpr double Kp_pos = 0;
constexpr double Ki_pos = 0;
constexpr double Kd_pos = 0;
constexpr long posPIDInterval = 10000; // in microseconds

constexpr double Kp_vel = 0;
constexpr double Ki_vel = 0;
constexpr double Kd_vel = 0;
constexpr long velPIDInterval = 1000; // in microseconds

double posPIDIn, posPIDOut, posPIDSetpt;
double velPIDIn, velPIDOut, velPIDSetpt;

// random timing/constrol stuff
long lastPosPIDTime = 0; // in microseconds
long lastVelPIDTime = 0; // in microseconds
unsigned long startTime;

long lastEncVal = 0;

bool running = false; // run the PID part of the loop and dont check for other input if true

// objects for literally everything
ESP32Encoder motorEnc;

BTS7960 motor(L_EN, R_EN, L_PWM, R_PWM);

Controls controls(targetDInM, targetT, CTRL_ENC_A, CTRL_ENC_B, CTRL_ENC_BTN);

Trajectory *traj = nullptr; // idk what pointers are so idk if i should be using them bc memory leaks or something

PID posPID(&posPIDIn, &posPIDOut, &posPIDSetpt, Kp_pos, Ki_pos, Kd_pos, DIRECT);
PID velPID(&velPIDIn, &velPIDOut, &velPIDSetpt, Kp_vel, Ki_vel, Kd_vel, DIRECT);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// drive motor left or right given a value from -255 to 255
void driveMotor(int PIDVal) {
  if(PIDVal >= 0) {
    motor.TurnLeft(PIDVal + motorDeadzone);
  } else {
    motor.TurnRight(-PIDVal - motorDeadzone);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("------------Beginning Setup------------"));

  // reset encoder value
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  motorEnc.attachFullQuad(MTR_ENC_B, MTR_ENC_A);
  motorEnc.clearCount();
  Serial.println(F("encoder configured"));

  // configure motor
  motor.Disable();
  Serial.println(F("ts motor should work hopefully"));

  // configure start button
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  Serial.println(F("start button configured"));

  // configure PID
  posPID.SetOutputLimits(-maxSpd, maxSpd);
  velPID.SetOutputLimits(-255 + motorDeadzone, 255 - motorDeadzone);
  posPID.SetMode(AUTOMATIC);
  velPID.SetMode(AUTOMATIC);
  posPID.SetSampleTime(1); // needed bc the PID has an internal timer to limit how often ts called
  velPID.SetSampleTime(1); // needed bc the PID has an internal timer to limit how often ts called
  Serial.println(F("PID configured"));

  // configure display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("yeah its over ts screen didnt work"));
    for(;;); // loop forever ig
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
  display.setCursor(0, 0);
  display.println(F("display configured"));
  display.display();

  // reset vars
  running = false;
  lastPosPIDTime = 0;
  lastVelPIDTime = 0;
  lastEncVal = 0;
  Serial.println(F("variables reset"));

  Serial.println(F("------------Setup Complete------------"));
}

void loop() {
  if(running) {
    unsigned long runTime = micros() - startTime;

    // pos loop
    if(runTime - lastPosPIDTime >= posPIDInterval || lastPosPIDTime == 0) {
      lastPosPIDTime = runTime;

      posPIDSetpt = traj->getTargetPos(runTime * 0.000001);
      posPIDIn = motorEnc.getCount();

      posPID.Compute(); // sets posPIDOut in-place btw
    }

    // vel loop
    if(runTime - lastVelPIDTime >= velPIDInterval || lastVelPIDTime == 0) {
      long currEncVal = motorEnc.getCount();
      double dt = 0.000001 * (runTime - lastVelPIDTime);
      velPIDIn = (currEncVal - lastEncVal) / dt; // calc curr velocity as early as possible
      lastEncVal = currEncVal;

      lastVelPIDTime = runTime; // reset time early so intervals are accurate

      velPIDSetpt = traj->getTargetVel(runTime * 0.000001) + posPIDOut;

      velPID.Compute(); // sets velPIDOut in-place btw

      driveMotor(velPIDOut);
    }

    // TODO: prob need a better termination condition
    if(motorEnc.getCount() >= targetD) {
      // disable motor
      motor.Stop();
      motor.Disable();

      // print results
      String timeTraveled = String((micros() - startTime) * 0.000001, 3);
      String distTraveled = String(motorEnc.getCount() / (double)CPR * wheelCircumference * 0.01, 3);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Acc dist: " + distTraveled);
      display.println("Acc time: " + timeTraveled);

      // reset vars
      running = false;
      controls.reset();
      motorEnc.clearCount();
      lastEncVal = 0;
      lastPosPIDTime = 0;
      lastVelPIDTime = 0;
      delete traj;
      traj = nullptr;
    }
  } else if(controls.ctrlsActive()) {
    // ts else if runs to set the targetD and targetT using controls object
    delay(10);

    if(controls.update()) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Target Dist: " + String(targetDInM, 3));
      display.println("Target Time: " + String(targetT, 3));
      display.display();
    }

    // check if ready to run after ts updated
    if(!controls.ctrlsActive()) {
      // if ctrls disabled bc ready to run,
      targetD = round(targetDInM * 100 / wheelCircumference * CPR);

      delete traj;
      traj = new Trajectory(targetD, targetT);

      lastEncVal = 0;
      lastPosPIDTime = 0;
      lastVelPIDTime = 0;

      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Target Dist: " + String(targetDInM, 3));
      display.println("Target Time: " + String(targetT, 3));
      display.println("READY TO RUN");
      display.display();
    }
  } else {
    // ts else runs if targetT and targetD are set and now just waiting for start button press
    delay(10);

    if(digitalRead(START_BTN_PIN) == LOW) {
      running = true;
      motorEnc.clearCount();
      motor.Enable();

      startTime = micros();
    }
  }
}