/* Compared to the firmware for AR4, The motor direction is inverted for all
 * axes except for J4.
 */
#include <AccelStepper.h>
#include <Encoder.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdio.h>
#include <string>

const bool SIMUL = false;
String SIMULPOS="JPA-170B40.5C84.5D184.4E85.4F179.0G0\n";
// Firmware version
const char* VERSION = "0.0.2";

///////////////////////////////////////////////////////////////////////////////
// Physical Params
///////////////////////////////////////////////////////////////////////////////

const int STEP_PINS[] = {0, 2, 4, 6, 8, 10, 12, 32, 34};
const int DIR_PINS[] = {1, 3, 5, 7, 9, 11, 13, 33, 35};
const int LIMIT_PINS[] = {26, 27, 28, 29, 30, 31, 37, 36, 38}; // 37 / 36 !?

const float MOTOR_STEPS_PER_DEG[] = {44.44444444, 55.55555556, 55.55555556,
                                     49.77777777, 21.86024888, 22.22222222,
                                     14.2857, 14.2857, 14.2857};
//const int MOTOR_STEPS_PER_REV[] = {400, 400, 400, 400, 800, 400};

// 1000 steps on linear joint is 14.2857 cm !

// set encoder pins
Encoder encPos[6] = {Encoder(14, 15), Encoder(17, 16), Encoder(19, 18),
                     Encoder(20, 21), Encoder(23, 22), Encoder(24, 25)};
// +1 if encoder direction matches motor direction, -1 otherwise
int ENC_DIR[] = {-1, 1, 1, 1, 1, 1, -1, 1, 1};
// +1 if encoder max value is at the minimum joint angle, 0 otherwise
int ENC_MAX_AT_ANGLE_MIN[] = {1, 0, 1, 0, 0, 1, 0, 0, 0};
// motor steps * ENC_MULT = encoder steps
const float ENC_MULT[] = {10, 10, 10, 10, 5, 10, 1, 10, 10};

// define axis limits in degrees, for calibration
int JOINT_LIMIT_MIN[] = {-170, -42, -89, -180, -105, -180, 0, 0, 0}; // NB! ?
int JOINT_LIMIT_MAX[] = {170, 90, 52, 180, 105, 180, 3450, 3450, 3450}; // NB! ?

///////////////////////////////////////////////////////////////////////////////
// ROS Driver Params
///////////////////////////////////////////////////////////////////////////////

// roughly equals 0, 0, 0, 0, 0, 0 degrees
const int REST_ENC_POSITIONS[] = {75556, 23333, 49444, 89600, 11477, 40000, 0};
enum SM { STATE_TRAJ, STATE_ERR };
SM STATE = STATE_TRAJ;

const int NUM_JOINTS = 6; // NB !  <- from param !?

AccelStepper stepperJoints[NUM_JOINTS];
AccelStepper LinearTrack;

// calibration settings
const int LIMIT_SWITCH_HIGH[] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1};  // to account for both NC and NO limit switches
const int CAL_DIR[] = {-1, -1, 1,
                       -1, -1, 1,
                       1, -1, -1}; // NB! ? // joint rotation direction to limit switch
const int CAL_SPEED = 500;          // motor steps per second
const int CAL_SPEED_MULT[] = {
    1, 1, 1, 2, 1, 1, 1, 1, 1};  // multiplier to account for motor steps/rev

// speed and acceleration settings
float JOINT_MAX_SPEED[] = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 100.0, 30.0, 30.0};  // deg/s
float JOINT_MAX_ACCEL[] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 30.0, 10.0, 10.0};  // deg/s^2
char JOINT_NAMES[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};

// num of encoder steps in range of motion of joint
int ENC_RANGE_STEPS[NUM_JOINTS];

const int PANIC_BTN = 39;
bool PanicSTop = false;
unsigned long PanicStopCooldown = 0;

bool PanicBtnPressed() {
    static elapsedMillis debounceTimer;    // timer to implement debounce lockout period
    unsigned int debounceDelay = 50;      // duration of the debounce lockout period (ms)
    static bool debounceLockout = false;   // true during enforcement of the debounce lockout
    static int oldLevel;                   // last switch input reading
    int newLevel;                          // current switch input value
    bool edgeDetected;                     // set true if a falling or rising edge is detected
    bool newSwitchPress;                   // set true if a falling edge is detected

    newSwitchPress = false;                // initialize to false, may be set true later
    if (debounceLockout) {                 // if currently in the debounce lockout period
        if (debounceTimer > debounceDelay) // and if the lockout delay has expired
            debounceLockout = false;       // clear this flag to indicate end of lockout
    }
    else {                                 // not in the lockout period
        newLevel = digitalRead(PANIC_BTN); // read the current switch level
        edgeDetected = (newLevel != oldLevel); // edge = diff between current and previous values
        oldLevel = newLevel;               // current level becomes previous value for next time around
        if (edgeDetected) {                // if an edge has been detected (rising or falling)
            debounceTimer = 0;             // start the lockout period
            debounceLockout = true;        // update the switch state to indicate the lockout is in effect
            if (newLevel == LOW)           // falling edge was detected
                newSwitchPress = true;     // so switch was pressed
        }
    }
    if (newSwitchPress){
      //Serial8.println("panic pressed");
    }
    return newSwitchPress;                 // return value is true only if a new switch press is detected
}

void PanicStopSteppers(){
 if (PanicStopCooldown == 0){
  //Serial8.println("Panic stop");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i].stop();
    stepperJoints[i].run();
      }
      LinearTrack.stop();
      LinearTrack.run();
  PanicStopCooldown = millis();
  }
}

unsigned long debugprint = millis();

void setup() {
  Serial.begin(9600);
//Serial8.begin(9600);
//Serial8.println("init");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(LIMIT_PINS[i], INPUT);

    int joint_range = JOINT_LIMIT_MAX[i] - JOINT_LIMIT_MIN[i];
    ENC_RANGE_STEPS[i] =
        static_cast<int>(MOTOR_STEPS_PER_DEG[i] * joint_range * ENC_MULT[i]);
  }

    pinMode(PANIC_BTN, INPUT_PULLUP);
}

bool initStateTraj(String inData) {
  // parse initialisation message
  int idxVersion = inData.indexOf('A');
  String softwareVersion =
      inData.substring(idxVersion + 1, inData.length() - 1);
  int versionMatches = (softwareVersion == VERSION);

  // return acknowledgement with result
  String msg = String("ST") + "A" + versionMatches + "B" + VERSION;
  Serial.println(msg);

  return versionMatches ? true : false;
}

void readMotorSteps(int* motorSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    motorSteps[i] = encPos[i].read() / ENC_MULT[i];
  }
}

void encStepsToJointPos(int* encSteps, double* jointPos) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointPos[i] = encSteps[i] / MOTOR_STEPS_PER_DEG[i] * ENC_DIR[i];
  }
}

void jointPosToEncSteps(double* jointPos, int* encSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    encSteps[i] = jointPos[i] * MOTOR_STEPS_PER_DEG[i] * ENC_DIR[i];
  }
}

String JointPosToString(double* jointPos) {
  String out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += JOINT_NAMES[i];
    out += String(jointPos[i], 6);
  }
  return out;
}

void updateStepperSpeed(String inData) {
  int idxSpeedJ1 = inData.indexOf('A');
  int idxAccelJ1 = inData.indexOf('B');
  int idxSpeedJ2 = inData.indexOf('C');
  int idxAccelJ2 = inData.indexOf('D');
  int idxSpeedJ3 = inData.indexOf('E');
  int idxAccelJ3 = inData.indexOf('F');
  int idxSpeedJ4 = inData.indexOf('G');
  int idxAccelJ4 = inData.indexOf('H');
  int idxSpeedJ5 = inData.indexOf('I');
  int idxAccelJ5 = inData.indexOf('J');
  int idxSpeedJ6 = inData.indexOf('K');
  int idxAccelJ6 = inData.indexOf('L');

  JOINT_MAX_SPEED[0] = inData.substring(idxSpeedJ1 + 1, idxAccelJ1).toFloat();
  JOINT_MAX_ACCEL[0] = inData.substring(idxAccelJ1 + 1, idxSpeedJ2).toFloat();
  JOINT_MAX_SPEED[1] = inData.substring(idxSpeedJ2 + 1, idxAccelJ2).toFloat();
  JOINT_MAX_ACCEL[1] = inData.substring(idxAccelJ2 + 1, idxSpeedJ3).toFloat();
  JOINT_MAX_SPEED[2] = inData.substring(idxSpeedJ3 + 1, idxAccelJ3).toFloat();
  JOINT_MAX_ACCEL[2] = inData.substring(idxAccelJ3 + 1, idxSpeedJ4).toFloat();
  JOINT_MAX_SPEED[3] = inData.substring(idxSpeedJ4 + 1, idxAccelJ4).toFloat();
  JOINT_MAX_ACCEL[3] = inData.substring(idxAccelJ4 + 1, idxSpeedJ5).toFloat();
  JOINT_MAX_SPEED[4] = inData.substring(idxSpeedJ5 + 1, idxAccelJ5).toFloat();
  JOINT_MAX_ACCEL[4] = inData.substring(idxAccelJ5 + 1, idxSpeedJ6).toFloat();
  JOINT_MAX_SPEED[5] = inData.substring(idxSpeedJ6 + 1, idxAccelJ6).toFloat();
  JOINT_MAX_ACCEL[5] = inData.substring(idxAccelJ6 + 1).toFloat();
}

void calibrateJoints(int* calJoints) {
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calJointsDone[i] = !calJoints[i];
  }

  // first pass of calibration, fast speed
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }


  while ((!PanicSTop)&&(!calAllDone)) {
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      // if joint is not calibrated yet
      if (!calJointsDone[i]) {
        // check limit switches
        if (!reachedLimitSwitch(i)) {
          // limit switch not reached, continue moving
          stepperJoints[i].runSpeed();
          calAllDone = false;
        } else {
          // limit switch reached
          stepperJoints[i].setSpeed(0);  // redundancy
          calJointsDone[i] = true;
        }
      }
    }
  PanicSTop = PanicSTop || PanicBtnPressed();
  }
  if (!PanicSTop){
    delay(2000);
  }
  return;
}

void moveAwayFromLimitSwitch() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] * -1);
  }

  for (int j = 0; j < 10000000; j++) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].runSpeed();
    }
  }
  // redundancy
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(0);
  }
  delay(2000);
  return;
}

bool reachedLimitSwitch(int joint) {
  int pin = LIMIT_PINS[joint];
  // check multiple times to deal with noise
  // possibly EMI from motor cables?
  for (int i = 0; i < 5; ++i) {
    if (digitalRead(pin) != LIMIT_SWITCH_HIGH[joint]) {
      return false;
    }
  }
  return true;
}

void stateTRAJ() {
  // clear message
  String inData = "";

  // initialise joint steps
  double curJointPos[NUM_JOINTS];
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps);

  double cmdJointPos[NUM_JOINTS+1];
  int cmdEncSteps[NUM_JOINTS+1];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    cmdEncSteps[i] = curMotorSteps[i];
  }

  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[i]);
    stepperJoints[i].setMinPulseWidth(10);
  }


    LinearTrack = AccelStepper(1, STEP_PINS[6], DIR_PINS[6]);
    LinearTrack.setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    LinearTrack.setAcceleration(JOINT_MAX_ACCEL[6] *
                                     MOTOR_STEPS_PER_DEG[6]);
    LinearTrack.setMaxSpeed(JOINT_MAX_SPEED[6] * MOTOR_STEPS_PER_DEG[6]);
    LinearTrack.setMinPulseWidth(10);

  // start loop
  while (STATE == STATE_TRAJ) {
    char received = '\0';
    // check for message from host
    if (Serial.available()) {
      received = Serial.read();
      inData += received;
    }

    // process message when new line character is received
    if ((received == '\n') && (!PanicSTop)) {

//        Serial8.println(inData.c_str());

      String function = inData.substring(0, 2);
      // update trajectory information
      if (function == "MT") {
        if (SIMUL){
                  SIMULPOS = "JP"+inData.substring(2);
                  Serial.print(SIMULPOS);
                  break;
        }

        readMotorSteps(curMotorSteps);

        // update host with joint positions
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos)+"G"+String( ENC_DIR[6]*LinearTrack.currentPosition()/100000.0 );

        Serial.println(msg);

        // get new position commands
        int msgIdxJ1 = inData.indexOf('A');
        int msgIdxJ2 = inData.indexOf('B');
        int msgIdxJ3 = inData.indexOf('C');
        int msgIdxJ4 = inData.indexOf('D');
        int msgIdxJ5 = inData.indexOf('E');
        int msgIdxJ6 = inData.indexOf('F');
        int msgIdxJ7 = inData.indexOf('G');

        cmdJointPos[0] = inData.substring(msgIdxJ1 + 1, msgIdxJ2).toFloat();
        cmdJointPos[1] = inData.substring(msgIdxJ2 + 1, msgIdxJ3).toFloat();
        cmdJointPos[2] = inData.substring(msgIdxJ3 + 1, msgIdxJ4).toFloat();
        cmdJointPos[3] = inData.substring(msgIdxJ4 + 1, msgIdxJ5).toFloat();
        cmdJointPos[4] = inData.substring(msgIdxJ5 + 1, msgIdxJ6).toFloat();
        if (msgIdxJ7 == -1){
        cmdJointPos[5] = inData.substring(msgIdxJ6 + 1).toFloat();
        } else {
        cmdJointPos[5] = inData.substring(msgIdxJ6 + 1, msgIdxJ7).toFloat();
        cmdJointPos[6] = inData.substring(msgIdxJ7 + 1).toFloat(); 
          //cmdEncSteps[6] = cmdJointPos[6] * MOTOR_STEPS_PER_DEG[6] * ENC_DIR[6];
cmdEncSteps[6] = ENC_DIR[6]*cmdJointPos[6]*100000;//*ENC_MUL[6]*ENC_DIR[6]/MOTOR_STEPS_PER_DEG[6];
        }//-1* 10 / 14 *
        // 0.14 ->  /    0.1 -> 100000
        // 10 steppiä on 14 mm
        // 1 step 0.14 mm
        // tuli 0.01 (eli 1 cm) -> 0.01* (ENC_MULT[6]/MOTOR_STEPS_PER_DEG[6] )
        // 1000 steps 0.141 
        // step/mm = 1000 / 0.14 
        jointPosToEncSteps(cmdJointPos, cmdEncSteps);

        // update target joint positions
        readMotorSteps(curMotorSteps);
        for (int i = 0; i < NUM_JOINTS; ++i) {
          int diffEncSteps = cmdEncSteps[i] - curMotorSteps[i];
          if (abs(diffEncSteps) > 2) {
            int diffMotSteps = diffEncSteps * ENC_DIR[i];
            // if (diffMotSteps < MOTOR_STEPS_PER_REV[i]) {
            //   // for the last rev of motor, introduce artificial
            //   decceleration
            //   // to help prevent overshoot
            //   diffMotSteps = diffMotSteps / 2;
            // }
            stepperJoints[i].move(diffMotSteps);
            stepperJoints[i].run();
          }
        }

        if (msgIdxJ7>=0){
          if ((cmdJointPos[6]>0.2) && (millis()-debugprint>100)){
            //Serial.printf("DB: G?? %s\n", inData.c_str());
            debugprint=millis();
          }
          //Serial.printf("DB: Linear track moveto %i / (%f)\n", cmdEncSteps[6],cmdJointPos[6]);
          LinearTrack.moveTo(cmdEncSteps[6]);
        }

      } else if (function == "CL"){ // calibrate linear track 
        while(!reachedLimitSwitch(6)){
        LinearTrack.setSpeed(CAL_SPEED*CAL_SPEED_MULT[6]*CAL_DIR[6]);
          LinearTrack.runSpeed();
           }
        LinearTrack.setSpeed(0);
        LinearTrack.setCurrentPosition(0);

        // move away from switch
        LinearTrack.setSpeed(CAL_SPEED * CAL_SPEED_MULT[6] * CAL_DIR[6] * -1);
        for (int j = 0; j < 10000000; j++) {
          LinearTrack.runSpeed();
        }
        LinearTrack.setSpeed(0);


      } else if (function == "JC") {
        // calibrate all joints
        int calJoints[] = {1, 1, 1, 1, 1, 1};
        calibrateJoints(calJoints);
        if (PanicSTop){ return; };

        // record encoder steps
        int calSteps[6];
        for (int i = 0; i < NUM_JOINTS; ++i) {
          calSteps[i] = encPos[i].read();
        }

        for (int i = 0; i < NUM_JOINTS; ++i) {
          encPos[i].write(ENC_RANGE_STEPS[i] * ENC_MAX_AT_ANGLE_MIN[i]);
        }

        // move away from the limit switches a bit so that if the next command
        // is in the wrong direction, the limit switches will not be run over
        // immediately and become damaged.
        moveAwayFromLimitSwitch();

        // return to original position
        for (int i = 0; i < NUM_JOINTS; ++i) {
          stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                           MOTOR_STEPS_PER_DEG[i]);
          stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                       MOTOR_STEPS_PER_DEG[i]);
        }

        bool restPosReached = false;
        while (!restPosReached) {
          restPosReached = true;
          readMotorSteps(curMotorSteps);

          for (int i = 0; i < NUM_JOINTS; ++i) {
            if (abs(REST_ENC_POSITIONS[i] / ENC_MULT[i] - curMotorSteps[i]) >
                10) {
              restPosReached = false;
              float target_pos =
                  (REST_ENC_POSITIONS[i] / ENC_MULT[i] - curMotorSteps[i]) *
                  ENC_DIR[i];
              stepperJoints[i].move(target_pos);
              stepperJoints[i].run();
            }
          }
        }

        // calibration done, send calibration values
        String msg = String("JC") + "A" + calSteps[0] + "B" + calSteps[1] +
                     "C" + calSteps[2] + "D" + calSteps[3] + "E" + calSteps[4] +
                     "F" + calSteps[5];
        Serial.println(msg);
      } else if (function == "JP") {
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);
        if (SIMUL){
                  Serial.print(SIMULPOS);
                  break;
        }
        String msg = String("JP") + JointPosToString(curJointPos)+"G"+String( ENC_DIR[6]*LinearTrack.currentPosition()/100000.0 );

        Serial.println(msg);
      } else if (function == "SS") {
        updateStepperSpeed(inData);
        // set motor speed and acceleration
        for (int i = 0; i < NUM_JOINTS; ++i) {
          stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                           MOTOR_STEPS_PER_DEG[i]);
          stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                       MOTOR_STEPS_PER_DEG[i]);
        }
        // read current joint positions
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);

        // update host with joint positions
        String msg = String("JP") + JointPosToString(curJointPos)+"G"+String( ENC_DIR[6]*LinearTrack.currentPosition()/100000.0 );
        Serial.println(msg);
      } else if (function == "ST") {
        if (!initStateTraj(inData)) {
          STATE = STATE_ERR;
          return;
        }
      }
      // clear message
      inData = "";
    } else if ((received == '\n') && (PanicSTop)){
       Serial.println("DB: Panic stopped. Please reset w/ button.");
    }

     if (PanicBtnPressed()){
      if ((PanicSTop) && (millis()-PanicStopCooldown>3000)){
        //Serial8.println("released");
        PanicSTop = false;
        PanicStopCooldown = 0;
      } else {
      PanicSTop = true;
      PanicStopSteppers();
      }
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].run();
    }
      LinearTrack.run();
  }
}

void stateERR() {
  // enter holding state
  for (int i = 0; i < NUM_JOINTS; ++i) {
    digitalWrite(STEP_PINS[i], LOW);
  }

  while (STATE == STATE_ERR) {
    Serial.println("DB: Unrecoverable error state entered. Please reset.");
    delay(1000);
  }
}

void loop() {
  STATE = STATE_TRAJ;

  switch (STATE) {
    case STATE_ERR:
      stateERR();
      break;
    default:
      stateTRAJ();
      break;
  }
}