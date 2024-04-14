//Maze Robot Code, built for SPT323 Robotics Competition Class at The University of Advancing Technology
//Robot Designated as M. A. C. (Maze Analyzing Car)
//Primary function is to just navigate around obstacles
//Written by Tyler Widener, tylerwidenerlm@gmail.com

//Last Updated: 4/25/2023
//Current version is for final!
//The robot is programmed to find and travel to the center of a 4ft square arena.
//  It does this in 3 steps:
//    1: Drive until the robot is 10cm from a wall in front.
//    2: Turn right, and drive until the robot is 10cm from a wall, thereby putting in the corner.
//    3: Turn right 135 degrees, and drive a set distance to reach the center of the box.

//=====PINS=====
//Ultrasonic Sensor Pins
const int fTrigPin = A4;
const int fEchoPin = 5;
const int lTrigPin = A2;
const int lEchoPin = 9;
const int rTrigPin = 10;
const int rEchoPin = 11;

//Motor Pins
//NOTE: 1 is the forward signal, 2 is the reverse signal. Min power is ~120, max power is 255.
//NOTE: 1 pulse = 1.144mm. 1 wheel rotation = 115 pulses = 100.5mm
//  Rotation 90 degrees = 137 pulses
//UPDATE: Motors will use 1 digital pin and 1 analog pin each. For speed control.
//  1 is the speed control pin.
//  2 is the directional pin:
//    LOW is FORWARD
//    HIGH is BACKWARD
//  Setting 1 to PWM gives speed control. The higher the number, the greater the speed.
//    Reversing the wheel uses the REVERSE of the PWM signal: 200 is actually 55 for reverse.
const int lMotor1 = 6;
const int lMotor2 = A3;
const int rMotor1 = 3;
const int rMotor2 = A5;

//Encoder Channels
const int lA = 2;
const int lB = 7;
const int rA = 12;
const int rB = 4;

//=====CODE VARIABLES======
//For Scanning Functions:
double duration;
double distance;
int input;

//These 3 variables are for varying sample array size.
int fCurrSize = 10;
int lCurrSize = 10;
int rCurrSize = 10;

//These 3 variables always equal the distance each sensor detects, refreshed each loop. They do not need to be changed outside the scanning functions.
double frontDistance = 0.0;
double rightDistance = 0.0;
double leftDistance = 0.0;

//For Encoders:
int directionL = 0; //Left Motor - 0 for rest, 1 for clockwise, 2 for counterclockwise
int directionR = 0; //Right Motor - 0 for rest, 1 for clockwise, 2 for counterclockwise
int stateLA = 0; //Variables for the current and last states of both encoder channels on each motor
int lastStateLA = 0;
int stateLB = 0;
int lastStateLB = 0;
int stateRA = 0;
int lastStateRA = 0;
int stateRB = 0;
int lastStateRB = 0;
float rightMotorOffset = 0.0; //How much the motor is offset. The one set to 0 during runtime is faster (normally right), while the other will be a decimal.
float leftMotorOffset = 0.0;
int offsetSpd = 0;

//More Variables Here:

/* ARCHIVED CODE VARIABLES
const int sampleSize = 20;
float inputArray[sampleSize];
*/

//========FUNCTIONS===========
//The following 3 functions scan in each direction. Best when called repeatedly.
double forwardScan() {
  const int rangeConstant = 5;
  double range = 0;
  double minDist = 0;
  double maxDist = 0;
  int currSize = 10;
  double readings[100];
  currSize = fCurrSize;
  for (int i = 0; i <= (currSize - 1); i++) { //For loop iterates through the array, currSize - 1 times
    readings[i] = scan(fTrigPin, fEchoPin); //Fill the array with a value
    if (i == 0) { //If it's the first loop, set the initial minimum and maximum values
      minDist = readings[i];
      maxDist = readings[i];
    }
    else {
      if (readings[i] < minDist) { //If the current reading is less than the lowest recorded value, replace it
        minDist = readings[i];
      }
      if (readings[i] > maxDist) { //If the current reading is greater than the highest recorded value, replace it
        maxDist = readings[i];
      }
    }
  }
  range = maxDist - minDist; //Calculate the range of the samples
  
  distance = 0;
  for (int i = 0; i <= (currSize - 1); i++) { //For loop to calculate average
    distance += readings[i];
  }
  distance = distance / currSize;
  
  Serial.print("0, "); //Print statements
  Serial.println(distance);

  currSize = 5 * range; //Sets the next sample size to be the range constant multiplied by the range itself
  if (currSize >= 100) { //If statements to catch over or under-sized arrays
    currSize = 100;
  }
  if (currSize <= 2) {
    currSize = 2;
  }
  fCurrSize = currSize;
  for (int i = 0; i <= (currSize - 1); i++) {
    readings[i] = 0;
  }
  return distance;
}

double leftScan() {
  const int rangeConstant = 5;
  double range = 0;
  double minDist = 0;
  double maxDist = 0;
  int currSize = 10;
  double readings[100];
  currSize = lCurrSize;
  for (int i = 0; i <= (currSize - 1); i++) { //For loop iterates through the array, currSize - 1 times
    readings[i] = scan(lTrigPin, lEchoPin); //Fill the array with a value
    if (i == 0) { //If it's the first loop, set the initial minimum and maximum values
      minDist = readings[i];
      maxDist = readings[i];
    }
    else {
      if (readings[i] < minDist) { //If the current reading is less than the lowest recorded value, replace it
        minDist = readings[i];
      }
      if (readings[i] > maxDist) { //If the current reading is greater than the highest recorded value, replace it
        maxDist = readings[i];
      }
    }
  }
  range = maxDist - minDist; //Calculate the range of the samples
  
  distance = 0;
  for (int i = 0; i <= (currSize - 1); i++) { //For loop to calculate average
    distance += readings[i];
  }
  distance = distance / currSize;
  
  Serial.print("0, "); //Print statements
  Serial.println(distance);

  currSize = 5 * range; //Sets the next sample size to be the range constant multiplied by the range itself
  if (currSize >= 100) { //If statements to catch over or under-sized arrays
    currSize = 100;
  }
  if (currSize <= 2) {
    currSize = 2;
  }
  lCurrSize = currSize;
  for (int i = 0; i <= (currSize - 1); i++) {
    readings[i] = 0;
  }
  return distance;
}

double rightScan() {
  const int rangeConstant = 5;
  double range = 0;
  double minDist = 0;
  double maxDist = 0;
  int currSize = 10;
  double readings[100];
  currSize = rCurrSize;
  for (int i = 0; i <= (currSize - 1); i++) { //For loop iterates through the array, currSize - 1 times
    readings[i] = scan(rTrigPin, rEchoPin); //Fill the array with a value
    if (i == 0) { //If it's the first loop, set the initial minimum and maximum values
      minDist = readings[i];
      maxDist = readings[i];
    }
    else {
      if (readings[i] < minDist) { //If the current reading is less than the lowest recorded value, replace it
        minDist = readings[i];
      }
      if (readings[i] > maxDist) { //If the current reading is greater than the highest recorded value, replace it
        maxDist = readings[i];
      }
    }
  }
  range = maxDist - minDist; //Calculate the range of the samples
  
  distance = 0;
  for (int i = 0; i <= (currSize - 1); i++) { //For loop to calculate average
    distance += readings[i];
  }
  distance = distance / currSize;
  
  Serial.print("0, "); //Print statements
  Serial.println(distance);

  currSize = 5 * range; //Sets the next sample size to be the range constant multiplied by the range itself
  if (currSize >= 100) { //If statements to catch over or under-sized arrays
    currSize = 100;
  }
  if (currSize <= 2) {
    currSize = 2;
  }
  rCurrSize = currSize;
  for (int i = 0; i <= (currSize - 1); i++) {
    readings[i] = 0;
  }
  return distance;
}

//fullScan() is the function that can be called in-loop, and it handles everything that needs done.
void fullScan() {
  frontDistance = forwardScan();
  leftDistance = leftScan();
  rightDistance = rightScan();
  Serial.print("0, ");
  Serial.print(frontDistance);
  Serial.print(", ");
  Serial.print(leftDistance);
  Serial.print(", ");
  Serial.println(rightDistance);
}

//Detects initial movement from rest for entire system
void detectStart() {
  stateLA = digitalRead(lA); //Set state variables to current channel states
  stateLB = digitalRead(lB);
  stateRA = digitalRead(rA);
  stateRB = digitalRead(rB);
  //Evaluate channels
  if ((stateLA != lastStateLA) && (directionL == 0)) { //If the Left A Signal Changes
    directionL = 1;
    Serial.println("Left Clockwise");
  }
  if ((stateLB != lastStateLB) && (directionL == 0)) { //If the Left B Signal Changes
    directionL = 2;
    Serial.println("Left Counterclockwise");
  }
  if ((stateRA != lastStateRA) && (directionR == 0)) { //If the Right A Signal Changes
    directionR = 1;
    Serial.println("Right Clockwise");
  }
  if ((stateRB != lastStateRB) && (directionR == 0)) { //If the Right B Signal Changes
    directionR = 2;
    Serial.println("Right Counterclockwise");
  }
  lastStateLA = stateLA;
  lastStateLB = stateLB;
  lastStateRA = stateRA;
  lastStateRB = stateRB;
}

//Waits for motors to come to a complete stop
void detectStop() {
  bool restLeft = true;
  bool restRight = true;
  const int delayTime = 2;
  while(restLeft || restRight) {
    stateLA = digitalRead(lA);
    stateLB = digitalRead(lB);
    stateRA = digitalRead(rA);
    stateRB = digitalRead(rB);
    if (stateLA == lastStateLA) {
      delay(delayTime);
      stateLA = digitalRead(lA);
      if (stateLA == lastStateLA) {
        delay(delayTime);
        stateLA = digitalRead(lA);
        if (stateLA == lastStateLA) {
          delay(delayTime);
          stateLA = digitalRead(lA);
          if (stateLA == lastStateLA) {
            restLeft = false;
          }
          else {
            restLeft = true;
          }
        }
        else {
          restLeft = true;
        }
      }
      else {
        restLeft = true;
      }
    }
    else {
      restLeft = true;
    }
    if (stateRA == lastStateRA) {
      delay(delayTime);
      stateRA = digitalRead(rA);
      if (stateRA == lastStateRA) {
        delay(delayTime);
        stateRA = digitalRead(rA);
        if (stateRA == lastStateRA) {
          delay(delayTime);
          stateRA = digitalRead(rA);
          if (stateRA == lastStateRA) {
            restRight = false;
          }
          else {
            restRight = true;
          }
        }
        else {
          restRight = true;
        }
      }
      else {
        restRight = true;
      }
    }
    else {
      restRight = true;
    }
    lastStateLA = stateLA;
    lastStateLB = stateLB;
    lastStateRA = stateRA;
    lastStateRB = stateRB;
  }
  directionL = 0;
  directionR = 0;
}

//Function for counting pulses, specifically right motor going forwards for now
void pulseTest() {
  int pulses = 0;
  drive(2, 'B', 90);
  //analogWrite(lMotor1, 120);
  while (pulses <= 50) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  int countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  //analogWrite(lMotor1, 0);
}

//Drive function for motors. 
//  mot represents the motors to be driven. 1 is for left, 2 is for right.
//      New Options: 3 drives both motors forward. 4 drives both motors backward. Uses motor offset.
//  dir represents direction. F is for forwards, B is for backwards, 0 turns the motor off.
//  spd represents speed, inputted as a percentage. Reversed percentage is accounted for automatically.
//      Updated: If you are using option 3 or 4, spd becomes irrelevant.
void drive(int mot, char dir, int spd) {
  spd = map(spd, 0, 100, 0, 255);
  if (dir == '0') {
    if (mot == 1) {
      digitalWrite(lMotor1, 0);
      digitalWrite(lMotor2, 0);
    }
    if (mot == 2) {
      digitalWrite(rMotor1, 0);
      digitalWrite(rMotor2, 0);
    }
    return;
  }
  else if (dir == 'F') {
    if (mot == 1) {
      analogWrite(lMotor1, spd);
      digitalWrite(lMotor2, 0);
    }
    if (mot == 2) {
      analogWrite(rMotor1, spd);
      digitalWrite(rMotor2, 0);
    }
    if (mot == 3) {
      if (leftMotorOffset == 0.0) {
        spd = 200 * rightMotorOffset;
        analogWrite(lMotor1, 200);
        analogWrite(rMotor1, spd);
        digitalWrite(lMotor2, 0);
        digitalWrite(rMotor2, 0);
      }
      if (rightMotorOffset == 0.0) {
        spd = 200 * leftMotorOffset;
        analogWrite(lMotor1, spd);
        analogWrite(rMotor1, 200);
        digitalWrite(lMotor2, 0);
        digitalWrite(rMotor2, 0);
      }
    }
    return;
  }
  else if (dir == 'B') {
    spd = ((spd - 255) * -1);
    if (mot == 1) {
      analogWrite(lMotor1, spd);
      digitalWrite(lMotor2, 1);
    }
    if (mot == 2) {
      analogWrite(rMotor1, spd);
      digitalWrite(rMotor2, 1);
    }
    if (mot == 4) {
      if (leftMotorOffset == 0.0) {
        spd = 55 * rightMotorOffset;
        analogWrite(lMotor1, 55);
        analogWrite(rMotor1, spd);
        digitalWrite(lMotor2, 1);
        digitalWrite(rMotor2, 1);
      }
      if (rightMotorOffset == 0.0) {
        spd = 55 * leftMotorOffset;
        analogWrite(lMotor1, spd);
        analogWrite(rMotor1, 55);
        digitalWrite(lMotor2, 1);
        digitalWrite(rMotor2, 1);
      }
    }
    return;
  }
}

//Function for calibrating motors so the robot drives straight. Based off loops between pulses.
void calibrate() {
  Serial.println("Calibrating...");
  int pulses = 0;
  const int testLimit = 500;
  const int testPulses = 150;
  int rightTimes[200];
  int leftTimes[200];
  int rightCount = 0;
  int leftCount = 0;
  int rightLoops = 0;
  int leftLoops = 0;
  double rightAVG = 0.0;
  double leftAVG = 0.0;
  double rightOffsets[10];
  double leftOffsets[10];
  for (int j = 0; j <= 9; j++) {
    //analogWrite(lMotor1, 120);
    rightCount = 0;
    leftCount = 0;
    rightLoops = 0;
    leftLoops = 0;
    rightAVG = 0.0;
    leftAVG = 0.0;
    for (int i = 0; i <= 200; i++) {
      rightTimes[i] = 0;
      leftTimes[i] = 0;
    }
    Serial.println();
    delay(250);
    pulses = 0;
    drive(2, 'B', 90);
    while (pulses <= testPulses) {
      stateRA = digitalRead(rA);
      if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
        pulses++;
        if (rightCount < testLimit) {
          rightTimes[rightCount] = rightLoops;
          rightLoops = 0;
          rightCount++;
        }
      }
      else if (rightCount < testLimit) {
        rightLoops++;
      }
      lastStateRA = stateRA;
    }
    drive(2, '0', 0);
    int countStop = 0;
    while (countStop <= 1000) {
      stateRA = digitalRead(rA);
      if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
        pulses++;
        countStop = 0;
        if (rightCount < testLimit) {
          rightTimes[rightCount] = rightLoops;
          rightLoops = 0;
          rightCount++;
        }
      }
      else if (rightCount < testLimit) {
        rightLoops++;
      }
      if (stateRA == lastStateRA) {
        countStop++;
      }
      lastStateRA = stateRA;
    }
    Serial.println(pulses);

    pulses = 0;
    drive(1, 'B', 90);
    //analogWrite(lMotor1, 120);
    while (pulses <= testPulses) {
      stateLA = digitalRead(lA);
      if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Left A Signal Changes
        pulses++;
        if (leftCount < testLimit) {
          leftTimes[leftCount] = leftLoops;
          leftLoops = 0;
          leftCount++;
        }
      }
      else if (leftCount < testLimit) {
        leftLoops++;
      }
      lastStateLA = stateLA;
    }
    drive(1, '0', 0);
    countStop = 0;
    while (countStop <= 1000) {
      stateLA = digitalRead(lA);
      if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Left A Signal Changes
        pulses++;
        countStop = 0;
        if (leftCount < testLimit) {
          leftTimes[leftCount] = leftLoops;
          leftLoops = 0;
          leftCount++;
        }
      }
      else if (leftCount < testLimit) {
        leftLoops++;
      }
      if (stateLA == lastStateLA) {
        countStop++;
      }
      lastStateLA = stateLA;
    }
    Serial.println(pulses);

    for (int i = 0; i <= rightCount; i++) { //Add up all collected values
      rightAVG = rightAVG + rightTimes[i];
    }
  
    Serial.print("Right Total: ");
    Serial.println(rightAVG);
    Serial.print("Right Count: ");
    Serial.println(rightCount);

    rightAVG = rightAVG / rightCount; //Divide to complete the average

    for (int i = 0; i <= leftCount; i++) { //Do the same for the left
      leftAVG = leftAVG + leftTimes[i];
    }

    Serial.print("Left Total: ");
    Serial.println(leftAVG);
    Serial.print("Left Count: ");
    Serial.println(leftCount);

    leftAVG = leftAVG / leftCount;

    Serial.print("Left AVG: ");
    Serial.println(leftAVG);
    Serial.print("Right AVG: ");
    Serial.println(rightAVG);

    if (rightAVG > leftAVG) { //If the right motor takes longer on average, meaning it is slower.
      leftMotorOffset = 0.0; //Left motor is fine, so set it to 0.
      rightOffsets[j] = rightAVG / leftAVG; //Set the right motor offset to the right avg / left avg, so the right motor moves offset times faster
    }
    else {
      rightMotorOffset = 0.0; //Right motor is fine.
      leftOffsets[j] = leftAVG / rightAVG; //Opposite for left motor.
    }
  }
  if (rightOffsets[0] > leftOffsets[0]) { //If the right motor takes longer on average, meaning it is slower.
    rightAVG = 0.0;
    leftMotorOffset = 0.0;
    for (int i = 0; i <= 10; i++) {
      rightAVG = rightAVG + rightOffsets[i];
    }
    rightAVG = rightAVG / 10;
    rightMotorOffset = rightAVG;
  }
  else {
    leftAVG = 0.0;
    rightMotorOffset = 0.0;
    for (int i = 0; i <= 10; i++) {
      leftAVG = leftAVG + leftOffsets[i];
    }
    leftAVG = leftAVG / 10;
    leftMotorOffset = leftAVG;
  }

  Serial.println();
  Serial.print("Left Offset: ");
  Serial.println(leftMotorOffset);
  Serial.print("Right Offset: ");
  Serial.println(rightMotorOffset);
}

//A setup function for the robot, that calls all pin modes and everything else the robot needs in setup()
void macSetup() {
  pinMode(fTrigPin, OUTPUT); //Pin Modes
  pinMode(fEchoPin, INPUT);
  pinMode(lTrigPin, OUTPUT);
  pinMode(lEchoPin, INPUT);
  pinMode(rTrigPin, OUTPUT);
  pinMode(rEchoPin, INPUT);
  pinMode(lMotor1, OUTPUT);
  pinMode(lMotor2, OUTPUT);
  pinMode(rMotor1, OUTPUT);
  pinMode(rMotor2, OUTPUT);
  pinMode(lA, INPUT);
  pinMode(lB, INPUT);
  pinMode(rA, INPUT);
  pinMode(rB, INPUT);
  Serial.begin(9600); // Starts the serial communication

  //Set initial rest values for motors
  stateLA = digitalRead(lA);
  lastStateLA = stateLA;
  stateLB = digitalRead(lB);
  lastStateLB = stateLB;
  stateRA = digitalRead(rA);
  lastStateRA = stateRA;
  stateRB = digitalRead(rB);
  lastStateRB = stateRB;

/* ARCHIVED CODE: Motor Calculated Offset
  delay(5000);
  calibrate();
  while((leftMotorOffset < 0.0) && (leftMotorOffset > 1.5) && (rightMotorOffset < 0.0) && (rightMotorOffset > 1.5)) {
    delay(1000);
    calibrate();
  }

  if (leftMotorOffset == 0.0) { //Several if statements to set up offset speed.
    offsetSpd = 200 * rightMotorOffset;
  }
  if (rightMotorOffset == 0.0) {
    offsetSpd = 200 * leftMotorOffset;
  }
  if (offsetSpd > 255) {
    offsetSpd = 255;
  }
*/
  leftMotorOffset = 1.05;
  rightMotorOffset = 0.0;
}

//===============EXTRA FUNCTIONS===================
double scan(int trigPin, int echoPin) { //Generic Function for Ultrasonic Sensor Scanning. Returns the distance.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  return (pulseIn(echoPin, HIGH) * 0.017);
}

//Function to drive the entire robot forward for the amount of pulses given in the argument. Returns the actual pulses traveled.
int drivePulses(int pulsesToDrive) {
  drive(3, 'F', 70);
  int pulses = 0;
  while (pulses <= (pulsesToDrive - 15)) { //-15 Pulses for attempted inertia compensation
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  drive(1, '0', 0);
  int countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  return pulses;
}

//Function to turn right 90 degrees.
void turnRight() {
  drive(2, 'F', 70);
  int pulses = 0;
  while (pulses <= 90) { //Pulse amount for rotation is 137, but giving an overhead for inertia.
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  int countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
}

//Function to turn left 90 degrees.
void turnLeft() {
  drive(1, 'F', 70);
  int pulses = 0;
  while (pulses <= 100) { //Pulse amount for rotation is 137, but giving an overhead for inertia.
    stateLA = digitalRead(lA);
    if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateLA = stateLA;
  }
  drive(1, '0', 0);
  int countStop = 0;
  while (countStop <= 1000) {
    stateLA = digitalRead(lA);
    if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateLA == lastStateLA) {
      countStop++;
    }
    lastStateLA = stateLA;
  }
  Serial.println(pulses);
}

void setup() {
  macSetup();

  //First, check the forward distance. If it is less than 10cm, reverse a bit.
  fullScan();
  if (frontDistance <= 10) {
    drive(4, 'B', 70);
    delay(500);
    drive(1, '0', 0);
    drive(2, '0', 0);
  }

  //While loop to cover step 1.
  while (true) {
    //Check the forward distance. If it is about 10cm, the first step is complete.
    fullScan();
    if (frontDistance > 9 && frontDistance < 11) {
      delay(500);
      fullScan();
      if (frontDistance > 9 && frontDistance < 11) {
        delay(500);
        fullScan();
        if (frontDistance > 9 && frontDistance < 11) {
          delay(500);
          fullScan();
          if (frontDistance > 9 && frontDistance < 11) {
            break;
          }
        }
      }
    }

    //Drive forward a short amount.
    delay(200);
    drivePulses(20);

    //Catching conditional for if the bot travels too far
    fullScan();
    if (frontDistance <= 9) {
      drive(4, 'B', 70);
      delay(500);
      drive(1, '0', 0);
      drive(2, '0', 0);
    }
  }

  turnRight();

  //While loop to cover step 2.
  while (true) {
    //Check the forward distance. If it is about 10cm, the first step is complete.
    fullScan();
    if (frontDistance > 9 && frontDistance < 11) {
      delay(500);
      fullScan();
      if (frontDistance > 9 && frontDistance < 11) {
        delay(500);
        fullScan();
        if (frontDistance > 9 && frontDistance < 11) {
          delay(500);
          fullScan();
          if (frontDistance > 9 && frontDistance < 11) {
            break;
          }
        }
      }
    }

    //Drive forward a short amount.
    delay(200);
    drivePulses(20);

    //Catching conditional for if the bot travels too far
    fullScan();
    if (frontDistance <= 9) {
      drive(4, 'B', 70);
      delay(500);
      drive(1, '0', 0);
      drive(2, '0', 0);
    }
  }

  drive(2, 'F', 70);
  delay(410);
  drive(2, '0', 0);

  leftMotorOffset = 1.15;
  delay(200);
  drivePulses(400);
}

void loop() {
  
}

/* ARCHIVED CODE: Figure Eight
  //NOTE: Latest test code is driving the robot in a figure eight of two square foot boxes.
  //  262 pulses = 30cm
  delay(5000);
  Serial.println("Driving...");

  //Drive forward 30cm
  //#1:
  drive(3, 'F', 70);
  int pulses = 0;
  while (pulses <= 247) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  drive(1, '0', 0);
  int countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  delay(2000);
  
  //Turn Right 90 Degrees
  drive(2, 'F', 70);
  pulses = 0;
  while (pulses <= 100) { //Pulse amount for rotation is 137, but giving an overhead for inertia.
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  
  //Drive forward 30cm
  //#2:
  drive(3, 'F', 70);
  pulses = 0;
  while (pulses <= 247) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  drive(1, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  delay(2000);
  
  //Turn Right 90 Degrees
  drive(2, 'F', 70);
  pulses = 0;
  while (pulses <= 100) { //Pulse amount for rotation is 137, but giving an overhead for inertia.
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);

  //Drive forward 30cm
  //#3:
  drive(3, 'F', 70);
  pulses = 0;
  while (pulses <= 247) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  drive(1, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  delay(2000);
  
  //Turn Right 90 Degrees
  drive(2, 'F', 70);
  pulses = 0;
  while (pulses <= 100) { //Pulse amount for rotation is 137, but giving an overhead for inertia.
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);

  //Drive forward 60cm
  drive(3, 'F', 70);
  pulses = 0;
  while (pulses <= 600) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  drive(1, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  delay(2000);

  //Turn left 90 degrees:
  drive(1, 'F', 70);
  pulses = 0;
  while (pulses <= 100) { //Pulse amount for rotation is 137, but giving an overhead for inertia.
    stateLA = digitalRead(lA);
    if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateLA = stateLA;
  }
  drive(1, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateLA = digitalRead(lA);
    if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateLA == lastStateLA) {
      countStop++;
    }
    lastStateLA = stateLA;
  }
  Serial.println(pulses);

  //Drive forward 30cm
  //#1:
  drive(3, 'F', 70);
  pulses = 0;
  while (pulses <= 247) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  drive(1, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  delay(2000);

  //Turn left 90 degrees:
  drive(1, 'F', 70);
  pulses = 0;
  while (pulses <= 100) { //Pulse amount for rotation is 137, but giving an overhead for inertia.
    stateLA = digitalRead(lA);
    if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateLA = stateLA;
  }
  drive(1, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateLA = digitalRead(lA);
    if ((stateLA != lastStateLA) && (stateLA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateLA == lastStateLA) {
      countStop++;
    }
    lastStateLA = stateLA;
  }
  Serial.println(pulses);

  //Drive forward 30cm
  //#2:
  drive(3, 'F', 70);
  pulses = 0;
  while (pulses <= 247) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
    }
    lastStateRA = stateRA;
  }
  drive(2, '0', 0);
  drive(1, '0', 0);
  countStop = 0;
  while (countStop <= 1000) {
    stateRA = digitalRead(rA);
    if ((stateRA != lastStateRA) && (stateRA == 1)) { //If the Right A Signal Changes
      pulses++;
      countStop = 0;
    }
    if (stateRA == lastStateRA) {
      countStop++;
    }
    lastStateRA = stateRA;
  }
  Serial.println(pulses);
  delay(2000);
  */
/* ARCHIVED CODE: Motor Testing Sequence
  analogWrite(rMotor1, 170); //Sequence to test motors. Should be done off the ground.
  detectStart();
  delay(1500);
  analogWrite(rMotor1, 0);
  detectStop();
  delay(1000);
  analogWrite(rMotor2, 170);
  detectStart();
  delay(1500);
  analogWrite(rMotor2, 0);
  detectStop();
  delay(1000);
  analogWrite(lMotor1, 170);
  detectStart();
  delay(1500);
  analogWrite(lMotor1, 0);
  detectStop();
  delay(1000);
  analogWrite(lMotor2, 170);
  detectStart();
  delay(1500);
  analogWrite(lMotor2, 0);
  detectStop();
  delay(1000);

  analogWrite(rMotor1, 255);
  analogWrite(lMotor1, 255);
  detectStart();
  delay(2000);
  analogWrite(rMotor1, 0);
  analogWrite(lMotor1, 0);
  detectStop();
  delay(2000);
  analogWrite(rMotor2, 255);
  analogWrite(lMotor2, 255);
  detectStart();
  delay(2000);
  analogWrite(rMotor2, 0);
  analogWrite(lMotor2, 0);
  detectStop();
  delay(10000);
*/
  /* ARCHIVED CODE FOR FIXED SAMPLE SIZE
        // Clears the trigPin
      for (int i = 0; i <= (sampleSize - 1); i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPin, HIGH);
        // Calculating the distance
        distance = duration;
        inputArray[i] = distance;
      }
      distance = 0;
        for (int i = 0; i <= (sampleSize - 1); i++) {
          distance += inputArray[i];
        }
        distance = (distance / sampleSize) * 0.017;
        // Prints the distance on the Serial Monitor
        //Serial.print("Distance: ");
        Serial.print("0, ");
        Serial.println(distance);
  *///END ARCHIVED CODE