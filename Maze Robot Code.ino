//Maze Robot Code, built for SPT323 Robotics Competition Class at The University of Advancing Technology
//Written by Tyler Widener, tylerwidenerlm@gmail.com

//Last Updated: 1/31/2023

//=====PINS=====
//Ultrasonic Sensor Pins
const int fTrigPin = 3;
const int fEchoPin = 5;
const int lTrigPin = 6;
const int lEchoPin = 9;
const int rTrigPin = 10;
const int rEchoPin = 11;

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

//More Variables Here:


/* ARCHIVED CODE VARIABLES
const int sampleSize = 20;
float inputArray[sampleSize];
*/

//========FUNCTIONS===========
//The following 3 functions scan in each direction. They are called each loop.
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

//fullScan() is the function that is called in-loop, and it handles everything that needs done.
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

//=========PRIMARY FUNCTIONS============
void setup() {
  pinMode(fTrigPin, OUTPUT);
  pinMode(fEchoPin, INPUT);
  pinMode(lTrigPin, OUTPUT);
  pinMode(lEchoPin, INPUT);
  pinMode(rTrigPin, OUTPUT);
  pinMode(rEchoPin, INPUT);
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  fullScan();
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