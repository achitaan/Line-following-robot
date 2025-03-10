// Robot Number: 17119
// Info: Servos seem to stop at 92-96 and move with a signal from 0-91 and 97-100
//--------------------------------------------------PREPARATION----------------------------------------

//Imports
#include <Servo.h>  
 
// Servo Setup
Servo leftServo;  
Servo rightServo;

int leftPin = 4;
int rightPin = 5;

int leftWheel = 94;
int rightWheel = 94;

int neutralSpeed = 100;
const int initializedSpeed = 100;

//PIN NUMBERS 
const byte rightSensor = A2;
const byte midSensor  = A1;
const byte leftSensor = A0;

// PID Variables:
float Kp = 1.1;
float Ki = 0.17;
float Kd = 1.4;

int P, I, D, error;
int prevError = 0;

// Delay
int dT = 30;

int counter = 0;    // Counts the iteration of robotRotations

//--------------------------------------------------SETUP----------------------------------------------
void setup() {
  leftServo.attach(leftPin);                                  //ATTACH SERVOS
  rightServo.attach(rightPin);
  Serial.begin(500000);                                       //SET SERIAL RATE
}

//-------------------------------------------------FUNCTIONS-------------------------------------------

int scaling(float x){
  // Returns the value with one digit removed
  return round(x / 10) * 10;
  }

void runMovement(int rightSpeed, int leftSpeed){
  // Runs the robot of at speed some speed X and some speed Y
  rightServo.write(rightSpeed);
  leftServo.write(leftSpeed);
}

void stopMoving(){ 
  // Stops the robot from running
  rightServo.write(94);
  leftServo.write(94);
}

void robotRotation(int theta){
  // Rotates the robot by theta degrees
  float t = abs(theta) / (60 / 0.46);
  
  runMovement(0, 180);
  delay(t * 100);
  stopMoving();
}


void PID_Control()  {
  // error = setpoint - actual, P = error, I += error, D = error - prevError

  // Difference between sensors
  error = round((scaling(analogRead(rightSensor)) - scaling(analogRead(leftSensor)))/100);
  
  P = error;                                                  // Proportional: Difference between what we want and wherewe are (Reacts and gets closer to the line)
  I += error;                                                 // Integral: Error added over the small change in time dT (Acounts for the past error)
  D = error - prevError;                                      // Derivative: Change in the error over the small time dT (avoids future overshoots)

  int PIDval = (Kp * P) + (Ki * I) + (Kd * D);                // PID control variable

  // Storing the current error
  prevError = error;

  // Apply the change to both wheels
  rightWheel = neutralSpeed + PIDval;
  leftWheel = neutralSpeed - PIDval;
}

void endReached(){
  switch(counter){  
      
    case 1:                                     // Second iteration, robotRotation around          
      robotRotation(180);  
                          
    case 2:                                     // Third iteration, robotRotation around
      robotRotation(180);  
    case 3:                                     // Fourth iteration, robotRotation around
      robotRotation(180);  
    case 4:                                     // Final iteration, robotRotation around and stop moing
        robotRotation(180); 
        stopMoving();  
        rightServo.detach();
        leftServo.detach();
  }
  counter += 1;
}

bool forwardState(int midL, int midH, int leftL, int leftH, int rightL, int rightH){
  // Reads the state 
  bool a, b, c, d, e, f;
  a = midL <= scaling(analogRead(midSensor));
  b = scaling(analogRead(midSensor)) < midH;
  
  c = leftL <= scaling(analogRead(leftSensor));
  d = scaling(analogRead(leftSensor)) < leftH;
  
  e = rightL <= scaling(analogRead(rightSensor));
  f = scaling(analogRead(rightSensor)) < rightH;

  return a && b && c && d && e && f;
}

void seekStart(){
  // While not all black
  while (forwardState(700, 1000, 0, 400, 0, 400) == false){
    runMovement(initializedSpeed, initializedSpeed);
    delay(50);
  }
  counter += 1;
}

void followLine(){
  int count = 0; 
  bool firstCondition = false;
  bool secondCondition = false;
  bool runLineFollow = false;
  bool isBWB = false; 
  
  while (not runLineFollow){

    PID_Control();
    
    testPrinting();

    runMovement(rightWheel, leftWheel);
    
    if (count == 0 && (forwardState(600, 1200, 600, 1200, 600, 1200) || forwardState(600, 1200, 600, 1200, 600, 1200))){
      // When it sees all black
      firstCondition = true; 
      count ++;
    }
    else if (count >= 3 and forwardState(0, 600, 0, 600, 0, 600)){
      // Checks to see white if enough time has passed
      secondCondition = true; 
      count = 0;
    }
    else if (count >= 1 && (not forwardState(0, 600, 600, 1200, 600, 1200) || not forwardState(600, 1200, 600, 1200, 600, 1200) || not forwardState(0, 600, 0, 600, 0, 600))){
      // If not on white or black, restart the sequence
      firstCondition = false;
      secondCondition = false;
      count = 0;

    }
    Serial.print("count:");
    Serial.println(count);
    if (firstCondition && not secondCondition && (forwardState(0, 600, 600, 1200, 600, 1200) || forwardState(600, 1200, 600, 1200, 600, 1200) || forwardState(0, 600, 0, 600, 0, 600))){
      count ++;
    }
    Serial.print("count:");
    Serial.println(count);
    runLineFollow = firstCondition && secondCondition; 

    delay(dT);
  }
}

void testPrinting(){
  Serial.print(analogRead(rightSensor));
  Serial.print(" ");
  Serial.print(analogRead(midSensor));
  Serial.print(" ");
  Serial.println(analogRead(leftSensor));


  Serial.print("Left: ");
  Serial.print(leftWheel);
  Serial.print(" Right: ");
  Serial.println(rightWheel);
}

//--------------------------------------------------LOOP-----------------------------------------------

void loop() {  //Basically just does some testing right now
  //seekStart();
  
  followLine();

  endReached();
}
