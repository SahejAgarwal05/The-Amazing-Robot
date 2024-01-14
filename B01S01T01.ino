#include <MeMCore.h>

/**************************** Global Definitions *****************************/
//Colour Sensor & IR
#define INPUT_A A0
#define INPUT_B A1
#define IR A2
#define LDR A3
#define LDR_DELAY 320 // stabilisation time (ms)
#define LDR_WAIT 10 // sampling interval
#define SAMPLES 10 // samples per LED per calibration
#define TIE_BREAKER 3

//IR Sensor
#define IR_STABILIZE 100
#define IR_RECEIVER A2

//Ultrasonic
#define TIMEOUT 1600
#define SPEED_OF_SOUND 342
#define ULTRASONIC 12

//Motor Variables
#define TURNING_TIME 389
#define TURN_WAIT 200
MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);
uint8_t motorSpeed = 200;
int targetDistance = 12;

// PID Terms
double Kp = 12.0;
double Kd = 4;

double previousError = 0;
double integral = 0;
long lastDistance = targetDistance;

//Line Sensor
MeLineFollower lineFinder(PORT_2); 

//Onboard LED
MeRGBLed led(0,30);
#define ONBOARD_LED 13

//Buzzer
MeBuzzer buzzer;

//Scaled Callibration
long white[3] = {839, 983, 952}; //0-1023 Values
long black[3] = {396, 887, 784};
long range[3] = {443, 96, 168};
long rgb[3] = {0, 0, 0};

void setup() {
  // When debugging
  Serial.begin(9600);

  //Colour Sensor Setup
  pinMode(INPUT_A, OUTPUT);
  pinMode(INPUT_B, OUTPUT);
  pinMode(LDR, INPUT);

  //Onboard LED
  led.setpin(ONBOARD_LED);
}

void loop() {
  if(isLineDetected()) {
    //Perform Movement Based on Colour Detected
    stopMoving();
    long colour = getColour();
    if(colour == 0) {
      led.setColor(255, 0, 0);

      turnLeft();
    } else if(colour == 1) {
      led.setColor(102, 0, 255);

      turnLeft();
      moveForward();
      delay(900);
      stopMoving();
      turnLeft();
    } else if(colour == 2) {
      led.setColor(255, 155, 0);

      UTurn();
    } else if(colour == 3) {
      led.setColor(0, 255, 0);

      turnRight();
    } else if(colour == 4) {
      led.setColor(0, 0, 255);

      turnRight();
      moveForward();
      delay(900);
      stopMoving();
      turnRight();
    } else if(colour == 5) {
      led.setColor(255, 255, 255);
      stopMoving();
      celebrate();
    }
    led.show();
  } else {
    //PID Straight Line Algorithm
    float distance = getDistance();
    if (distance <= 0 || distance >= 17) {
      float IRDist = getIRDistance();
      if(IRDist <= 0 || IRDist >= 12) {
        //travelling "blind"
        distance = lastDistance;
      } else {
        //Use IR Reading
        distance = IRDist;
      }
    }

    // Calculate PID terms
    double error = targetDistance - distance;
    integral += error;
    double derivative = error - previousError;

    int leftSpeed = motorSpeed + (Kp * error + Kd * derivative);
    int rightSpeed = motorSpeed - (Kp * error + Kd * derivative);

    if (leftSpeed > 255) leftSpeed = 255;
    if (rightSpeed > 255) rightSpeed = 255;
    if (leftSpeed < -255) leftSpeed = -255;
    if (rightSpeed < -255) rightSpeed = -255;

    leftMotor.run(-leftSpeed);
    rightMotor.run(rightSpeed);

    // Update previous error
    previousError = error;

    delay(50);
  }
}

/********************** Line Sensor Functions **************************/
//Line Sensor
bool isLineDetected() {
  int sensorState = lineFinder.readSensors();
  return sensorState == S1_OUT_S2_IN || sensorState == S1_IN_S2_OUT || sensorState == S1_IN_S2_IN;
}

/********************** Ultrasonic Distance Functions **************************/
//Ultrasonic Sensor
float getDistance() {
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC, INPUT);
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);

  return (float) duration * SPEED_OF_SOUND * 0.0001 / 2;
}

/******************************* Basic Movement Functions ********************************/
//Basic Movement
void moveForward() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed);
}

void turnLeft() {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(TURNING_TIME);
  stopMoving();
  delay(TURN_WAIT);
}

void UTurn() {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  delay(TURNING_TIME * 2 - 5);
  stopMoving();
  delay(TURN_WAIT);
}

void turnRight() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(TURNING_TIME);
  stopMoving();
  delay(TURN_WAIT);
}

void stopMoving() {
  leftMotor.stop();
  rightMotor.stop();
}

/*********************************** Colour Sensor Functions *******************************************/
//getColour classifies the color and returns an integer 0 to 5;
long getColour() {
  long best_of_N[TIE_BREAKER] = {0, };
  long colour = 0;
  long count = 0;
  long freq[6] = {0, };
  for(long i = 0; i < TIE_BREAKER; i += 1) {
    calibrate_LDR();
    best_of_N[i] = classifyColour(rgb);
    freq[best_of_N[i]] += 1;

    if(freq[best_of_N[i]] > count) {
      count = freq[best_of_N[i]];
      colour = best_of_N[i];
    }
  }

  return colour;
}

// Helper function for getColour()
// Returns best of N colour value from 0 to 5 for each colour on maze
long classifyColour(long rgb[3]) {
   // R^2 Error
  long colour[6][3] = {
    {248, 98, 71}, //red - 0-255 Values
    {158, 159, 194}, //purple
    {255, 167, 86}, //orange
    {84, 175, 119}, //green
    {113, 212, 238}, //blue
    {255, 255, 255}, //white
  };

  long best = 0;
  long min_error = 1000000;
  for(long i = 0; i < 6; i += 1) {
    long squared_error = (rgb[0] - colour[i][0]) * (rgb[0] - colour[i][0]) + (rgb[1] - colour[i][1]) * (rgb[1] - colour[i][1]) + (rgb[2] - colour[i][2]) * (rgb[2] - colour[i][2]);
    if(squared_error < min_error) {
      min_error = squared_error;
      best = i;
    }
  }
  return best;
}

// Helper function for getColour()
//calibrate_LDR will read the voltages from the samples and return the populated RGB array with averages
/*
  {INPUT_A, INPUT_B}
  {HIGH, LOW} - RED
  {LOW, HIGH} - GREEN
  {LOW, LOW} - BLUE
*/
void calibrate_LDR() {
  long val[3][2] = {
    {HIGH, LOW},
    {LOW, HIGH},
    {LOW, LOW}
  };

  for(long i = 0; i < 3; i += 1) {
    digitalWrite(INPUT_A, val[i][0]);
    digitalWrite(INPUT_B, val[i][1]);
    delay(LDR_DELAY);

    long sum = 0;
    for(long j = 0; j < SAMPLES; j += 1) {
      sum += analogRead(LDR);
      delay(LDR_WAIT);
    }
    
    long average = sum / SAMPLES;
    rgb[i] = (long) (((double) (average - black[i]) / (double) (range[i])) * 255.0);
    rgb[i] = max(0, min(255, rgb[i]));
  }
}

/************************* IR Distance Function ****************************/
float getIRDistance() {
  // Sensor Read
  turnOnIR();
  delayMicroseconds(IR_STABILIZE);
  int sensorReading = analogRead(IR_RECEIVER);

  // Base Line Read
  turnOffIR();
  delayMicroseconds(IR_STABILIZE);
  int baseline = analogRead(IR_RECEIVER);

  return convertToDistance(baseline - sensorReading);
}

// getIRDistance helper function
// Function to convert analog sensor value to distance in centimeters
float convertToDistance(int val) {
  const float k = 1344.6;
  const float c = 73.774;

  return sqrt((k)/(val - c));
}

//getIRDistance helper functions
// turns the IR on and off respectively
void turnOnIR() {
  digitalWrite(INPUT_A, HIGH);
  digitalWrite(INPUT_B, HIGH);
}
void turnOffIR() {
  digitalWrite(INPUT_A, LOW); //turns on some LED instead
  digitalWrite(INPUT_B, LOW);
}

/********************** Celebration Function **************************/
//Celebration Tunes
void celebrate() {
  buzzer.tone(392, 200);
  buzzer.tone(523, 200);
  buzzer.tone(659, 200);
  buzzer.tone(784, 200);
  buzzer.tone(659, 150);
  buzzer.tone(784, 400);
  buzzer.noTone();
}