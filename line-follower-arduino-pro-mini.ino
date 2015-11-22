// Line follower based on arduino pro mini
#include <Servo.h>
#include <SimpleTimer.h>

const int lineSensorServoPositionMinVal = 20;
const int lineSensorServoPositionMaxVal = 110;
const int lineSensorThresholdVal = 5;
const int lineServoSensorPinNo = 9;
const int lineSensorServoStep = 5;
const int leftMotorForwardPinNo = 10;
const int leftMotorBackwardPinNo = 11;
const int rightMotorForwardPinNo = 12;
const int rightMotorBackwardPinNo = 13;
const int leftMotorSpeedPinNo = 5;
const int rightMotorSpeedPinNo = 6;
const int rightLineSensorPinNo = 0; // Right part of the line sensor
const int middleLineSensorPinNo = 1; // Middle part of the line sensor
const int leftLineSensorPinNo = 2; // Left part of the line sensor
const int motorMinimumSpeedValue = 0;
const int motorMaximumSpeedValue = 255;

Servo lineSensorServo;  // Servo object to control servo of line sensor
int lineSensorServoPos = (lineSensorServoPositionMinVal + lineSensorServoPositionMaxVal) >> 1;
int trackFindingLineServoStep = lineSensorServoStep;
int fakeServoPosForSpiraleMove = lineSensorServoPositionMinVal;
unsigned long time;

//SimpleTimer timer;

// a function to be executed periodically
void repeatMe() {
}

void setup()  {
  Serial.begin(115200);
  lineSensorServo.attach(lineServoSensorPinNo);  // attaches the servo on pin 9 to the servo object
  lineSensorServo.write(lineSensorServoPos); // Turn servo to the initial position
  pinMode(leftMotorForwardPinNo, OUTPUT) ;
  pinMode(leftMotorBackwardPinNo, OUTPUT) ;
  pinMode(rightMotorForwardPinNo, OUTPUT) ;
  pinMode(rightMotorBackwardPinNo, OUTPUT) ;
  
  pinMode(leftMotorSpeedPinNo, OUTPUT) ;
  pinMode(rightMotorSpeedPinNo, OUTPUT) ;
  
  digitalWrite(leftMotorForwardPinNo, HIGH) ;
  digitalWrite(leftMotorBackwardPinNo, LOW) ;
  digitalWrite(rightMotorForwardPinNo, HIGH) ;
  digitalWrite(rightMotorBackwardPinNo, LOW) ;
  
  //timer.setInterval(1000, repeatMe);
}

void setMotorSpeedByServoPosition(int servoPos) {
  int middlePos = (lineSensorServoPositionMinVal + lineSensorServoPositionMaxVal) >> 1;
  int leftOrRightPart = servoPos - middlePos; // Set less speed for the right part if it has plus, for the left part otherwise
  int servoHalfRangeLength = (lineSensorServoPositionMaxVal - lineSensorServoPositionMinVal) >> 1;
  int leftOrRightPartPercent = leftOrRightPart * 100 / servoHalfRangeLength;
  int motorSpeedRangeLength = motorMaximumSpeedValue - motorMinimumSpeedValue;
  int motorSpeedRelativeValue = motorSpeedRangeLength * leftOrRightPartPercent / 100;
  int actualMotorSpeed = motorMaximumSpeedValue - abs(motorSpeedRelativeValue);
  
  Serial.println(actualMotorSpeed);
      
  if (motorSpeedRelativeValue > 0) {
    analogWrite(leftMotorSpeedPinNo, motorMaximumSpeedValue);
    analogWrite(rightMotorSpeedPinNo, actualMotorSpeed);
    Serial.println(actualMotorSpeed);
  }
  else {
    analogWrite(rightMotorSpeedPinNo, motorMaximumSpeedValue);
    analogWrite(leftMotorSpeedPinNo, actualMotorSpeed);
    Serial.println(actualMotorSpeed);
  }
}

void loop()  { 
  boolean lineSensorServoPosChanged = false;
  boolean utmostLineSensorsCrossTrack = false;
  boolean isOnTheField = true;
  boolean isTrackFind = false;
  
  //timer.run();

  int val=analogRead(rightLineSensorPinNo);
  if (val < lineSensorThresholdVal && lineSensorServoPos < lineSensorServoPositionMaxVal) {
    lineSensorServoPos += lineSensorServoStep;
    lineSensorServoPosChanged = true;
    utmostLineSensorsCrossTrack = true;
    lineSensorServoPos = min(lineSensorServoPositionMaxVal, lineSensorServoPos);
  }
  
  val=analogRead(leftLineSensorPinNo);
  if (val < lineSensorThresholdVal && lineSensorServoPos > lineSensorServoPositionMinVal) {
    if (!lineSensorServoPosChanged) {
      lineSensorServoPos -= lineSensorServoStep;
      lineSensorServoPosChanged = true;
      utmostLineSensorsCrossTrack = true;
      lineSensorServoPos = max(lineSensorServoPositionMinVal, lineSensorServoPos);
    }
    else {
      // In this case do nothing. It seems that line sensor does not see white (robot is turned out or is out of the playing field)
      lineSensorServoPosChanged = false;
      lineSensorServoPos -= lineSensorServoStep;
      isOnTheField = false;
    }
  }
  
  val=analogRead(middleLineSensorPinNo);
  if (val > lineSensorThresholdVal && !utmostLineSensorsCrossTrack) {
    // We are in the track finding mode
    lineSensorServoPos += trackFindingLineServoStep;
    if (lineSensorServoPos >= lineSensorServoPositionMaxVal ||
      lineSensorServoPos <= lineSensorServoPositionMinVal) {
      
        trackFindingLineServoStep = -trackFindingLineServoStep;
        lineSensorServoPos = max(lineSensorServoPositionMinVal, lineSensorServoPos);
        lineSensorServoPos = min(lineSensorServoPositionMaxVal, lineSensorServoPos);
    }
    lineSensorServoPosChanged = true;
    isTrackFind = true;
  }
  
  // Change line sensor servo position
  if (lineSensorServoPosChanged) {
    lineSensorServo.write(lineSensorServoPos);
  }
 
 // Change motors speed
 if (isTrackFind) {
   setMotorSpeedByServoPosition(fakeServoPosForSpiraleMove);
   // Try to increase servo pos with respective to time
   unsigned long dt = millis() - time;
   if (dt > 500) {
     time = millis();
     fakeServoPosForSpiraleMove += abs(trackFindingLineServoStep);
     if (fakeServoPosForSpiraleMove >= lineSensorServoPositionMaxVal) {
       fakeServoPosForSpiraleMove = lineSensorServoPositionMinVal;
     }
   }
 } else if (isOnTheField) {
   // Try to connect line servo position with motors speed
   // Vehicle should rotate according to the line sensor servo position. Moving like this helps us to follow the line. 
   setMotorSpeedByServoPosition(lineSensorServoPos);
 }
 else {
    analogWrite(leftMotorSpeedPinNo, 0);
    analogWrite(rightMotorSpeedPinNo, 0);
 }
  
  delay(15); // waits for the servo to get target position
}
