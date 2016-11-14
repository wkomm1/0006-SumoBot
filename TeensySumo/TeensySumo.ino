#include <Wire.h>  //include the i2c library
#include <Servo.h>  //include the servo library

//pin assignments
byte ledPin = 6;  //led pin for status indicator
byte trigPin = 7;  //ultrasonic trigger pin
byte echoPin = 8;  //ultrasonic echo pin
byte line_FL = A5;  //front left line sensor pin
byte line_FR = A6;  //front right line sensor pin
byte line_BR = A4;  //back right line sensor pin
byte line_BL = A3;  //back left line sensor pin
byte motor_A1 = 15;  //motor A terminal 1 pin
byte motor_A2 = 14;  //motor A terminal 2 pin
byte motor_B1 = 17;  //motor B terminal 1 pin
byte motor_B2 = 16;  //motor B terminal 2 pin
byte servoPin = 27;  //servo command line pin
byte cellPinA = A0;  //lipo balance checker pin A
byte cellPinB = A2;  //lipo balance checker pin A

//constants
Servo flipServo;  //not realy a constant but its the servo object
const int diagRate = 500;  //serial diagnostic refresh rate
const byte maxRange = 50;  //maximum range to look for
const byte address  = 0x3A;  //IMU module i2c address
const byte servoCenter = 88;  //center point for the servo
const byte lineFault = 100;  //difference required to trigger line fault

//calabaration vaiables
int xCal;  //x acceleration calabration value
int yCal;  //y acceleration calabration value
int zCal;  //z acceleration calabration value
int lineCal_FL;  //front left line sensor calabration value
int lineCal_FR;  //front right line sensor calabration value
int lineCal_BR;  //back right line sensor calabration value
int lineCal_BL;  //back left line sensor calabration value

//working variables
float cellVoltA;  //last cell voltage for cell A in mv
float cellVoltB;	//last cell voltage for cell B in mv
byte servoPos = 90;  //current servo position, doesnt need to be initilized
byte lastRange = 75;  //last measured range for ultrasonic sensor
unsigned long lastDiag = 0;  //time of last serial diagnostic refresh
unsigned long pulseTime = 0;  //pulse time for the ultrasonic sensors ping
boolean servoNormal = false;  //flag to indicate the arm is pointed up
boolean rotDirection = true;  //direction of rotation on next turn true = CW
boolean objectSpotted = false;  //an object is spotted



//================PROTOTYPES===================
//void ADCS() {}  //accepts none returns none
//void printDiag() {}  //accepts none returns none
//byte getRange() {}  //accepts none returns range in cm
//void servoControl(byte angle) {}  //accepts psoition 0 to 180 returns none
//float gForce(char axis) {}  //accepts 'x' 'y' or 'z' returns -2.00000 to 2.00000
//void motorControl(char fblr, byte duration) {}  //accepts 'f' 'b' 'l' or 'r' and 0 to 255
//int rawXYZ(char axis) {}  //accepts 'x' 'y' or 'z' returns -128 to 128 returns -16384 to 16384
//void powerCheck() {}  //accepts none returns none, refreshes cellVoltA and cellVoltB values



//===========================================================================================================
void setup() {
  pinMode(ledPin, OUTPUT);  //initilize board status led
  digitalWrite(ledPin, HIGH);  //turn on board status led

  Wire.begin();  //initilize i2c bus
  Serial.begin(9600);  //initilize serial port

  //initilize IO pins
  pinMode(line_FL, INPUT);  //front left line sensor
  pinMode(line_FR, INPUT);  //front right line sensor
  pinMode(line_BR, INPUT);  //back right line sensor
  pinMode(line_BL, INPUT);  //back left line sensor
  pinMode(echoPin, INPUT);  //ultrasonic echo pin
  pinMode(cellPinA, INPUT);  //initilize the cell A voltage pin
  pinMode(cellPinB, INPUT);  //initilize the cell B voltage pin
  pinMode(trigPin, OUTPUT);  //ultrasonic trigger pin
  pinMode(motor_A1, OUTPUT);  //motor A terminal 1
  pinMode(motor_A2, OUTPUT);  //motor A terminal 2
  pinMode(motor_B1, OUTPUT);  //motor B terminal 1
  pinMode(motor_B2, OUTPUT);  //motor B terminal 2
  pinMode(servoPin, OUTPUT);  //pin attached to the servo
  flipServo.attach(servoPin);  //attaches the pin to the servo

  //initilize the IMU to use just accelaromiter
  Wire.beginTransmission(address>>1);  //send the 7 bit address
  Wire.write(0x20);  //send location of register to use --> CTRL1 
  Wire.write(0x37);  //12.5hz refresh and initilize
  Wire.endTransmission();  //end the i2c transmission

    delay(1000);  //let the bot settle

  //calabarate the IMU to the startup orientation
  xCal = rawXYZ('x');  //get gurrent value and set it to the callibration value
  yCal = rawXYZ('y');  //get gurrent value and set it to the callibration value
  zCal = rawXYZ('z');  //get gurrent value and set it to the callibration value

  //calibrate the line sensors to the current position
  lineCal_FL = analogRead(line_FL);  //get gurrent value and set it to the callibration value
  lineCal_FR = analogRead(line_FR);  //get gurrent value and set it to the callibration value
  lineCal_BR = analogRead(line_BR);  //get gurrent value and set it to the callibration value
  lineCal_BL = analogRead(line_BL);  //get gurrent value and set it to the callibration value

  //center the servo and flas to indicate ready
  servoControl(servoCenter);  //point the servo up
  digitalWrite(ledPin, LOW);  //turn off the status led
  delay(100);  //delay for a short time
  digitalWrite(ledPin, HIGH);  //turn on the status led
  delay(100);  //delay for a short time
  digitalWrite(ledPin, LOW);  //turn off the status led
  delay(100);  //delay for a short time
  digitalWrite(ledPin, HIGH);  //turn on the status led
}



//===========================================================================================================
void loop() {
  while(0) {
    Serial.println(getRange());delay(100);
  }
  printDiag();  //print system information to serial port
  ADCS();  //check the orientation and adjust it
  powerCheck();  //check the avalible power

  if(getRange() <= maxRange) {  //do if something there
    for(int i = 0; i < 10; i++) {  //loop through 10 times
      motorControl('f', 10);  //go forward for a very short duration
      if(checkLine()) {  //check the line data
        i = 10000;  //reset the loop if line data is triped
      }
    }
  }
  else {  //defaults to search mode (spin and look)
    while(!objectSpotted) {  //keep doing if nothing is there
      if(rotDirection) {
        motorControl('r', 10);
      }
      else {
        motorControl('l', 10);
      }
      checkLine();
      if(getRange() <= maxRange) {  //something there
        objectSpotted = true;
        for(int i = 0; i < 10; i++) {
          motorControl('f', 10);
          if(checkLine()) {
            i = 10000;
          }
        }
      } //
    }
    objectSpotted = false;
  }
}



//===========================================================================================================
//checks the line sensors for a fault and correct for it
boolean checkLine() {  //return if fault is detected
  printDiag();  //print system information to serial port
  int tempRange = getRange();  //get a temporary range measurement
  for(int i = 0; i < 4; i++) {
    tempRange += getRange();
  }
  tempRange /= 4;
  boolean lineFaultFlag = false;  //create a line fault flag
  int value_FL = abs(analogRead(line_FL) - lineCal_FL);  //find the value of the diference from cal
  int value_FR = abs(analogRead(line_FR) - lineCal_FR);  //find the value of the diference from cal
  int value_BR = abs(analogRead(line_BR) - lineCal_BR);  //find the value of the diference from cal
  int value_BL = abs(analogRead(line_BL) - lineCal_BL);  //find the value of the diference from cal

  if(value_FL >= lineFault && value_FR >= lineFault && tempRange >= 15) {  //only act if nothing is there
    motorControl('b', 1500);  //back up for a short duration
    motorControl('r', 100);  //turn right for a short duration
    lineFaultFlag = true;  //set the flag to idicate a line fault
    rotDirection = true;  //maintain standard direction of rotation
  }
  else if(value_FL >= lineFault && tempRange >= 15) {  //only act if nothing is there
    motorControl('b', 1500);  //back up for a short duration
    motorControl('r', 100);  //turn right for a short duration
    lineFaultFlag = true;  //set the flag to idicate a line fault
    rotDirection = true; //maintain standard direction of rotation
  }
  else if(value_FR >= lineFault && tempRange >= 15) {  //only act if nothing is there
    motorControl('b', 1500);  //back up for a short duration
    motorControl('l', 100);  //turn left for a short duration
    lineFaultFlag = true;  //set the flag to idicate a line fault
    rotDirection = false;  //reverse direction of rotation
  }
  else if(value_BL >= lineFault && value_BR >= lineFault) {  //go forward if back triped
    motorControl('f', 500);  //go forward for a short duration
    lineFaultFlag = true;  //set the flag to idicate a line fault
    rotDirection = true; //maintain standard direction of rotation
  }
  else if(value_BR >= lineFault) {  //go forward if back triped
    motorControl('f', 500);  //go forward for a short duration
    lineFaultFlag = true;  //set the flag to idicate a line fault
    rotDirection = false;  //reverse direction of rotation
  }
  else if(value_BL >= lineFault) {  //go forward if back triped
    motorControl('f', 500);  //go forward for a short duration
    lineFaultFlag = true;  //set the flag to idicate a line fault
    rotDirection = true; //maintain standard direction of rotation
  }
  return lineFaultFlag;  //return the fault condition
}



//===========================================================================================================
byte getRange() {  //measure the distance to other things within 100cm
  digitalWrite(trigPin, LOW);  //reset the trigger
  delayMicroseconds(2);  //pause for a short time
  digitalWrite(trigPin, HIGH);  //set the trigger
  delayMicroseconds(10);  //keep high for 10 microseconds
  digitalWrite(trigPin, LOW);  //reset the trigger

  pulseTime = pulseIn(echoPin, HIGH, 6000);  //count pulse time (pulse width)
  lastRange = pulseTime/57;  // speed of sound = 1cm/29us (echo = double distance)
  if(lastRange == 0) {  //responce timeout or too far
    lastRange = 100;  //set to max range
  }
  return lastRange;  //return the measured distance
}



//===========================================================================================================
//converts the raw 16 bit xyz values to G force, input is -16384 to 16384
//x axis is left and right, y is up and down, and z is front and back
float gForce(char axis) {  //sumo i am your creator...
  float force;  //variable to store the force value
  if(axis == 'x') {  //do this for the x axis
    force = (rawXYZ('x') - xCal) / 16384.0;  //calabrate reading and get ratio, normaly 0
  }
  else if(axis =='y') {  //do this for the y axis
    force = rawXYZ('y') / (0.0 - yCal);  //devide by the negitive calibrated reading, normaly -1
  }
  else if(axis == 'z') { //do this for the z axis
    force = (rawXYZ('z') - zCal) / 16384.0;  //calabrate reading and get ratio, normaly 0
  }
  return force;  //return the force to the caller
}  //sumo i am your creator...



//===========================================================================================================
//read the raw accelerometer axis data returns -16384 to 16384
//x is left to right, y is bottom to top, z is back to front
//Left, bottom and back directions are positive
int rawXYZ(char axis) {
  boolean correct = false;  //flag for only i2c if x, y or z
  byte regInfo[4];  //lowReg, lowData, highReg, highData

  if(axis == 'x') {  //x axis read
    correct = true;  //set the correct flag
    regInfo[0] = 0x28;  //low register address
    regInfo[2] = 0x29;  //high register address
  }
  else if(axis =='y') {  //x axis read
    correct = true;  //set the correct flag
    regInfo[0] = 0x2A;  //low register address
    regInfo[2] = 0x2B;  //high register address
  }
  else if(axis == 'z') {  //x axis read
    correct = true;  //set the correct flag
    regInfo[0] = 0x2C;  //low register address
    regInfo[2] = 0x2D;  //high register address
  }
  else {
    regInfo[1] = 0;  //write a 0 to lowData
    regInfo[3] = 0;  //write a 0 to highData
  }

  if(correct) {  //only do i2c if x, y, or z was entered
    for(int i = 0; i <= 2; i += 2) {  //loop to access the registers
      Wire.beginTransmission(address>>1);  //start i2c with the 7 bit address
      Wire.write(regInfo[i]);  //ask for info in register
      Wire.endTransmission();  //complete the send
      Wire.requestFrom(address>>1, 1);  //Request 1 byte from 7 bit address
      while(Wire.available() == 0);  //wait for a responce
      regInfo[i + 1] = Wire.read();  //read the low data
      Wire.endTransmission();  //end the i2c transmittion
    }
  }
  return (regInfo[3]<<8 | regInfo[1]);  //return the full 16 bit number
}



//===========================================================================================================
//isdtruct the motors to go forward, back, spin left, or spin right and for a certain length of time
void motorControl(char fblr, byte duration) {
  powerCheck();  //check the avalible power before using the motors
  ADCS();  //check orientation and right if fliped
  if(fblr == 'f') {  //go forward if the caharcter was 'f'
    digitalWrite(motor_A1, HIGH);  //set motor pin high
    digitalWrite(motor_A2, LOW);  //set motor pin low
    digitalWrite(motor_B1, HIGH);  //set motor pin high
    digitalWrite(motor_B2, LOW);  //set motor pin low
  }
  else if(fblr == 'b') {  //go backward if the caharcter was 'b'
    digitalWrite(motor_A1, LOW);  //set motor pin low
    digitalWrite(motor_A2, HIGH);  //set motor pin high
    digitalWrite(motor_B1, LOW);  //set motor pin low
    digitalWrite(motor_B2, HIGH);  //set motor pin high
  }
  else if(fblr == 'l') {  //spin CCW if the caharcter was 'l'
    digitalWrite(motor_A1, LOW);  //set motor pin low
    digitalWrite(motor_A2, HIGH);  //set motor pin high
    digitalWrite(motor_B1, HIGH);  //set motor pin high
    digitalWrite(motor_B2, LOW);  //set motor pin low
  }
  else if(fblr == 'r') {  //spin CW if the caharcter was 'r'
    digitalWrite(motor_A1, HIGH);  //set motor pin high
    digitalWrite(motor_A2, LOW);  //set motor pin low
    digitalWrite(motor_B1, LOW);  //set motor pin low
    digitalWrite(motor_B2, HIGH);  //set motor pin high
  }
  delay(duration);  //delay for a hopefuly short time
  digitalWrite(motor_A1, LOW);  //set motor pin low resets the pin
  digitalWrite(motor_A2, LOW);  //set motor pin low resets the pin
  digitalWrite(motor_B1, LOW);  //set motor pin low resets the pin
  digitalWrite(motor_B2, LOW);  //set motor pin low resets the pin
}



//===========================================================================================================
//doesnt do much anymore but this function is for constraining the flip servo input
//and to save the current position to be read in diagnostics screen
//you can just use flipServo.write() if you want
void servoControl(byte angle) {
  servoPos = constrain(angle, 0, 180);  //constrain the angle between 0 and 180
  flipServo.write(servoPos);  //write the position to the servo
}



//===========================================================================================================
//Attitude Determination Control System, checks the orientation and rights itsself if needed
void ADCS() {
  powerCheck();  //check the remaining power before updating the servo
  if(gForce('x') < -0.8) {  //do if bot flips to left
    servoControl(0);  //send arm to the left
    servoNormal = false;  //reset the servo normal flag
  }
  else if(gForce('x') > 0.8) {  //do if bot flips to right
    servoControl(180);  //send arm to the right
    servoNormal = false;  //reset the servo normal flag
  }
  else {
    if(!servoNormal) {  //stops servo spasum when its idle
      servoControl(servoCenter);  //center the servo
      servoNormal = true;  //set the servo normal flag
    }
  }
}



//===========================================================================================================
//checks the remaning power inthe battery and refreshes the variables
void powerCheck() {
  cellVoltA = analogRead(cellPinA) / 0.1023;  //0.1023 adc per mv
  cellVoltB = analogRead(cellPinB) / 0.1023;  //0.1023 adc per mv
  if(cellVoltB > cellVoltA) { //cell A is correct voltage cell B is doubble
    cellVoltB = cellVoltB - cellVoltA;  //get the diference in the full voltage to the single cell
  }
  else {//cell B is correct voltage cell B is doubble
    cellVoltA = cellVoltA - cellVoltB;  //get the diference in the full voltage to the single cell
  }
  while(cellVoltA < 3300 || cellVoltB < 3300) {  //if the battery is too low force a shutdown
    cellVoltA = analogRead(cellPinA) / 0.1023;  //0.1023 adc per mv
    cellVoltB = analogRead(cellPinB) / 0.1023;  //0.1023 adc per mv
    digitalWrite(ledPin, HIGH);  //turn on the led
    delay(100);  //delay for short period
    digitalWrite(ledPin, LOW);  //turn off the led
    delay(100);  //delay for short period
    digitalWrite(ledPin, HIGH);  //turn on the led
    delay(100);  //delay for short period
    digitalWrite(ledPin, LOW);  //turn off the led
    delay(2000);  //delay for longer period period
  }
}



//===========================================================================================================
//prints the systems status to the serial port
void printDiag() {
  if(millis() > lastDiag + diagRate) {  //stops serial spam
    Serial.print("G-Force  X:");  //print some text to the serial port
    Serial.print(gForce('x'));  //print current x acceleration
    Serial.print(" Y:");  //print some text to the serial port
    Serial.print(gForce('y'));  //print y acceleration
    Serial.print(" Z:");  //print some text to the serial port
    Serial.println(gForce('z'));  //print z acceleration

    Serial.print("reflect  FL:");  //print some text to the serial port
    Serial.print(analogRead(line_FL));  //print the front left line sensor value
    Serial.print(" FR:");  //print some text to the serial port
    Serial.print(analogRead(line_FR));  //print the front right line sensor value
    Serial.print(" BR:");  //print some text to the serial port
    Serial.print(analogRead(line_BR));  //print the back right line sensor value
    Serial.print(" BL:");  //print some text to the serial port
    Serial.println(analogRead(line_BL));  //print the back left line sensor value

    Serial.print("range    ");  //print some text to the serial port
    Serial.print(getRange());  //print the current range
    Serial.print("cm  ");  //print some text to the serial port
    Serial.print("pos:");  //print some text to the serial port
    Serial.print(servoPos);  //print the current servo position
    Serial.println("deg");  //print some text to the serial port

    powerCheck();  //refresh the voltage readings
    Serial.print("power  CellA:");  //print some text to the serial port
    Serial.print(cellVoltA / 1000, 2);  //print callA's voltage with two digits
    Serial.print("v CellB:");  //print some text to the serial port
    Serial.print(cellVoltB/1000, 2);  //print callA's voltage with two digits
    Serial.print("v Bat:");  //print some text to the serial port
    Serial.print((cellVoltA + cellVoltB) / 1000, 2);   //print bats voltage with two digits
    Serial.print("v ");  //print some text to the serial port
    Serial.print(map((cellVoltA + cellVoltB), 6400, 8400, 0, 100));  //map the voltage to precentage
    Serial.println("%");  //print some text to the serial port
    lastDiag = millis();  //reset the serial delay counter
  }
}


