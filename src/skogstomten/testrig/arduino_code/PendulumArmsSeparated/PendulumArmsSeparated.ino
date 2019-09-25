//      This code is used to control the pendulum arms of the XT-28 test rig.

//      2019-09-05
//      Axel Sundkvist

//      How to? Write a desired motor choice (1-6) and its desired position (0, 100)

//      NOTE: pwm is not used, instead feed the motor drivers with either 5V or 3.5V 
//      from the Arduino

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 1  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<3>                  |          CTRL<4>,  INA                   |
//      |    Digital<2>                  |          CTRL<5>,  INB                   |
//      |    Digital<X>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 2  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<5>                  |          CTRL<4>,  INA                   |
//      |    Digital<4>                  |          CTRL<5>,  INB                   |
//      |    Digital<X>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 2  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<7>                  |          CTRL<4>,  INA                   |
//      |    Digital<6>                  |          CTRL<5>,  INB                   |
//      |    Digital<X>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 2  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<9>                  |          CTRL<4>,  INA                   |
//      |    Digital<8>                  |          CTRL<5>,  INB                   |
//      |    Digital<X>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 2  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<11>                 |          CTRL<4>,  INA                   |
//      |    Digital<10>                 |          CTRL<5>,  INB                   |
//      |    Digital<X>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 2  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<13>                 |          CTRL<4>,  INA                   |
//      |    Digital<12>                 |          CTRL<5>,  INB                   |
//      |    Digital<X>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

// ----------------- Arduino Pins ----------------------------------------
const int M1Apin = 3;    // Motor control output for M1 forward
const int M1Bpin = 2;    // Motor control output for M1 backward

const int M2Apin = 5;    // Motor control output for M2 forward
const int M2Bpin = 4;    // Motor control output for M2 backward

const int M3Apin = 7;    // Motor control output for M3 forward
const int M3Bpin = 6;    // Motor control output for M3 backward

const int M4Apin = 9;    // Motor control output for M4 forward
const int M4Bpin = 8;    // Motor control output for M4 backward

const int M5Apin = 11;   // Motor control output for M5 forward
const int M5Bpin = 10;   // Motor control output for M5 backward

const int M6Apin = 13;   // Motor control output for M6 forward
const int M6Bpin = 12;   // Motor control output for M6 backward

const int pot1Pin = A0;   // Potentiometer input for M1
const int pot2Pin = A1;   // Potentiometer input for M2
const int pot3Pin = A2;   // Potentiometer input for M3
const int pot4Pin = A3;   // Potentiometer input for M4
const int pot5Pin = A4;   // Potentiometer input for M5
const int pot6Pin = A5;   // Potentiometer input for M6

//-------------------------------------------------------------------------

//------------------ Variables to change -----------------------------------
float potRef;      // Potentiometer reference value

//--------------------------------------------------------------------------

//------------------ Variables ---------------------------------------------
float pot1Val;            // Up to date input value from potentiometer in 1 (M1)
float pot2Val;            // Up to date input value from potentiometer in 2 (M2)
float pot3Val;            // Up to date input value from potentiometer in 3 (M3)
float pot4Val;            // Up to date input value from potentiometer in 4 (M4)
float pot5Val;            // Up to date input value from potentiometer in 5 (M5)
float pot6Val;            // Up to date input value from potentiometer in 6 (M6)
int resol = 2;            // Resolution of when position is accepted ("potRef +- resol" is accepted as "goal reached")
float input;
float x;
float motor_choice = 0;  

void setup() {
  // configure pins
  pinMode(M1Apin, OUTPUT);
  pinMode(M1Bpin, OUTPUT);
  pinMode(M2Apin, OUTPUT);
  pinMode(M2Bpin, OUTPUT);
  pinMode(M3Apin, OUTPUT);
  pinMode(M3Bpin, OUTPUT);
  pinMode(M4Apin, OUTPUT);
  pinMode(M4Bpin, OUTPUT);
  pinMode(M5Apin, OUTPUT);
  pinMode(M5Bpin, OUTPUT);
  pinMode(M6Apin, OUTPUT);
  pinMode(M6Bpin, OUTPUT);
  
  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
  pinMode(pot3Pin, INPUT);
  pinMode(pot4Pin, INPUT);
  pinMode(pot5Pin, INPUT);
  pinMode(pot6Pin, INPUT);
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
    //------------------------------ Read desired value --------------------------------------
    if (Serial.available() > 0) {                                // if serial message is availabe
      motor_choice = Serial.parseFloat();                          // store as float
      x = Serial.parseFloat();                                     // store as float
      Serial.read();                                               // to clear buffer
      if (x < 0) {
        x = 0;
      }
      if (x > 100) {
        x = 100;
      }
      input = float(x*1024/100);
      Serial.print(input);
      if (input >= 0 && input <= 1024)                             // if value between min and max (1024 since 10-bit)
      {                                  
        potRef = input;                                         // set desired value as reference for M1
    }
    }
    //----------------------------------------------------------------------------------------

    //------------------------------ Arm 1 (M1) --------------------------------------------
    pot1Val = analogRead(pot1Pin);                               // Read potentiometer value for M1
    if (motor_choice == 1) { // here you can put if statement to control the motors separately
      if (pot1Val < potRef - resol) {
        // if reference value smaller than actual - go "backward"
        digitalWrite(M1Bpin, HIGH);
        digitalWrite(M1Apin, LOW);
      }
      else if (pot1Val > potRef + resol) {
        // if reference value larger than actual - go "forward"
        digitalWrite(M1Bpin, LOW);
        digitalWrite(M1Apin, HIGH);
      }
      else {
        // goal reached!
        digitalWrite(M1Bpin, LOW);
        digitalWrite(M1Apin, LOW);
        motor_choice = 0;                // to stop all action
      }
    }
    //---------------------------------------------------------------------------------------

    //------------------------------ Arm 2 (M2) --------------------------------------------
    pot2Val = analogRead(pot2Pin);                               // Read potentiometer value for M1
    if (motor_choice == 2) { // here you can put if statement to control the motors separately
      if (pot2Val < potRef - resol) {
        // if reference value smaller than actual - go "backward"
        digitalWrite(M2Bpin, HIGH);
        digitalWrite(M2Apin, LOW);
      }
      else if (pot2Val > potRef + resol) {
        // if reference value larger than actual - go "forward"
        digitalWrite(M2Bpin, LOW);
        digitalWrite(M2Apin, HIGH);
      }
      else {
        // goal reached!
        digitalWrite(M2Bpin, LOW);
        digitalWrite(M2Apin, LOW);
        motor_choice = 0;                // to stop all action
      }
    }
    //---------------------------------------------------------------------------------------

    //------------------------------ Arm 3 (M3) --------------------------------------------
    pot3Val = analogRead(pot3Pin);                               // Read potentiometer value for M1
    if (motor_choice == 3) { // here you can put if statement to control the motors separately
      if (pot3Val < potRef - resol) {
        // if reference value smaller than actual - go "backward"
        digitalWrite(M3Bpin, HIGH);
        digitalWrite(M3Apin, LOW);
      }
      else if (pot3Val > potRef + resol) {
        // if reference value larger than actual - go "forward"
        digitalWrite(M3Bpin, LOW);
        digitalWrite(M3Apin, HIGH);
      }
      else {
        // goal reached!
        digitalWrite(M3Bpin, LOW);
        digitalWrite(M3Apin, LOW);
        motor_choice = 0;                // to stop all action
      }
    }
    //---------------------------------------------------------------------------------------

    //------------------------------ Arm 4 (M4) --------------------------------------------
    pot4Val = analogRead(pot4Pin);                               // Read potentiometer value for M1
    if (motor_choice == 4) { // here you can put if statement to control the motors separately
      if (pot4Val < potRef - resol) {
        // if reference value smaller than actual - go "backward"
        digitalWrite(M4Bpin, HIGH);
        digitalWrite(M4Apin, LOW);
      }
      else if (pot4Val > potRef + resol) {
        // if reference value larger than actual - go "forward"
        digitalWrite(M4Bpin, LOW);
        digitalWrite(M4Apin, HIGH);
      }
      else {
        // goal reached!
        digitalWrite(M4Bpin, LOW);
        digitalWrite(M4Apin, LOW);
        motor_choice = 0;                // to stop all action
      }
    }
    //---------------------------------------------------------------------------------------

    //------------------------------ Arm 5 (M5) --------------------------------------------
    pot5Val = analogRead(pot5Pin);                               // Read potentiometer value for M1
    if (motor_choice == 5) { // here you can put if statement to control the motors separately
      if (pot5Val < potRef - resol) {
        // if reference value smaller than actual - go "backward"
        digitalWrite(M5Bpin, HIGH);
        digitalWrite(M5Apin, LOW);
      }
      else if (pot5Val > potRef + resol) {
        // if reference value larger than actual - go "forward"
        digitalWrite(M5Bpin, LOW);
        digitalWrite(M5Apin, HIGH);
      }
      else {
        // goal reached!
        digitalWrite(M5Bpin, LOW);
        digitalWrite(M5Apin, LOW);
        motor_choice = 0;                // to stop all action
      }
    }
    //---------------------------------------------------------------------------------------

    //------------------------------ Arm 6 (M6) --------------------------------------------
    pot6Val = analogRead(pot6Pin);                               // Read potentiometer value for M1
    if (motor_choice == 6) { // here you can put if statement to control the motors separately
      if (pot6Val < potRef - resol) {
        // if reference value smaller than actual - go "backward"
        digitalWrite(M6Bpin, HIGH);
        digitalWrite(M6Apin, LOW);
      }
      else if (pot6Val > potRef + resol) {
        // if reference value larger than actual - go "forward"
        digitalWrite(M6Bpin, LOW);
        digitalWrite(M6Apin, HIGH);
      }
      else {
        // goal reached!
        digitalWrite(M6Bpin, LOW);
        digitalWrite(M6Apin, LOW);
        motor_choice = 0;                // to stop all action
      }
    }
    //---------------------------------------------------------------------------------------
  
}
