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

//------------------ INCLUDES -------------------------------------------
#include <stdio.h>
#include <ros.h>
#include <std_msgs/Int64.h>

// ----------------- Functions ------------------------------------------
void pendulum_callback(const std_msgs::Int64& pendulum_msg);

//------------------ ROS ------------------------------------------------
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int64> move_pendulum("pendulum_action", pendulum_callback);

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
int middle = 512;      // Potentiometer reference value in middle (512 is mean)

//--------------------------------------------------------------------------

//------------------ Variables ---------------------------------------------
int resol = 2;            // Resolution of when position is accepted ("potRef +- resol" is accepted as "goal reached")
static int motor_choice = 0;
float potRef;

int A_pins[] = {M1Apin, M2Apin, M3Apin, M4Apin, M5Apin, M6Apin};
int B_pins[] = {M1Bpin, M2Bpin, M3Bpin, M4Bpin, M5Bpin, M6Bpin};
float potVal;   // Up to date input value from potentiometer in 1 (M1)
float potPins[] = {pot1Pin, pot2Pin, pot3Pin, pot4Pin, pot5Pin, pot6Pin};

void setup() {
  // ROS
  nh.initNode();
  nh.subscribe(move_pendulum);
  
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
}

void loop() {
    // ----------------------------- Read from bus -------------------------------------
    nh.spinOnce(); 
}


void pendulum_callback(const std_msgs::Int64& pendulum_msg) {
  // choose motor [1-6] then press:
  //    - 7 to get to middle
  //    - 8 to lift the pendulum arm
  //    - 9 to lower the pendulum arm
  int action = pendulum_msg.data;
  if (action <= 6 && action >= 1) {     // choose motor
    motor_choice = action;
  }
  else if (action == 8) {
    //  - go "backward"
    digitalWrite(B_pins[motor_choice-1], HIGH);
    digitalWrite(A_pins[motor_choice-1], LOW);
  }
  else if (action == 9) {
    //  - go "forward"
    digitalWrite(B_pins[motor_choice-1], LOW);
    digitalWrite(A_pins[motor_choice-1], HIGH);
  }
  else if (action == 7) {
    // go to middle
    potVal = analogRead(potPins[motor_choice-1]);
      if (potVal > middle + resol) {
        // if reference value smaller than actual - go "backward"
        digitalWrite(B_pins[motor_choice-1], HIGH);
        digitalWrite(A_pins[motor_choice-1], LOW);
      }
      else if (potVal < middle - resol) {
        // if reference value larger than actual - go "forward"
        digitalWrite(B_pins[motor_choice-1], LOW);
        digitalWrite(A_pins[motor_choice-1], HIGH);
      }
      else {
        // goal reached!
        digitalWrite(B_pins[motor_choice-1], LOW);
        digitalWrite(A_pins[motor_choice-1], LOW);
        motor_choice = 0;       // to stop all action
      }
    }
    else {
        digitalWrite(B_pins[motor_choice-1], LOW);
        digitalWrite(A_pins[motor_choice-1], LOW);
    }
}
