//      This code is used to control the steering of the XT-28 test rig. A value sent to
//      the Arduino will be interpreted as a desired steering angle for waist 1 which this 
//      code will aim to steer the XT-28 test rig to.

//      2019-09-05
//      Axel Sundkvist

//      How to? Write a desired angle (-48.5, 40.5) that you want the forward-most waist 
//      to turn to.

//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Arduino          |      Connector at BB-VNH3SP30      no 1  |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power, 5V                   |          CTRL<2>,  +5V                   |
//      |    Power, GND                  |          CTRL<3>,  GND                   |
//      |    Digital<13>                 |          CTRL<4>,  INA                   |
//      |    Digital<12>                 |          CTRL<5>,  INB                   |
//      |    Digital<11>                 |          CTRL<6>,  PWM                   |
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
//      |    Digital<5>                  |          CTRL<6>,  PWM                   |
//      |--------------------------------|------------------------------------------|

//------------------ FOR PENDULUM ARMS PWM ------------------------------
// Does NOT affect this code!
const int pwm3Pin = 3;    // pwm output for pendulum arms
float pwm3 = 63.75;          // pwm for penduum arms

// ----------------- Arduino Pins ----------------------------------------
const int M1Apin = 13;    // Motor control output for M1 forward
const int M1Bpin = 12;    // Motor control output for M1 backward
const int M2Apin = 7;     // Motor control output for M2 forward
const int M2Bpin = 6;    // Motor control output for M2 backward
const int pwm1Pin = 11;   // pwm output for M1
const int pwm2Pin = 5;    // pwm output for M2

const int pot1Pin = A0;   // Potentiometer input for M1
const int pot2Pin = A1;   // Potentiometer input for M2

//-----------------------------------------100--------------------------------

//------------------ Variables to change -----------------------------------
float factor = 0.9;         // Scaling factor (speed and distance) between waist 1 and 2 (M1 and M2)
float pwm1 = 200;         // PWM duty cycle for waist 1 (M1)

//--------------------------------------------------------------------------

//------------------ Variables ---------------------------------------------
float pwm2 = pwm1;//*factor; // PWM duty cycle for waist 2 (M2), calculated as "pwm1*factor"
float lastpot1Ref;        // To save last iteration's reference value for M1
float lastpot2Ref;        // To save last iteration's reference value for M2
float pot1Val;            // Up to date input value from potentiometer in waist 1 (M1)
float pot2Val;            // Up to date input value from potentiometer in waist 2 (M2)
float input1;              // calculated value between 0 and 1024
float input2;
float x;                  // desired turning between min_angle1 and max_angle1
float y;                  // desired turning between min_angle2 and max_angle2
float pot1Ref;      // Potentiometer reference value for M1 (to be chosen), 500 is in the middle
float pot2Ref;      // Potentiometer reference value for M2 (to be chosen), 500 is in the middle
float actual_angle1; // The current actual angle of front waist
float actual_angle2; // The current actual angle of back waist

bool reached_goal_1 = false;  // goal reached for waist 1
bool reached_goal_2 = false;  // goal reached for waist 2
float RG_resol = 6;             // reached_goal resolution, to allow for some fluctuation from read value
float resol = 2;            // Resolution of when position is accepted ("potRef +- resol" is accepted as "goal reached")


// Calibration of Waist 1
// calibration values
float middle1 = 530;         // potentiometer value in middle
float lower1 = 50;           // potentiometer value in minimum
float higher1 = 940;         // potentiometer value in maximum
float max_angle1 = 40.5;      //=   //=43.5;      // Actual positive angle at potentiometer maximum (0 in middle, negative to left, positive to right)
float min_angle1 = -48.5;     // Actual negative (with sign) angle at potentiometer minimum (0 in middle, negative to left, positive to right)
float k1;
float m1;
//---------------------

// Calibration of Waist 2
// calibration values
float middle2 = 592;         // potentiometer value in middle
float lower2 = 50;           // potentiometer value in minimum
float higher2 = 940;         // potentiometer value in maximum
float max_angle2 = 40.5;      //=   //=43.5;      // Actual positive angle at potentiometer maximum (0 in middle, negative to left, positive to right)
float min_angle2 = -48.5;     // Actual negative (with sign) angle at potentiometer minimum (0 in middle, negative to left, positive to right)
float k2;
float m2;
//---------------------



void setup() {
  // configure pins
  pinMode(M1Apin, OUTPUT);
  pinMode(M1Bpin, OUTPUT);
  pinMode(M2Apin, OUTPUT);
  pinMode(M2Bpin, OUTPUT);
  pinMode(pwm1Pin, OUTPUT);
  pinMode(pwm2Pin, OUTPUT);
  pinMode(pwm3Pin, OUTPUT);
  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps

  // set PWM duty cycle
  analogWrite(pwm1Pin, pwm1);
  analogWrite(pwm2Pin, pwm2);
  analogWrite(pwm3Pin, pwm3);

  // Calibration of Waist 1
  // f(x) = kx + m
  m1 = middle1;
  if (abs(middle1-lower1) > abs(middle1-higher1)) {
    // distance to lower bound larger
    k1 = -(higher1-m1)/(max_angle1);
  } else {
    // distance to higher bound larger
    k1 = -(lower1-m1)/(-min_angle1);
  }

  // Calibration of Waist 2
  // f(x) = kx + m
  m2 = middle2;
  if (abs(middle2-lower2) > abs(middle2-higher2)) {
    // distance to lower bound larger
    k2 = -(higher2-m2)/(max_angle2);
  } else {
    // distance to higher bound larger
    k2 = -(lower2-m2)/(-min_angle2);
  }
  // Initiate position
  pot1Ref = middle1;
  pot2Ref = middle2;
}

void loop() {
    //------------------------------ Read desired value --------------------------------------
    if (Serial.available() > 0) {                                // if serial message is availabe
      x = Serial.parseFloat();                                   // store as float
      y = Serial.parseFloat();
      Serial.read();                                            // To ensure inpt buffer is empty. Without this parseInt() will return the number + 0.000!!!
      input1 = ref_calc(x, k1, m1, min_angle1, max_angle1);
      input2 = ref_calc(y, k2, m2, min_angle2, max_angle2);
      lastpot2Ref = pot2Ref;                                   // store last value for M1
      lastpot1Ref = pot1Ref;                                   // store last value for M2
      pot1Ref = input1;                                         // set desired value as reference for M1
      pot2Ref = input2;//lastpot2Ref+(pot1Ref-lastpot1Ref)*factor;      // calculate reference value for M2 and set it
    }
    //----------------------------------------------------------------------------------------

    //------------------------------ Waist 1 (M1) --------------------------------------------
    pot1Val = analogRead(pot1Pin);                               // Read potentiometer value for M1
    actual_angle1 = ref_calc_inv(pot1Val, k1, m1);             // Calculate the current steering angle
    
    
    if (pot1Val < pot1Ref - RG_resol || pot1Val > pot1Ref + RG_resol) {
      reached_goal_1 = false;
    }
    
    if (pot1Val < pot1Ref - resol && !reached_goal_1) {
      // if reference value smaller than actual - go "backward"
      digitalWrite(M1Bpin, HIGH);
      digitalWrite(M1Apin, LOW);
    }
    else if (pot1Val > pot1Ref + resol && !reached_goal_1) {
      // if reference value larger than actual - go "forward"
      digitalWrite(M1Bpin, LOW);
      digitalWrite(M1Apin, HIGH);
    }
    else {
      // goal reached!
      reached_goal_1 = true;
      digitalWrite(M1Bpin, LOW);
      digitalWrite(M1Apin, LOW);
    }
    //---------------------------------------------------------------------------------------

    //------------------------------ Waist 2 (M2) --------------------------------------------
    pot2Val = analogRead(pot2Pin);                               // Read potentiometer value for M1
    actual_angle2 = ref_calc_inv(pot2Val, k2, m2);

    Serial.print(actual_angle1);
    Serial.print(" ");
    Serial.println(actual_angle2);
    
    if (pot2Val < pot2Ref - RG_resol || pot2Val > pot2Ref + RG_resol) {
      reached_goal_2 = false;
    }
    
    if (pot2Val < pot2Ref - resol && !reached_goal_2) {
      // if reference value smaller than actual - go "backward"
      digitalWrite(M2Bpin, HIGH);
      digitalWrite(M2Apin, LOW);
    }
    else if (pot2Val > pot2Ref + resol && !reached_goal_2) {
      // if reference value larger than actual - go "forward"
      digitalWrite(M2Bpin, LOW);
      digitalWrite(M2Apin, HIGH);
    }
    else {
      // goal reached!
      reached_goal_2 = true;
      digitalWrite(M2Bpin, LOW);
      digitalWrite(M2Apin, LOW);
    }
    //---------------------------------------------------------------------------------------
}

float ref_calc(float x, float k, float m, float min_angle, float max_angle){
  // Function that calculates the potentiometer reference value from a desired steering angle
  float diff = min(abs(max_angle), abs(min_angle));
  if (abs(x) > diff) {
    if (x < 0) {
      x = -diff;
    } else {
      x = diff;
    }
  }
  float result = k*x + m;
  return result;
}

float ref_calc_inv(float potVal, float k, float m){
  // Function that calculates the angle corresponding to a potentiometer value
  // inverse of 'ref_calc()'
  float result = (potVal-m)/k;
  return result;
}
