#include <PID_v1.h>

/*
       UPT RACING TEAM

       TSR2//2022//GABOR ALEXANDRU
*/

// Motor PWM range; DO NOT CHANGE THESE!
const int PWM_MAX = 180;
const int PWM_MIN = 0;

// throttles
const int POT_THROTTLE = A5;  // Servo Position throttle
const int POT_PEDAL = A2;     // Pedal Sensor throttle
const int POT_PEDAL2 = A3;    //al doilea senzor pedala
const int SAFE = A4;
//const int BSPD = 4;
const int RELANTIU = A6;

// Pins H bridge
const int HB_FORWARD = 6;  // H bridge Forward
const int HB_REVERSE = 5;  // H bridge Reverse
const int HB_PWM = 3;      // H bridge PWM (speed)
const int HB_ENABLE = 7;   // H bridge Enable

// Throttle constraints
const int THROTTLE_MIN = 165;
const int THROTTLE_MAX = 1004;

// Pedal constraints
const int PEDAL_MIN = 72;
const int PEDAL_MAX = 400;

const int PED2_MIN = 140;
const int PED2_MAX = 800;

//PID Control variables
const double kP = 2;
const double kI = 0;
const double kD = 0;

// Define the PID controller
double throttle, Output, pedal;
PID throttlePID(&throttle, &Output, &pedal, kP, kI, kD, DIRECT);

void setup() {
  // Serial.begin(9600);

  // TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  TCCR0B = TCCR0B & B11111000 | B00000001;  // for PWM frequency of 62500.00 Hz

  // Enable PID control
  throttlePID.SetMode(AUTOMATIC);

  // Enable motor controller
  digitalWrite(HB_ENABLE, HIGH);
}


void loop() {
  if (SAFE < 1023) {
    digitalWrite(HB_FORWARD, LOW);
    digitalWrite(HB_REVERSE, LOW);
  }

  int tps = 0;  //analogRead(A1);
  //tps = tps / 10;
  int RELANTIU = PEDAL_MIN - tps;

  // Read throttle position
  throttle = map(analogRead(POT_THROTTLE), THROTTLE_MIN, THROTTLE_MAX, 0, 1023);
  //
  // if (throttle > 256) {
  //    digitalWrite(BSPD, HIGH);
  // } else {
  //    digitalWrite(BSPD, LOW);
  // }
  // Read pedal sensor position
  pedal = map(analogRead(POT_PEDAL), RELANTIU, PEDAL_MAX, 0, 1023);
  throttlePID.Compute();

  // Map PID output to motor PWM power
  Output = map(Output, 0, 255, PWM_MIN, PWM_MAX);

  // Open throttle if pedal position > throttle position
  if (pedal > throttle) {
    digitalWrite(HB_FORWARD, HIGH);
    digitalWrite(HB_REVERSE, LOW);
  } else if (pedal == throttle) {
    // Return throttle to resting position
    digitalWrite(HB_FORWARD, LOW);
    digitalWrite(HB_REVERSE, LOW);
  } else {
    // This is a hack so that the throttle goes reverse full power
    Output = map(throttle, 0, 1023, PWM_MIN, PWM_MAX);
    digitalWrite(HB_FORWARD, LOW);
    digitalWrite(HB_REVERSE, HIGH);
  }

  Serial.print("p: ");
  Serial.print(pedal);
  Serial.print(" | t: ");
  Serial.print(throttle);
  Serial.print(" | o: ");
  Serial.print(Output);
  Serial.println();
  analogWrite(HB_PWM, Output);
  // delay(2000);
}
