#include <PID_v1.h>
#include<Servo.h>

/*
       UPT RACING TEAM

       TSR2//2022//GABOR ALEXANDRU
*/


// throttles
const int POT_THROTTLE = A5; // Servo Position throttle
const int POT_PEDAL = A2; // Pedal Sensor throttle
const int POT_PEDAL2 = A3; //al doilea senzor pedala
const int SAFE = A4;
//const int BSPD = 4;
const int RELANTIU = A6;


// Pins H bridge
const int HB_FORWARD = 6; // H bridge Forward
const int HB_REVERSE = 5; // H bridge Reverse
const int HB_PWM = 3;     // H bridge PWM (speed)
const int HB_ENABLE = 7;  // H bridge Enable

// Throttle constraints
// 7.97
const int THROTTLE_MIN = 161;
const int THROTTLE_MAX = 950;
//const int THROTTLE_DIFF = 797;

//Pedal constraints
// 3.26
const int PEDAL_MIN = 74;
const int PEDAL_MAX = 390;
//const int PEDAL_DIFF = 326;

// (958 - 400) / (161 - 74)
const int PED2_MIN = 140;
const int PED2_MAX = 800;

//Motor PWM range
const int PWM_MAX = 180;
const int PWM_MIN = 0;

int throttleRest; //Normalized throttle resting position

//PID Control variables
double kP = 2;
double kI = 0;
double kD = 0;

//Define the PID controller
double throttle, Output, pedal;
PID throttlePID(&throttle, &Output, &pedal, kP, kI, kD, DIRECT);

void setup() {
  //Serial.begin(9600);
    
  //TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz

  //Determine pot value with throttle at rest
  throttleRest = map(constrain(analogRead(POT_THROTTLE), THROTTLE_MIN, THROTTLE_MAX), THROTTLE_MIN, THROTTLE_MAX, 0, 1023);
  
  //Enable PID control
  throttlePID.SetMode(AUTOMATIC);

  //Enable motor controller
  digitalWrite(HB_ENABLE, HIGH);
}


void loop() {
  if (SAFE < 1023) {
    digitalWrite(HB_FORWARD, LOW);
    digitalWrite(HB_REVERSE, LOW);
  }

  // Serial.println(RELANTIU);
  //TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  //  TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz
  int tps = 0; //analogRead(A1);
  //tps = tps / 10;
  int RELA = PEDAL_MIN - tps;

  //Open throttle if pedal position > throttle position
  if (pedal > throttle) {
    digitalWrite(HB_FORWARD, HIGH);
    digitalWrite(HB_REVERSE, LOW);
    //Close throttle
//  } else if (pedal < throttle) {
//    digitalWrite(HB_FORWARD, LOW);
//    digitalWrite(HB_REVERSE, HIGH);
    //Return throttle to resting position
  } else if (pedal == throttle){
    digitalWrite(HB_FORWARD, LOW);
    digitalWrite(HB_REVERSE, LOW);
  } else {
    digitalWrite(HB_FORWARD, LOW);
    digitalWrite(HB_REVERSE, HIGH);
  }

  // Read throttle position
  throttle = map(constrain(analogRead(POT_THROTTLE) / 10 * 10, THROTTLE_MIN, THROTTLE_MAX), THROTTLE_MIN, THROTTLE_MAX, 0, 1023);
  //throttle = (map(constrain(analogRead(POT_THROTTLE), THROTTLE_MIN, THROTTLE_MAX), THROTTLE_MIN, THROTTLE_MAX, 0, 1023) - THROTTLE_MIN) / THROTTLE_DIFF * 100;
//    Serial.print("throttle: ");
//    Serial.print(throttle);
//    Serial.println();
  //
  // if (throttle > 256) {
  //    digitalWrite(BSPD, HIGH);
  // } else {
  //    digitalWrite(BSPD, LOW);
  // }
  // Read pedal sensor position
  pedal = map(constrain(analogRead(POT_PEDAL) / 10 * 10, PEDAL_MIN, PEDAL_MAX), PEDAL_MIN, PEDAL_MAX, 0, 1023);
//    Serial.print("pedal: ");
//    Serial.print(analogRead(POT_PEDAL));
//    Serial.println();
  // pedal = map(constrain(analogRead(POT_PEDAL), RELA, PEDAL_MAX), RELA, PEDAL_MAX, 0, 1023);
  //Go PID, Go!
  throttlePID.Compute();

  // Map PID output to motor PWM range
  Output = map(Output, 0, 255, PWM_MIN, PWM_MAX);
  Serial.print("p: ");
  Serial.print(pedal);
  Serial.print(" | t: ");
  Serial.print(throttle);
  Serial.print(" | o: ");
  Serial.print(Output);
  Serial.println();
  //  analogWriteFrequency(3, 1000);
  analogWrite(HB_PWM, Output);
  //  delay(1000);
}
