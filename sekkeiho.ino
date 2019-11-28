#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//  called this way,  it uses the default address 0x40
Adafruit_PWMServoDriver pwm =  Adafruit_PWMServoDriver();
/* #define SERVOMIN  150 //  This is the 'minimum' pulse length count (out of 4096) */
/* #define SERVOMAX  600 //  This is the 'maximum' pulse length count (out of 4096) */
/* #define USMIN  600 //  This is the rounded 'minimum' microsecond length based on the minimum pulse of 150 */
/* #define USMAX  2400 //  This is the rounded 'maximum' microsecond length based on the maximum pulse of 600 */
#define SERVO_FREQ 50 //  Analog servos run at ~50 Hz updates

// pin
const int rmpin = 3;
const int lmpin = 0;
const int fingerpin = 9;
const int rpsdpin = A0;
const int lpsdpin = A1;

// motor speed
const int rightMaxSpeed = 267;
const int rightMinSpeed = 348;
const int leftMaxSpeed = 348;
const int leftMinSpeed = 267;
const int stopspeed = 312;

// gain
const double distgain = 1.0;
const double radgain = 1.0;
const double lpf = 0.9;


void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  //  The int.osc. is closer to 27MHz
  pwm.setPWMFreq(SERVO_FREQ);  //  Analog servos run at ~60 Hz updates

  pinMode(rpsdpin, INPUT);
  pinMode(lpsdpin, INPUT);

  delay(10);
}

//  You can use this function if you'd like to set the pulse length in seconds
//  e.g. setServoPulse(0,  0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n,  double pulse) {
  double pulselength;

  pulselength = 1000000;   //  1, 000, 000 us per second
  pulselength /= SERVO_FREQ;   //  Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  //  12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  //  convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
}


void forward(int speed, double dist){
  forward(speed);
  delay(dist*distgain);
}
void forward(int speed){
  pwm.setPWM(rmpin, 0, constrain(map(speed, 0, 255, stopspeed, rightMaxSpeed), 0, 255));
  pwm.setPWM(lmpin, 0, constrain(map(speed, 0, 255, stopspeed, leftMaxSpeed),  0, 255));
}

void backward(int speed, double dist){
  backward(speed);
  delay(dist*distgain);
}
void backward(int speed){
  pwm.setPWM(rmpin, 0, constrain(map(speed, 0, 255, stopspeed, rightMinSpeed), 0, 255));
  pwm.setPWM(lmpin, 0, constrain(map(speed, 0, 255, stopspeed, leftMinSpeed),  0, 255));
}

void rightRotation(int speed, double rad){
  rightRotation(speed);
  delay(rad*radgain);
}
void rightRotation(int speed){
  pwm.setPWM(rmpin, 0, constrain(map(speed, 0, 255, stopspeed, rightMinSpeed), 0, 255));
  pwm.setPWM(lmpin, 0, constrain(map(speed, 0, 255, stopspeed, leftMaxSpeed),  0, 255));
}

void leftRotation(int speed, double rad){
  leftRotation(speed);
  delay(rad*radgain);
}
void leftRotation(int speed){
  pwm.setPWM(rmpin, 0, constrain(map(speed, 0, 255, stopspeed, rightMaxSpeed), 0, 255));
  pwm.setPWM(lmpin, 0, constrain(map(speed, 0, 255, stopspeed, leftMinSpeed),  0, 255));
}

void brake(int time){
  brake();
  delay(time);
}
void brake(){
  pwm.setPWM(rmpin, 0, stopspeed);
  pwm.setPWM(lmpin, 0, stopspeed);
}
