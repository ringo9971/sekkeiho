#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <QTRSensors.h>   //QTRセンサを使用するのに必要


//  called this way,  it uses the default address 0x40
Adafruit_PWMServoDriver pwm =  Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 //  Analog servos run at ~50 Hz updates
#define NUM_SENSORS 8


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
const int rightstopspeed = 312;
const int leftstopspeed = 311;

const int top = 102;
const int bottom = 491;

// gain
const double distgain = 110;
const double radgain = 30;
const double lpf = 0.9;


QTRSensors qtr;                      // センサを使用するためのオブジェクトの生成
const uint8_t SensorCount = 8;       // 使用するセンサの数
const int middlevalue = 500;
uint16_t sensorValues[SensorCount];  // 各センサの値を格納するための変数

//制御ゲイン
double Kp = 0.28;
double Kd = 0.15;

//制御偏差（目標値と現在の値との差）
double err = 0;

//操作量（偏差を0にするためのモーターへの入力値）
double u_in = 0;

//目標値（ラインセンサの中心の値）
double ref = 7000 *0.5;

//標準的な回転速度（PWMの指令値で指定）
double omgR0 = rightMaxSpeed;
double omgL0 = leftMaxSpeed;

double position;
double pastposition = ref;


void setup() {
  Serial.begin(115200);
  Serial.println("Sample Program : LineTracer v2");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  //  The int.osc. is closer to 27MHz
  pwm.setPWMFreq(SERVO_FREQ);  //  Analog servos run at ~60 Hz updates

  pinMode(rpsdpin, INPUT);
  pinMode(lpsdpin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Calibrating line sensor ...");
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 6, 7, 8, 9}, SensorCount);


  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 150; i++){
    qtr.calibrate();
  }
  // キャリブレーション中を知らせるためのLEDを消灯
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibrated line sensor.");
  delay(800);
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

int state = 0;
int32_t timer;
void loop() {
  /* while(1){ */
  /*   int diff = analogRead(0)-analogRead(1); */
  /*   diff *= 0.4; */
  /*   if(diff > 0) rightRotation(diff); */
  /*   else leftRotation(diff); */
  /* } */
  /* while(1){ */
  /*   leftRotation(100); */
  /*   if(analogRead(0) >= 300 && analogRead(1) <= 200){ */
  /*     break; */
  /*   } */
  /* } */
  /* leftRotation(100, 10); */
  /* brake(); */
  /* delay(500); */
  /* while(1){ */
  /*   forward(100); */
  /*   /1* if(max(analogRead(0), analogRead(1) >= 600)) break; *1/ */
  /* } */
  /* brake(); */
  /* while(1); */


  switch(state){
    case 0:
      linetrace();
      if(sensorValues[0] > middlevalue){
        forward(255, 15);
        leftturn(255);
        state++;
      }
      break;
    case 1:
      linetrace();
      if(sensorValues[NUM_SENSORS-1] > middlevalue) state++;
      break;
    case 2:
      forward(255, 15); // フタツメカーブ
      rightturn(255);
      state++;
      timer = millis();
      break;
    case 3:
      if(millis()-timer >= 2000) state++;
      linetrace();
      break;
    case 4:
      leftturn(255);
      state++;
      break;
    case 5:
      linetrace();
      if(sensorValues[0] > middlevalue){
        forward(255, 15);
        rightturn(255);
        state++;
      }
      break;
    case 6: 
      linetrace();
      if(sensorValues[0] > middlevalue){
        rightturn(255);
        backward(255, 5);
        brake(2000); // イッカイメノシュート  
        state = 100;
      }
      break;
    case 100:
      linetrace();
      if(sensorValues[0] > middlevalue){
        forward(255, 15);
        leftturn(255);
        state++;
      }
      timer = millis();
      break;
    case 101:
      if(millis()-timer >= 2000) state++;
      linetrace();
      break;
    case 102:
      rightturn(255);
      state++;
      break;
    case 103:
      linetrace();
      if(sensorValues[0] > middlevalue){
        forward(255, 15);
        rightturn(255);
        state++;
      }
      break;
    case 104:
      linetrace();
      if(sensorValues[0] > middlevalue){
        rightturn(255);
        backward(255, 5);
        brake(2000); // 二回目のシュート
        state = 200;
      }
      break;

    case 200:
      linetrace();
      if(sensorValues[NUM_SENSORS-1] > middlevalue){
        forward(255, 15);
        rightturn(255);
        rightturn(255);
        forward(255, 20);
        state++;
      }
      break;

    default:
      brake();
  }
}

void rightturn(int speed){
  rightRotation(speed);
  while(onLine()) delay(10);
  delay(10);
  while(!onLine()) delay(10);
  delay(100);
  while(sensorValues[3] > middlevalue) delay(10);
}

void leftturn(int speed){
  leftRotation(speed);
  while(onLine()) delay(10);
  delay(10);
  while(!onLine()) delay(10);
  delay(100);
  while(sensorValues[4] > middlevalue) delay(10);
}

void showpsd(){
  Serial.print(analogRead(0));
  Serial.print("\t");
  Serial.print(analogRead(1));
  Serial.print("\n");
}
void debug(){
  for (unsigned char i = 0; i < NUM_SENSORS; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print('|');
  Serial.print('\t');
  Serial.print(position);
  Serial.print('\n');
}

bool onLine(){
  position = qtr.readLineBlack(sensorValues);
  for(int i = 0; i < NUM_SENSORS; i++){
    if(sensorValues[i] > middlevalue) return true;
  }
  return false;
}

void linetrace(){
  position = qtr.readLineBlack(sensorValues);
  err = ref-position;
  u_in = Kp*err+Kd*(position-pastposition);
  u_in = Kp*err+Kd*(position-pastposition);
  double omgR = omgR0 + u_in;
  double omgL = omgL0 - u_in;

  omgR = constrain((int)omgR, 0, 255);
  omgL = constrain((int)omgL, 0, 255);

  diagonallyforward(omgR, omgL);

  pastposition = position;
}

void diagonallyforward(int rightspeed, int leftspeed){
  pwm.setPWM(rmpin, 0, map(rightspeed, 0, 255, rightstopspeed, rightMaxSpeed));
  pwm.setPWM(lmpin, 0, map(leftspeed, 0, 255, leftstopspeed, leftMaxSpeed));
}

void forward(int speed, double dist){
  forward(speed);
  delay(dist*distgain);
}
void forward(int speed){
  pwm.setPWM(rmpin, 0, map(speed, 0, 255, rightstopspeed, rightMaxSpeed));
  pwm.setPWM(lmpin, 0, map(speed, 0, 255, leftstopspeed, leftMaxSpeed));
}

void backward(int speed, double dist){
  backward(speed);
  delay(dist*distgain);
}
void backward(int speed){
  pwm.setPWM(rmpin, 0, map(speed, 0, 255, rightstopspeed, rightMinSpeed));
  pwm.setPWM(lmpin, 0, map(speed, 0, 255, leftstopspeed, leftMinSpeed));
}

void rightRotation(int speed, double rad){
  rightRotation(speed);
  delay(rad*radgain);
}
void rightRotation(int speed){
  pwm.setPWM(rmpin, 0, map(speed, 0, 255, rightstopspeed, rightMinSpeed));
  pwm.setPWM(lmpin, 0, map(speed, 0, 255, leftstopspeed, leftMaxSpeed));
}

void leftRotation(int speed, double rad){
  leftRotation(speed);
  delay(rad*radgain);
}
void leftRotation(int speed){
  pwm.setPWM(rmpin, 0, map(speed, 0, 255, rightstopspeed, rightMaxSpeed));
  pwm.setPWM(lmpin, 0, map(speed, 0, 255, leftstopspeed, leftMinSpeed));
}

void brake(int time){
  brake();
  delay(time);
}
void brake(){
  pwm.setPWM(rmpin, 0, rightstopspeed);
  pwm.setPWM(lmpin, 0, leftstopspeed);
}
