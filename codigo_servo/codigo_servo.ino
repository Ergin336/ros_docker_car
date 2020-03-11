#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
std_msgs::Float64 Distancia;
ros::Publisher chatter("chatter",&Distancia);

#define FRONT (8)
#define LEFT (4)
#define BACK (2)
#define RIGHT (6)
#define STAY (0)


#define ANGLE_LEFT_MAX (180) 

#define ANGLE LEFT VISION (130)
#define ANGLE_MIDDLE ( 90) 
#define ANGLE_RIGHT_VISION ( 50)
#define ANGLE_RIGHT_MAX(0) 
#define ANGLE_STEP (10) 
#define ANGLE_WINDOW_RIGHT ( 75) 
#define ANGLE_WINDOW_LEFT (105) 
#define SERVO_SPEED (2) 


#define DELAY_FACTOR_F (8) 
#define DELAY_FACTOR_B (1)
#define DELAY_BACK_RND (200 / DELAY_FACTOR_B)
#define DELAY_FRONT (40 / DELAY_FACTOR_F)
#define DELAY_TURN (600 / DELAY_FACTOR_F)
#define DELAY_STOP (100)
#define DELAY_BACK_NEAR (200 / DELAY_FACTOR_B)
#define DELAY_BACK_BACK (500 / DELAY_FACTOR_B)
#define DELAY_BACK_TURN (100 / DELAY_FACTOR_B)
#define WINDOW_MINIMUM (40) 
#define DIST_MINIMUM (20)
#define DIST_WARNING (40)

#define DEBUG (0)
#define _SERIAL if (DEBUG) Serial

int pinLB = 2; 
int pinLF = 4; 
int pinRB = 5; 
int pinRF = 7; 
int inputPin = 13; 

int outputPin = 12; 
int servoPin = 9; 

int Fspace = 0;
int Rspace = 0;
int Lspace = 0;
int directionn = STAY; 

Servo myservo;

int angle = ANGLE_MIDDLE; 
int prevAngle = ANGLE_MIDDLE; 
int incAngle = 1;

int distances[(ANGLE_LEFT_MAX-ANGLE_RIGHT_MAX)/ANGLE_STEP*2+2] = {0}; 

void setup() {
 Serial.begin(115200);

  pinMode (pinLB, OUTPUT);
  pinMode (pinLF, OUTPUT);
  pinMode (pinRB, OUTPUT);
  pinMode (pinRF, OUTPUT);

  pinMode (inputPin , INPUT );
  pinMode (outputPin, OUTPUT);

  myservo.attach(servoPin);
} 

void advance(int a) {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  delay(a);
} 
 

void turnR(int d) {
  digitalWrite(pinRB, LOW); 
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  delay(d);
} 

void turnL(int e) {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  delay(e);
} 

void stopp(int f) {

  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH); 
  digitalWrite(pinLF, HIGH);
  delay(f);
}

void back(int g) {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  delay(g);
} 

void rotateServoTo(int alpha) {
  _SERIAL.print("Rotating servo from ");
  _SERIAL.print(prevAngle, DEC);
  _SERIAL.print(" to ");
  _SERIAL.print(alpha, DEC);
  _SERIAL.print(" : Delay=");
  _SERIAL.println(abs(alpha-prevAngle)*2, DEC);
  if (prevAngle==alpha) return; 
  myservo.write(alpha);
  delay(abs(alpha-prevAngle)*2); 
  prevAngle = alpha;
  return;
} 

void detection() {

  Fspace = ask_pin(angle);
  
  _SERIAL.print("Front distance = ");
  _SERIAL.println(Fspace, DEC);
  if (Fspace<DIST_MINIMUM) { 
    _SERIAL.println("ALERT!, going back!");
    stopp(DELAY_STOP);
    back(DELAY_BACK_NEAR);

  }

  if (Fspace<DIST_WARNING) { 
    _SERIAL.println("Warning... look out!");
    stopp(DELAY_STOP);
    int i=0;
    int _run = 0;
    int _ini_run = 0;
    int _run_max = 0;
    int _ini_run_max = 0;
    if (prevAngle < ANGLE_MIDDLE) {
    rotateServoTo(ANGLE_RIGHT_MAX);
    for (int alpha=ANGLE_RIGHT_MAX;
    alpha<=ANGLE_LEFT_MAX;
    alpha+=ANGLE_STEP, i++) {
    distances[i] = ask_pin(alpha);
    _SERIAL.print(" alpha=");
    _SERIAL.print(" : dist[]=");
    _SERIAL.println(distances[i], DEC);

    if (distances[i] > DIST_WARNING && alpha<=ANGLE_LEFT_MAX-ANGLE_STEP) {
      _run += ANGLE_STEP;
    } else {
      if (_run > _run_max) {
         _run_max = _run;
         _ini_run_max = _ini_run;
      }
      _run = 0;
      _ini_run = alpha;
    }
  }
} else {
  
    rotateServoTo(ANGLE LEFT MAX);
    for (int alpha=ANGLE_LEFT_MAX;
      alpha>=ANGLE_RIGHT_MAX;
      alpha-=ANGLE_STEP, i++) {
      distances[i] = ask_pin(alpha);
      _SERIAL.print(" alpha=");
      _SERIAL.print(alpha, DEC);
      _SERIAL.print(" : dist[]=");
      _SERIAL.println(distances[i], DEC);
      if (distances[i] > DIST_WARNING && alpha>=ANGLE_RIGHT_MAX+ANGLE_STEP) {
      _run -= ANGLE_STEP;
      } else {
        if (_run < _run_max) {
          _run_max = run;
          _ini_run_max = _ini_run;
        }
        _run = 0;
        _ini_run = alpha;
      }
      _SERIAL.print(" _ini_run=");
      _SERIAL.print(_ini_run, DEC);
      _SERIAL.print(" : _run=");
      _SERIAL.print(_run, DEC);
      _SERIAL.print(" : _ini_run_max=");
      _SERIAL.print(_ini_run_max, DEC);
      _SERIAL.print(" : _run_max=");
      _SERIAL.println(_run_max, DEC);
    } 
    _ini_run_max = _ini_run_max + _run_max; 
    _run_max = -_run_max;
    _SERIAL.print(" END->CHANGE: _ini_run=");
    _SERIAL.print(_ini_run, DEC);
    _SERIAL.print(" : _run=");
    _SERIAL.print(_run, DEC);
    _SERIAL.print(" : _ini_run_max=");
    _SERIAL.print(_ini_run_max, DEC);
    _SERIAL.print(" : _run_max=");
    _SERIAL.println(_run_max, DEC);
   } 
  _SERIAL.print("_run_max=");
  _SERIAL.print(_run_max, DEC);
  _SERIAL.print(" : _ini_run_max=");
  _SERIAL.print(_ini_run_max, DEC);
  if (_run_max >= WINDOW_MINIMUM) {

    angle = _ini_run_max + _run_max/2;
    _SERIAL.print(" : alpha=");
    _SERIAL.println(angle, DEC);
    if (angle < ANGLE_WINDOW_RIGHT) {
      directionn = RIGHT; 
      _SERIAL.print(" --> RIGHT");
      } else if (angle > ANGLE_WINDOW_LEFT) {
      directionn = LEFT ; 
      _SERIAL.print(" --> LEFT");
      } else {
      directionn = FRONT; 
      _SERIAL.print(" --> FRONT");
      }
      angle = ANGLE_MIDDLE; 
      _SERIAL.print(" : angle=");
      _SERIAL.println(angle, DEC);
      rotateServoTo(angle);
      } else { 
      directionn = BACK;
   
      _SERIAL.println(" --> BACK");
      }
   } else { 
    directionn = FRONT;
  
    _SERIAL.println("No problem, continue --> FRONT");
    }
  } 

int ask_pin(int alpha) {

  rotateServoTo(alpha);
  digitalWrite(outputPin, LOW ); 
  delayMicroseconds(3);
  digitalWrite(outputPin, HIGH); 
  delayMicroseconds(15);
  digitalWrite(outputPin, LOW ); 
  float Fdistance = pulseIn(inputPin, HIGH); 
  Fdistance = Fdistance/58;
  _SERIAL.print("Fdistance:");
  _SERIAL.println(Fdistance);
  return (int) Fdistance;
  } 

void loop() {
  detection();
 if (directionn == BACK ){
  back(DELAY_BACK_BACK); 
  turnL(DELAY_BACK_RND); 
  _SERIAL.print("Back"); 
 }
 if (directionn == RIGHT){
  back(DELAY_BACK_TURN);
  turnR(DELAY_TURN);
  _SERIAL.print("Right");
 }
 if (directionn == LEFT ){
  back(DELAY_BACK_TURN);
  turnL(DELAY_TURN);
  _SERIAL.print("Left"); 
}
 if (directionn == FRONT){
  advance(DELAY_FRONT); 
  _SERIAL.print("Front");
  angle += incAngle;

 if (angle >= ANGLE_LEFT_VISION ) incAngle = -ANGLE_STEP;
 else if (angle <= ANGLE_RIGHT_VISION) incAngle = +ANGLE_STEP;
 }
 Distancia.data = d;
 chatter.publish(&Distancia);
 nh.spinOnce();
} 
