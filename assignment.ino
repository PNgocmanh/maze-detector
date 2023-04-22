#include <TimerOne.h>

#include <PID_v1.h>
#include <NewPing.h>

const int in1 = 4;
const int in2 = 5;
const int in3 = 7;
const int in4 = 6;

const int trig = 8;  // chân trig của HC-SR04
const int echo1 = 9; // chân echo của HC-SR04 -> 1 H  
const int echo2 = 10; // chân echo của HC-SR04 -> 2 L
const int echo3 = 11; // chân echo của HC-SR04 -> 3 R

const int encoderA = 2; // left
const int encoderB = 3; // right

unsigned long previousMillis = 0;
long interval = 1000;
volatile boolean A_set = false;  // flag indicating the state of encoder pin A
volatile boolean B_set = false;  // flag indicating the state of encoder pin B


float speed;
double lastoutput;
double T = 0.01; // thoi gian lay mau
double E, E1, E2;
double alpha, beta, gamma;
double Kp = 1;
double Ki = 5;
double Kd = 0.001;

const int distance_limit = 10 ; // cm

int distance_head;
int distance_left;
int distance_right;

volatile int pulse = 0;
double error, error1, error2;
double kp = 5 , ki = 1 , kd = 0.01 ,input = 0, output = 0, setpoint = 0;   // modify kp, ki and kd for optimal performance
long temp;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void move_forward();
void turn_left();
void turn_right();
void backward();
void stop_motor();
void readSensor();
void actionMotor();

// Initialize the encoder interrupt
void motor(){
  if (digitalRead(3) == LOW) {
    pulse++;
  } else {
    pulse--;
  }
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if (digitalRead(encodPinB1)==HIGH)  count ++;
  else                      encoderPos--;             // if (digitalRead(encodPinB1)==LOW)   count --;
}


float getDistance(int trig, int echo){
  float distance = 0;
  unsigned long duration = 0;
  pinMode(trig,OUTPUT);
  
  /* Phát xung từ chân trig */
  digitalWrite(trig, LOW);   // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);   // phát xung từ chân trig
  delayMicroseconds(10);   // xung có độ dài 5 microSeconds
  digitalWrite(trig, LOW);   // tắt chân trig
  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo. 
  pinMode(echo, INPUT);
  distance = pulseIn(echo, HIGH, 30000) / 58.0;  
  // Tính khoảng cách đến vật.
//  distance = int(duration / 2 / 29.412); // duration * 0.0034 / 2.0; // duration/2/29.1
  if(distance == 0) distance = 200;
  return distance;
}

void setup()
{
  Serial.begin(9600);     // giao tiếp Serial với baudrate 9600
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

  pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo1,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(echo2,INPUT);
  pinMode(echo3,INPUT);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);

  speed = 0; E = 0 ; E1 = 0; E2 = 0;
  output = 0; lastoutput = 0;
  attachInterrupt(0, encoder, FALLING);
  Timer1.initialize(10000);
  Timer1.attachInterrupt(PID);

//  attachInterrupt(digitalPinToInterrupt(encoderA), updateEncoder, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoderB), updateEncoder, CHANGE);
  

}

void loop()
{  
//    readSensor();
//    actionMotor();
//  digitalWrite(in1, LOW);
//  analogWrite(in2, 100);
//
//  digitalWrite(in3, LOW);
//  analogWrite(in4, 100);

  temp += analogRead(0);                              // increment position target with potentiometer value (speed), potmeter connected to A0
  if (temp < 0) {                                     // in case of overflow
    pulse = 0;
    temp = 0;
  }
  setpoint = temp / 500;                              // modify division to fit motor and encoder characteristics
  input = pulse ;                                // data from encoder
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H-Bridge module
}
void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(in2, out);                             // drive motor CW        
    analogWrite(in1, 0);                                              
  }
  else {
    analogWrite(in2, 0);                         
    analogWrite(in1, abs(out));                        // drive motor CCW
  }

}


void PID(){
  speed = (pulse/1500)*(1/T)*60;
  pulse = 0;
  E = 500 - speed;
  alpha = 2*T*Kp + Ki * T * T + 2*Kd;
  beta = T*T*Ki - 4*Kd - 2*T*Kp;
  gamma = 2 *Kd;
  output = (alpha * E + beta * E1 + gamma * E2 +  2*T*lastoutput)/(2*T);
  lastoutput = output;
  E2 = E1;
  E1 = E;
  if(output > 255) output = 255;
  if(output < 0) output = 0;
}

void updateEncoder() {
  // read the current state of the encoder pins
  boolean A_val = digitalRead(encoderA);
  boolean B_val = digitalRead(encoderB);

  // determine which direction the encoder is turning
  if (A_val != A_set) {
    encoderPos += (B_val != A_val) ? 1 : -1;
  } else if (B_val != B_set) {
    encoderPos += (A_val == B_val) ? 1 : -1;
  }
  
  // update the flags for the next interrupt
  A_set = A_val;
  B_set = B_val;
}

void readSensor(){
      distance_head = getDistance(trig,echo1);
      distance_left = getDistance(trig,echo2);
      distance_right = getDistance(trig,echo3);
  
      Serial.print("Sensor head: ");
      Serial.print(distance_head);
      Serial.println("cm");
      Serial.print("Sensor left: ");
      Serial.print(distance_left);
      Serial.println("cm");  
      Serial.print("Sensor right: ");
      Serial.print(distance_right);
      Serial.println("cm");     
}

void actionMotor(){
  /*Actions*/
  distance_left = getDistance(trig,echo2);
  distance_right = getDistance(trig,echo3);

  if(distance_head < distance_limit){
    stop_motor();
    delay(1000);
    // front and right => turn left
    if(distance_left > distance_limit && distance_right < distance_limit){ 
      turn_left();
      delay(400); // 90 degree
    } 
    // front and left => turn right
    else if(distance_right > distance_limit && distance_left < distance_limit){ 
      turn_right();
      delay(400); // 90 degree
    } 
    // all 3 sides
    else if(distance_right < distance_limit && distance_left < distance_limit){
        // more action
        backward();
    }
    else {    
      if(distance_left > distance_right){
        turn_left();
        delay(400); // 90 degree
      } else {
        turn_right();
        delay(400); // 90 degree
      }
      stop_motor();
      delay(1000);
    }
  } else {
    if(distance_right < distance_limit && distance_left > distance_limit){
        turn_left();
        delay(25);
    } else if(distance_left < distance_limit && distance_right > distance_limit){
        turn_right();
        delay(25);
    } else {
      move_forward(); 
    }
  }
}

void move_forward(){
  Serial.println("Move Forward");
  // motor right
//  analogWrite(in1, speed_motor_right);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW); 

  // motor left
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turn_left(){
  Serial.println("Turn left");
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turn_right(){
  Serial.println("Turn right");
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stop_motor(){
  Serial.println("Stop move");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void backward(){
  Serial.println("Move Backward");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  delay(800); // turn 180 degree
}
