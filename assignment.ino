#include <PID_v1.h>
#include <TimerOne.h>

#define STOP      0
#define MOVEFORWARD   1
#define TURNLEFT  2
#define TURNRIGHT 3
#define TURNBACK  4

const int in1 = 4;
const int in2 = 5;
const int in3 = 7;
const int in4 = 6;

const int trig = 8;  // chân trig của HC-SR04
const int echo1 = 9; // chân echo của HC-SR04 -> 1 H  
const int echo2 = 10; // chân echo của HC-SR04 -> 2 L
const int echo3 = 11; // chân echo của HC-SR04 -> 3 R

float diskslots = 20.00; 
const float stepcount = 20.00;
const float wheeldiameter = 60.00;

const int encoderA = 2; // left
volatile int pulse_left = 0;
const int encoderB = 3; // right
volatile int pulse_right = 0;

const int distance_limit = 10 ; // cm
int mang[100];
int idx = 0;
int dodai = 0;

unsigned long PingTime[3];
volatile int toComplete;

int distance_head = 0;
int distance_left = 0;
int distance_right = 0;

const int echoPins[] = {9, 10, 11}; 
volatile bool echoReceived[3] = {false};
volatile unsigned long echoStartTimes[3];
volatile unsigned long echoEndTimes[3];
volatile unsigned long distance[3]; 

       // Khai báo các biến PID
double setpoint = 30.0;  // Giá trị mục tiêu
double input, output;
double Kp = 0.5, Ki = 0.5, Kd = 1.0;  // Hằng số PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


// PID parameters
double prevInput_left = 0;   // Previous input for derivative calculation
double prevInput_right = 0;   // Previous input for derivative calculation
double integral_left = 0;    // Integral term
double derivative_left = 0;  // Derivative term
double integral_right = 0;    // Integral term
double derivative_right = 0;  // Derivative term
double tocdoleft, tocdoright;
// Initialize the encoder interrupt
void doEncoderA(){
  pulse_left++;
}

void doEncoderB(){
  pulse_right++;
}

double pid(int input, int setpoint, double kp, double ki, double kd, double integral, double derivative, double prevInput){
  double error = setpoint - input;
  derivative = input - prevInput;
  integral += error;

  double output = kp*error + ki*integral + kd*derivative;
  output = constrain(output, 0, 255);
  prevInput = input;
  return output;
}

float getDistance(int trig, int echo){
  float distance = 0;
  unsigned long duration = 0;
  /* Phát xung từ chân trig */
//  digitalWrite(trig, LOW);   // tắt chân trig
//  delayMicroseconds(2);
  digitalWrite(trig, HIGH);   // phát xung từ chân trig
  delayMicroseconds(10);   // xung có độ dài 5 microSeconds
  digitalWrite(trig, LOW);   // tắt chân trig
  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo. 
  distance = pulseIn(echo, HIGH, 30000) / 58.0;  
  // Tính khoảng cách đến vật.
//  distance = int(duration / 2 / 29.412); // duration * 0.0034 / 2.0; // duration/2/29.1
  if(distance == 0) distance = 200;
  return distance;
}

// TimerOne ISR
void ISR_timerone(){
  Timer1.detachInterrupt(); // stop timer
//  float rotation_left = (pulse_left/diskslots)*60.00; // calculator rpm left motor
//  tocdoleft = pid(rotation_left, 120, 1, 3, 0.01, integral_left, derivative_left, prevInput_left);
//  Serial.print("Motor speed left: ");
//  Serial.println(tocdoleft);
//  float rotation_right = (pulse_right/diskslots)*60.00; // calculator rpm left motor
//  tocdoright = pid(rotation_right, 140, 1, 0.1, 0, integral_right, derivative_right, prevInput_right);
//  Serial.print("Motor speed right: ");
//  Serial.println(tocdoright);
//  digitalWrite(in3, HIGH);
//    analogWrite(in4, tocdoleft);
//    digitalWrite(in1, HIGH);
//     analogWrite(in2, tocdoright);
  Serial.print("Motor speed left: ");
  float rotation1 = (pulse_left/diskslots)*60.00; // calculator rpm left motor
  Serial.print(rotation1);
//  Serial.print(" RPM - ");
  pulse_left = 0;
  Serial.print("Motor speed right: ");
  float rotation2 = (pulse_right/diskslots)*60.00; // calculator rpm left motor
  Serial.print(rotation2);
  Serial.println(" RPM");
  pulse_right = 0;
  Timer1.attachInterrupt(ISR_timerone); // enable the timer
}

// pIN change Interrupt
ISR(PCINT0_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a pins 8~13
//    if (digitalRead(echoPins[0]) == HIGH) {
//      echoStartTimes[0] = micros();
//    } else {
//      echoEndTimes[0] = micros();
////      distance[0] = calculateDistance(0);
//      echoReceived[0] = true;
//    }

}

// Hàm đọc giá trị encoderCount
int readEncoderCount() {
  int count;
  noInterrupts();
  count = pulse_left;
  interrupts();
  return count;
}

void PingTrigger() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); // Trigger another pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

unsigned long calculateDistance(int sensorIndex) {
  unsigned long duration = echoEndTimes[sensorIndex] - echoStartTimes[sensorIndex];
  unsigned long distance = duration / 58;  // Divide by 58 to convert microseconds to centimeters
  return distance;
}

void pciSetup(byte pin)
{
  pinMode(pin, INPUT);
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void setup()
{
  Serial.begin(9600);     // giao tiếp Serial với baudrate 9600
//  tocdoleft = 0; tocdoright = 0;
  
  Timer1.initialize(1000000); // set timer for one second
  
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

//  PCICR |= B00000001;
//  PCMSK0 |= B00001110;

  pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
  pciSetup(echo1);
  pciSetup(echo2);
  pciSetup(echo3);
//  pinMode(echo1,INPUT);    // chân echo sẽ nhận tín hiệu
//  pinMode(echo2,INPUT);
//  pinMode(echo3,INPUT);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  
  attachInterrupt(0, doEncoderA, RISING);
  attachInterrupt(1, doEncoderB, RISING);
  Timer1.attachInterrupt(ISR_timerone);

// Khởi tạo giá trị ban đầu cho PID
  input = readEncoderCount();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-200, 200);  // Giới hạn tín hiệu điều khiển PWM (-255 đến 255)
    
}

void loop()
{    

//  unsigned long starttime, endtime, waitcount;
//  waitcount = 0;
//  starttime = millis();
//  PingTrigger();
//  while (echoReceived[0] == false  ) {
//    waitcount++;
//    endtime = millis();
//  }

//  for(int i = 0; i < 3; i++){
//    if(echoReceived[i] == true){
//      distance[i] = calculateDistance(i);
//      echoReceived[i] = false;
//    }
//
//    Serial.print("Distance from sensor ");
//    Serial.print(i+9);
//    Serial.print(": ");
//    Serial.print(distance[i]);
//    Serial.println(" cm");
//  }
    input = readEncoderCount();  // Đọc giá trị từ module H206

    myPID.Compute();  // Tính toán tín hiệu điều khiển PID

    // motor right
//    digitalWrite(in1, HIGH);
//    analogWrite(in2, 140);
  
    // motor left
    if (output > 0) {
//      digitalWrite(in3, HIGH);
//      analogWrite(in4, 255 - output);
    analogWrite(in4, output);  // Chiều thuận
    } else {
//      digitalWrite(in3, LOW);
//      analogWrite(in4, -output);
    analogWrite(in4, -output);  // Chiều ngược
    }
//    Serial.println(output);
    

//  motorRun(MOVEFORWARD);
//  motorRun(TURNLEFT);
//  delay(540); // 90 degree
//  motorRun(TURNRIGHT);
//  delay(580); // 90 degree
//  motorRun(STOP);
    
//  readSensor();
//  actionMotor();
  
//  delay(1000); 
//  for(int i = 0; i < dodai; i++){
//    Serial.print("Chay = ");
//    Serial.print(mang[i]);
//  }
//  Serial.println("");
}

void readSensor(){
  
  distance_head = getDistance(trig,echo1);
  delay(10);
  distance_left = getDistance(trig,echo2);
  delay(10);
  distance_right = getDistance(trig,echo3);
  delay(10);
  
//  Serial.print("Sensor head: ");
//  Serial.print(distance_head);
//  Serial.println("cm");
//  Serial.print("Sensor left: ");
//  Serial.print(distance_left);
//  Serial.println("cm");  
//  Serial.print("Sensor right: ");
//  Serial.print(distance_right);
//  Serial.println("cm"); 
}

void actionMotor(){
  /*Actions*/
  if(distance_head < distance_limit + 2){
    motorRun(STOP);
    // front and right => turn left
    if(distance_left > distance_limit && distance_right < distance_limit){
      mang[idx] = TURNLEFT;
      idx++;
      dodai++;
       
      motorRun(TURNLEFT);
      delay(540); // 90 degree
    } 
    // front and left => turn right
    else if(distance_right > distance_limit && distance_left < distance_limit){ 
      mang[idx] = TURNRIGHT;
      idx++;
      dodai++;
      
      motorRun(TURNRIGHT);
      delay(580); // 90 degree
    } 
    // all 3 sides
    else if(distance_right < distance_limit - 3 && distance_left < distance_limit - 3){
      mang[idx] = TURNBACK;
      idx++;
      dodai++;
      // more action
      motorRun(TURNBACK);
      motorRun(STOP);
    }
    else {    
      if(refactor() && (distance_right > 2 * distance_limit &&  distance_left > 2 * distance_limit)){ // loop
        int temp = mang[idx-2];
        idx = idx - (4*2);
        dodai = dodai - (4*2);
        // select another way
        if(temp == TURNLEFT){
          mang[idx] = TURNRIGHT;
          idx++;
          dodai++;
          motorRun(TURNRIGHT);
          delay(580); // 90 degree
        } else if(temp == TURNRIGHT){
          mang[idx] = TURNLEFT;
          idx++;
          dodai++;
          motorRun(TURNLEFT);
          delay(540); // 90 degree
        }
      } else {
        if(distance_left > distance_right){
          mang[idx] = TURNLEFT;
          idx++;
          dodai++;
          motorRun(TURNLEFT);
          delay(540); // 90 degree
        } else if(distance_left < distance_right){
          mang[idx] = TURNRIGHT;
          idx++;
          dodai++;
          motorRun(TURNRIGHT);
          delay(580); // 90 degree
        }
        motorRun(STOP);
        }
    }
  } else {
//    if(distance_left > distance_limit + 5 && distance_right < distance_limit){ // có ngã rẽ
//      motorRun(STOP);
//      mang[idx] = TURNLEFT;
//      idx++;
//      dodai++;
//      motorRun(TURNLEFT);
//      delay(560); // 90 degree
//    } else if(distance_right > distance_limit + 5 && distance_left < distance_limit){
//      motorRun(STOP);
//      mang[idx] = TURNRIGHT;
//      idx++;
//      dodai++;
//      motorRun(TURNRIGHT);
//      delay(600); // 90 degree
//    } else 
    if(distance_right < 3 && distance_left > distance_right){
      motorRun(TURNLEFT);
      delay(25);
    } else if(distance_left < 3 && distance_right > distance_left){
      motorRun(TURNRIGHT);
      delay(25);
    } else {
      if(mang[idx-1] != MOVEFORWARD){
        mang[idx] = MOVEFORWARD;
        idx++;
        dodai++;
      }
      motorRun(MOVEFORWARD); 
    }
  }
}

bool refactor(){
  if(mang[idx-2] == mang[idx-4] && mang[idx-2] == mang[idx-6] && mang[idx-2] == mang[idx - 8]){
    return true;
  }
  return false;
}

void motorRun(int action){
  switch(action){
    case 1: // forward
//      Serial.println("MOVE FORWARD");
      // motor right
      digitalWrite(in1, HIGH);
      analogWrite(in2, 140);
    
      // motor left
      digitalWrite(in3, HIGH);
      analogWrite(in4, 110);
      break;
    case 2: //turn left
      Serial.println("TURN LEFT");
      digitalWrite(in1, HIGH);
      analogWrite(in2, 50);
    
      digitalWrite(in3, LOW);
      analogWrite(in4, 200);
      
      break;
    case 3: // turn right
      Serial.println("TURN RIGHT");
      digitalWrite(in1, LOW);
      analogWrite(in2, 200);
    
      digitalWrite(in3, HIGH);
      analogWrite(in4, 40);
      
      break;
    case 4: // backward
      Serial.println("Move Backward");
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      delay(1050); // turn 180 degree
      break;
    default: // stop
//      Serial.println("STOP");
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      delay(1000);
      break;
  }
}
