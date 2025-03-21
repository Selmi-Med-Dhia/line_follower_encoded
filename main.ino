int sensors[8] = {15, 12, 13, 14, 25, 26, 27, 35};
int pushButton = 34;
int motorRA = 19;
int motorRB = 21;
int motorLA = 22;
int motorLB = 23;
int encoderRA = 33;
int encoderRB = 18;
int encoderLA = 5;
int encoderLB = 4;

volatile int encoderRCount = 0;
int targetR;
volatile int encoderLCount = 0;
int targetL;

bool blind = false;
float kpE = 1;
float kiE = 0.5;
float kdE = 0.5;
float ksE = 1;
int previousErrorR = 0;
int previousErrorL = 0;

int getEncoderCorrectionR(){
  int error = targetR - encoderRCount;
  int value = (int)( ksE*( kpE*error + kd*(error - previousErrorR) + ki*(error + previousErrorR)) );
  previousErrorR = error;
  return value;
}

int getEncoderCorrectionL(){
  int error = targetL - encoderLCount;
  int value = (int)( ksE*( kpE*error + kd*(error - previousErrorL) + ki*(error + previousErrorL)) );
  previousErrorL = error;
  return value;
}

void IRAM_ATTR encoderRISR() {
  if(digitalRead(encoderRB)){
    encoderRCount++;
  }else{
    encoderRCount--;
  }
}

void IRAM_ATTR encoderLISR() {
  if(digitalRead(encoderLB)){
    encoderLCount++;
  }else{
    encoderLCount--;
  }
}

void speedRight(int speed){
  if (speed >0){
    analogWrite(motorRA, 0);
    analogWrite(motorRB, speed);
  }else{
    analogWrite(motorRB, 0);
    analogWrite(motorRA, (-1)*speed);
  }
}

void speedLeft(int speed){
  if(speed >0){
    analogWrite(motorLA, 0);
    analogWrite(motorLB, speed);
  }else{
    analogWrite(motorLB, 0);
    analogWrite(motorLA, (-1)*speed);
  }
}

void setup() {
    pinMode(encoderRA, INPUT);
    pinMode(encoderRB, INPUT);
    pinMode(encoderLA, INPUT);
    pinMode(encoderLB, INPUT);
    pinMode(pushButton, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISR, RISING);
}

void loop() {
  if(blind){
    speedRight(getEncoderCorrectionR());
    speedLeft(getEncoderCorrectionL());
    delay(1);
    if(previousErrorR < 2 && previousErrorL < 2){
      blind = false;
    }
  }else{
    //PIDing with the black and white sensors
  }
}
