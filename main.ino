int sensors[8] = {15, 12, 13, 14, 25, 26, 27, 35};
int pushButton = 34;
int motorR1 = 19;
int motorR2 = 21;
int motorL1 = 22;
int motorL2 = 23;
int encoderRA = 33;
int encoderRB = 18;
int encoderLA = 5;
int encoderLB = 4;

volatile int encoderRCount = 0;
volatile int encoderLCount = 0;

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
}
