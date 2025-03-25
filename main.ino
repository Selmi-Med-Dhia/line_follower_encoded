int sensors[8] = {15, 12, 13, 14, 25, 26, 27, 35};
int pushButton = 34;
int motorRA = 21;
int motorRB = 19;
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
float kiE = 2.5;
float kdE = 3.0;
float ksE = 0.1;
int previousErrorR = 0;
int previousErrorL = 0;

int maxSpeed = 200;
int minSpeed = 70;

int rightCorrection, leftCorrection;
int nbrOssilationsL = 0;
int previousValueL = 0;
int nbrOssilationsR = 0;
int previousValueR = 0;

int getEncoderCorrectionR(){
  int error = targetR - encoderRCount;
  int value = (int)( ksE*( kpE*error + kdE*(error - previousErrorR) + kiE*(error + previousErrorR)) );
  if( (value > 0 && previousValueR <= 0) || (value < 0 && previousValueR >= 0)){
    nbrOssilationsR++;
    if (nbrOssilationsR>5){
      return (0);
    }
  }
  previousValueR = value;
  previousErrorR = error;
  if( abs(encoderRCount-targetR)<10 && abs(error - previousErrorR) < 2){
    return(0);
  }
  if (value >= 0){
    return( max( minSpeed, min( maxSpeed, value ) ) );
  }
  return( min( (-1)*minSpeed, max( (-1)*maxSpeed, value ) ) );
}

int getEncoderCorrectionL(){
  int error = targetL - encoderLCount;
  int value = (int)( ksE*( kpE*error + kdE*(error - previousErrorL) + kiE*(error + previousErrorL)) );

  if( (value > 0 && previousValueL <= 0) || (value < 0 && previousValueL >= 0)){
    nbrOssilationsL++;
    if (nbrOssilationsL>5){
      return (0);
    }
  }
  previousValueL = value;
  previousErrorL = error;
  if (value >= 0){
    return( max( minSpeed, min( maxSpeed, value ) ) );
  }
  return( min( (-1)*minSpeed, max( (-1)*maxSpeed, value ) ) );
}

void IRAM_ATTR encoderRISR() {
  if(digitalRead(encoderRB) == digitalRead(encoderRA)){
    encoderRCount--;
  }else{
    encoderRCount++;
  }
}

void IRAM_ATTR encoderLISR() {
  if(digitalRead(encoderLB) == digitalRead(encoderLA)){
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

void setTargetL(int target){
  targetL = target;
  nbrOssilationsL = 0;
  previousValueL = 0;
  previousErrorL = 0;
}
void setTargetR(int target){
  targetR = target;
  nbrOssilationsR = 0;
  previousValueR = 0;
  previousErrorR = 0;
}

void setup() {
    pinMode(encoderRA, INPUT);
    pinMode(encoderRB, INPUT);
    pinMode(encoderLA, INPUT);
    pinMode(encoderLB, INPUT);
    pinMode(pushButton, INPUT);

    Serial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISR, CHANGE);
    blind = true;
    setTargetL(202);
    setTargetR(-202);
}

void loop() {
  if(blind){
    /*
    Serial.print(getEncoderCorrectionL());
    Serial.print(" : ");
    Serial.println(previousErrorL);
    delay(10);
    */
    rightCorrection = getEncoderCorrectionR();
    leftCorrection = getEncoderCorrectionL();
    speedRight(rightCorrection);
    speedLeft(leftCorrection);
    delayMicroseconds(1000);
    if(leftCorrection == 0 && rightCorrection == 0){
      speedRight(0);
      speedLeft(0);
      //blind = false;
      setTargetL((-1)*targetL);
      setTargetR((-1)*targetR);
      delay(1000);
    }
  }else{
    //PIDing with the black and white sensors
  }
}
