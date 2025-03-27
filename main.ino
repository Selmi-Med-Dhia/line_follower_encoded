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
volatile int previousEncoderRCount = 0;
volatile double speedR = 0;
volatile unsigned long previousMeasureTimeR = 0;
int targetR;
double targetSpeedR = 200;
float speedCorrectionR = 1.0;

volatile int encoderLCount = 0;
volatile int previousEncoderLCount = 0;
volatile double speedL = 0;
volatile unsigned long previousMeasureTimeL = 0;
int targetL;
double targetSpeedL = 200;
float speedCorrectionL = 1.0;

bool blind = false;
float kpE = 1;
float kiE = 2.5;
float kdE = 3.0;
float ksE = 0.08;
int previousErrorR = 0;
int previousErrorL = 0;

int maxSpeed = 170;
int minSpeed = 70;

int rightCorrection, leftCorrection;
int nbrOssilationsL = 0;
int previousValueL = 0;
int nbrOssilationsR = 0;
int previousValueR = 0;
float full360 = 2.344;

int getEncoderCorrectionR(){
  int error = targetR - encoderRCount;
  int value = (int)( ksE*( kpE*error + kdE*(error - previousErrorR) + kiE*(error + previousErrorR)) );
  if( (value > 0 && previousValueR <= 0) || (value < 0 && previousValueR >= 0)){
    nbrOssilationsR++;
    if (nbrOssilationsR>6){
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
    if (nbrOssilationsL>6){
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
  unsigned long tmp = micros();
  speedR = ( ( (double)(encoderRCount - previousEncoderRCount)*(60000000/202) ) / (tmp - previousMeasureTimeR) );
  previousEncoderRCount = encoderRCount;
  previousMeasureTimeR = tmp;
}

void IRAM_ATTR encoderLISR() {
  if(digitalRead(encoderLB) == digitalRead(encoderLA)){
    encoderLCount++;
  }else{
    encoderLCount--;
  }
  unsigned long tmp = micros();
  speedL = ( ( (double)(encoderLCount - previousEncoderLCount)*(60000000/202) ) / (tmp - previousMeasureTimeL) );
  previousEncoderLCount = encoderLCount;
  previousMeasureTimeL = tmp;
}
void speedRight(int speed){
  if (speed >=0){
    analogWrite(motorRA, 0);

    analogWrite(motorRB, speed);
  }else{
    analogWrite(motorRB, 0);
    analogWrite(motorRA, (-1)*speed);
  }
}

void speedLeft(int speed){
  if(speed >=0){
    analogWrite(motorLA, 0);
    analogWrite(motorLB, speed);
  }else{
    analogWrite(motorLB, 0);
    analogWrite(motorLA, (-1)*speed);
  }
}

void setTargetL(float nbrOfRotations){
  targetL += (int)(nbrOfRotations*202);
  nbrOssilationsL = 0;
  previousValueL = 0;
  previousErrorL = 0;
}
void setTargetR(float nbrOfRotations){
  targetR += (int)(nbrOfRotations*202);
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

    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISR, CHANGE);
    while(digitalRead(pushButton) == 0){
      delay(1);
    }
    delay(300);
}

void goToTargets(double targetSpeedRight, double targetSpeedLeft){
  rightCorrection = getEncoderCorrectionR();
  leftCorrection = getEncoderCorrectionL();
  speedCorrectionR = 1;
  speedCorrectionL = 1;

  int i = 0;
  int rampLength = min(1200, (int)(max( ( ( (float)abs(targetR-encoderRCount)/202 )/targetSpeedRight )*60000, ( ( (float)abs(targetL-encoderLCount)/202 )/targetSpeedLeft )*60000 ) /1.0) );
  float stepR = (float)(targetSpeedRight-40)/rampLength;
  float stepL = (float)(targetSpeedLeft-40)/rampLength;
  targetSpeedR = 40 - stepR*10;
  targetSpeedL = 40 - stepL*10;
  
  bool breaking = false;
  double timeLeft;

  while(leftCorrection != 0 || rightCorrection != 0){
    //ramping
    if (i <= rampLength){
      if (i%10 == 0){
        targetSpeedR += stepR*10;
        targetSpeedL += stepL*10;
      }
      i++;
      if (i>rampLength && abs(targetSpeedLeft - targetSpeedL) < 12 && abs(targetSpeedRight - targetSpeedR) < 12){
        targetSpeedR = targetSpeedRight;
        targetSpeedL = targetSpeedLeft;
      }
    }
    
    //breaking
    if (breaking == false){
      timeLeft = max( ( ( (float)abs(targetR-encoderRCount)/202 )/targetSpeedRight )*60000, ( ( (float)abs(targetL-encoderLCount)/202 )/targetSpeedLeft )*60000 );
      if(timeLeft < rampLength*( (float)max(targetSpeedLeft, targetSpeedRight)/600 ) ){
        breaking = true;
        stepR = stepR/( (float)max(targetSpeedLeft, targetSpeedRight)/600 );
        stepL = stepL/( (float)max(targetSpeedLeft, targetSpeedRight)/600 );
      }
    }else{
      targetSpeedR = max( 85.0, targetSpeedR - stepR);
      targetSpeedL = max( 85.0, targetSpeedL - stepL);
    }

    //speed control
    if (abs(encoderRCount - targetR) > 20){
      speedCorrectionR = max ( 0.2 , speedCorrectionR + (targetSpeedR - abs(speedR))*0.0004 );
    }else{
      speedCorrectionR = 1;
    }
    if (abs(encoderLCount - targetL) > 20){
      speedCorrectionL = max ( 0.2 , speedCorrectionL + (targetSpeedL - abs(speedL))*0.0004 );
    }else{
      speedCorrectionL = 1;
    }

    //position control
    speedRight(speedCorrectionR*rightCorrection);
    speedLeft(speedCorrectionL*leftCorrection);
    delayMicroseconds(1000);
    rightCorrection = getEncoderCorrectionR();
    leftCorrection = getEncoderCorrectionL();
    //Serial.println(targetSpeedL);
  }
  stop();
}

void stop(){
  for(int i = 0;i<3;i++){
    speedRight(0);
    speedLeft(0);
    delay(10);
  }
  speedR = 0;
  speedL = 0;
}
void loop() {
  setTargetL(4);
  setTargetR(4);
  goToTargets(300,300);
  delay(200);
  setTargetL(full360/2);
  setTargetR(-full360/2);
  goToTargets(200,200);
  delay(200);
  setTargetL(4);
  setTargetR(4);
  goToTargets(300,300);
  delay(200);
  setTargetL(-full360/2);
  setTargetR(full360/2);
  goToTargets(200,200);
  delay(10000);
}
