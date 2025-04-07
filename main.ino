#define PI 3.14159265358979323846

int sensors[8] = {35, 27, 26, 25, 14, 13, 12, 15};
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
int currentVoltageR = 100;
double previousSpeedErrorR = 0;
double newErrorR;
volatile unsigned long previousMeasureTimeR = 0;
int targetR;
double targetSpeedR = 100;
double previousTargetSpeedR = 100;
float speedCorrectionR = 1.0;

volatile int encoderLCount = 0;
volatile int previousEncoderLCount = 0;
volatile double speedL = 0;
int currentVoltageL = 100;
double previousSpeedErrorL = 0;
double newErrorL;
volatile unsigned long previousMeasureTimeL = 0;
int targetL;
double targetSpeedL = 100;
double previousTargetSpeedL = 100;
float speedCorrectionL = 1.0;

float kpE = 1;
float kiE = 2.5;
float kdE = 3.0;
float ksE = 0.1;
int previousErrorR = 0;
int previousErrorL = 0;

float kpS = 0.3;
float kiS = 0.02;
float kdS = 0.05;
float ksS = 0.13;

int maxSpeed = 170;
int minSpeed = 70;

int rightCorrection, leftCorrection;
int nbrOssilationsL = 0;
int previousValueL = 0;
int nbrOssilationsR = 0;
int previousValueR = 0;
float full360 = 2.35;

int weightsPreferingRight[8] = {-200,-20,-10,-10,10,160,180,500};

int weights[8] = {-180,-50,-12,-10,10,12,50,180};

int weights300[8] = {-700,-50,-20,-15,15,20,50,700};
int weights170[8] = {-180,-50,-12,-10,10,12,50,180};

bool flags[10] = { false, false, false, false, false, false, false, false, false, false};

int oldSums[8] = {0,0,0,0,0};
int threashholds[8] = {0,0,0,0,0,0,0,0};
float kp = 1;
float kd = 0.33;
float ki = 0;
float ks = 0.3;
bool blackOnWhite = true;
float tmp;
int baseRPM = 170;

float getPIDValue(){
  int sum = 0;
  for(int j=0; j<1; j++){
    for(int i=0; i<8; i++){
      sum += getValue(i) * weights[i] * 4;
    }
    delayMicroseconds(10);
  }
  float value =    kp*sum   +    kd*(sum - oldSums[4])   +  ki*(sum + oldSums[4]);
  for(int i=1;i<4; i++){
    oldSums[i] = oldSums[i-1];
  }
  oldSums[0] = sum;
  return ks*value;
}

int getValue(int sensor){
  if(blackOnWhite){
    return( analogRead(sensors[sensor]) > threashholds[sensor]);
  }else{
    return( analogRead(sensors[sensor]) < threashholds[sensor]);
  }
}

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
  if(encoderRCount%2){
    unsigned long tmp = micros();
    speedR = ( ( (double)(encoderRCount - previousEncoderRCount)*(60000000/202) ) / (tmp - previousMeasureTimeR) );
    previousEncoderRCount = encoderRCount;
    previousMeasureTimeR = tmp;
  }
}

void IRAM_ATTR encoderLISR() {
  if(digitalRead(encoderLB) == digitalRead(encoderLA)){
    encoderLCount++;
  }else{
    encoderLCount--;
  }
  if(encoderLCount%2){
    unsigned long tmp = micros();
    speedL = ( ( (double)(encoderLCount - previousEncoderLCount)*(60000000/202) ) / (tmp - previousMeasureTimeL) );
    previousEncoderLCount = encoderLCount;
    previousMeasureTimeL = tmp;
  }
}

void calibrate(){
  setTargetR(full360*3);
  setTargetL(-full360*3);
  int min[8] = {4000,4000,4000,4000,4000,4000,4000,4000};
  int reading;
  rightCorrection = getEncoderCorrectionR();
  leftCorrection = getEncoderCorrectionL();

  while(leftCorrection != 0 || rightCorrection != 0){
    for(int i=0;i<8;i++){
      reading = analogRead(sensors[i]);
      if(min[i] > reading ){
        min[i] = reading;
      }
      if(threashholds[i] < reading ){
        threashholds[i] = reading;
      }
    }

    speedRight(rightCorrection);
    speedLeft(leftCorrection);
    delayMicroseconds(1000);
    rightCorrection = getEncoderCorrectionR();
    leftCorrection = getEncoderCorrectionL();
  }
  stop();
  for(int i=0;i<8;i++){
    threashholds[i] = (threashholds[i] + min[i])/2 ;
  }
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

void turn(float degree){
  setTargetR(full360*(degree/360));
  setTargetL(full360*(degree/-360));

  rightCorrection = getEncoderCorrectionR();
  leftCorrection = getEncoderCorrectionL();

  while(leftCorrection != 0 || rightCorrection != 0){
    speedRight(rightCorrection);
    speedLeft(leftCorrection);
    delayMicroseconds(1000);
    rightCorrection = getEncoderCorrectionR();
    leftCorrection = getEncoderCorrectionL();
  }
  stop();
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
      if(timeLeft < rampLength*( (float)max(targetSpeedLeft, targetSpeedRight)/1000 ) ){
        breaking = true;
        stepR = stepR/( (float)max(targetSpeedLeft, targetSpeedRight)/1000 );
        stepL = stepL/( (float)max(targetSpeedLeft, targetSpeedRight)/1000 );
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

int distanceToTicks(float distance){
  return (int)( ( distance / (66.0*PI) )*202 );
}

void keepWalking(float distance){
  int targetEncoderRight = encoderRCount + distanceToTicks(distance);
  double targetSpeed = (targetSpeedL + targetSpeedR) / 2;
  while (encoderRCount < targetEncoderRight ){
    newErrorR = speedR - targetSpeed;
    newErrorL = speedL - targetSpeed;
    currentVoltageR -= (int)( ksS*( kpS*newErrorR + kdS*(newErrorR - previousSpeedErrorR) + kiS*(newErrorR + previousSpeedErrorR) ) );
    currentVoltageL -= (int)( ksS*( kpS*newErrorL + kdS*(newErrorL - previousSpeedErrorL + kiS*(newErrorL+ previousSpeedErrorL) ) ) );
    currentVoltageR = max(-255, min(255, currentVoltageR) );
    currentVoltageL = max(-255, min(255, currentVoltageL) );
    speedRight(currentVoltageR);
    speedLeft(currentVoltageL);
    previousSpeedErrorR = newErrorR;
    previousSpeedErrorL = newErrorL;
    delayMicroseconds(100);
  }
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

void setup() {
  pinMode(encoderRA, INPUT);
  pinMode(encoderRB, INPUT);
  pinMode(encoderLA, INPUT);
  pinMode(encoderLB, INPUT);
  pinMode(pushButton, INPUT);
  pinMode(2, OUTPUT);

  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISR, CHANGE);
  
  encoderRCount = 0;
  encoderLCount = 0;
  digitalWrite(2, HIGH);
  calibrate();
  delay(2000);
  digitalWrite(2, LOW);
  /*
  setTargetL( ( 400.0 / (66.0*3.14) ) );
  setTargetR( ( 400.0 / (66.0*3.14) ) );
  goToTargets(400,400);
  */
  delay(3000);
  setTargetL( distanceToTicks(630)/202 );
  setTargetR( distanceToTicks(630)/202 );
  goToTargets(300,300);
  turn(-94);
  setTargetL( distanceToTicks(280)/202 );
  setTargetR( distanceToTicks(280)/202 );
  goToTargets(150,150);
  turn(-94);
  setTargetL( distanceToTicks(260)/202 );
  setTargetR( distanceToTicks(260)/202 );
  goToTargets(150,150);
  turn(94);
  setTargetL( distanceToTicks(360)/202 );
  setTargetR( distanceToTicks(360)/202 );
  goToTargets(300,300);
  turn(-94);
  setTargetL( distanceToTicks(270)/202 );
  setTargetR( distanceToTicks(270)/202 );
  goToTargets(150,150);
  turn(40);
  setTargetL( distanceToTicks(170)/202 );
  setTargetR( distanceToTicks(170)/202 );
  goToTargets(150,150);
}

void loop(){
  //Serial.println(encoderLCount - encoderRCount);

  tmp = getPIDValue();
  targetSpeedR = max(-570.0f, min( 570.0f, baseRPM - tmp) );
  targetSpeedL = max(-570.0f, min( 570.0f, baseRPM + tmp) );
  
  //jumping to speed
  if(targetSpeedR != previousTargetSpeedR){
    currentVoltageR = (targetSpeedR / previousTargetSpeedR) * currentVoltageR;
    currentVoltageL = (targetSpeedL / previousTargetSpeedL) * currentVoltageL;
    speedRight(currentVoltageR);
    speedLeft(currentVoltageL);
    delay(1);
    previousTargetSpeedL = targetSpeedL;
    previousTargetSpeedR = targetSpeedR;
  }

  newErrorR = speedR - targetSpeedR;
  newErrorL = speedL - targetSpeedL;
  currentVoltageR -= (int)( ksS*( kpS*newErrorR + kdS*(newErrorR - previousSpeedErrorR) + kiS*(newErrorR + previousSpeedErrorR) ) );
  currentVoltageL -= (int)( ksS*( kpS*newErrorL + kdS*(newErrorL - previousSpeedErrorL + kiS*(newErrorL+ previousSpeedErrorL) ) ) );
  currentVoltageR = max(-255, min(255, currentVoltageR) );
  currentVoltageL = max(-255, min(255, currentVoltageL) );
  speedRight(currentVoltageR);
  speedLeft(currentVoltageL);
  previousSpeedErrorR = newErrorR;
  previousSpeedErrorL = newErrorL;

  /*
  if( !flags[0] && (encoderLCount - encoderRCount) > (-2310) ){
    baseRPM = 150;
    digitalWrite(2, HIGH);
    for(int i=0;i<8;i++){
      weights[i] = weightsPreferingRight[7-i];
    }
    weights[3] = 500;
    flags[0] = true;
  }
  /*
  if ( flags[0] && !flags[1] && (encoderLCount - encoderRCount) < (full360/2)*202 ){
    digitalWrite(2, LOW);
    for(int i=0;i<8;i++){
      weights[i] = weights[-i];
    }
    flags[1] = true;
  }
  /*
  if( abs(encoderRCount - 202.0*( .0 / (66.0*3.14) ) ) < 20 ){
    digitalWrite(2, HIGH);
    baseRPM = 170;
    for(int i=0;i<8; i++){
      weights[i] = weights170[i];
      Serial.println("haw tbaddel");
    }
  }
  /*
  if( abs(encoderRCount - 202.0*( 2350.0 / (66.0*3.14) ) ) < 20 ){
    digitalWrite(2, LOW);
    baseRPM = 300;
    for(int i=0;i<8; i++){
      weights[i] = weights300[i];
      Serial.println("haw tbaddel");
    }
  }
  */
}
