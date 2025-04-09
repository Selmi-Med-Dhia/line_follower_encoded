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
int minSpeed = 100;

int rightCorrection, leftCorrection;
int nbrOssilationsL = 0;
int previousValueL = 0;
int nbrOssilationsR = 0;
int previousValueR = 0;
float full360 = 2.35;

int weightsPreferingRight[8] = {-200,-20,-10,-10,10,160,180,500};

int weights[8] = {-280,-80,-12,-10,10,12,80,280};

int weights300[8] = {-700,-50,-20,-15,15,20,50,700};
int weights170[8] = {-280,-80,-12,-10,10,12,80,280};

bool flags[16] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
int flag2EncoderRCount, flag0EncoderRCount, flag4EncoderLCount, flag6EncoderRCount, flag7EncoderLCount, flag8EncoderLCount, flag10EncoderLCount, flag9EncoderLCount, flag12EncoderLCount;

int oldSums[8] = {0,0,0,0,0};
int threashholds[8] = {0,0,0,0,0,0,0,0};
float kp = 1;
float kd = 0.33;
float ki = 0;
float ks = 0.3;
bool blackOnWhite = true;
float tmp;
int baseRPM = 155;

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
  double targetSpeed = max(targetSpeedL , targetSpeedR);
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
  
  // PHASE 1
  delay(3000);
  setTargetL( distanceToTicks(617)/202.0 );
  setTargetR( distanceToTicks(617)/202.0 );
  goToTargets(200,200);
  turn(-90);
  setTargetL( distanceToTicks(265)/202.0 );
  setTargetR( distanceToTicks(265)/202.0 );
  goToTargets(150,150);
  turn(-90);
  setTargetL( distanceToTicks(250)/202.0 );
  setTargetR( distanceToTicks(250)/202.0 );
  goToTargets(150,150);
  turn(90);
  setTargetL( distanceToTicks(330)/202.0 );
  setTargetR( distanceToTicks(330)/202.0 );
  goToTargets(150,150);
  turn(-90);
  setTargetL( distanceToTicks(220)/202.0 );
  setTargetR( distanceToTicks(220)/202.0 );
  goToTargets(150,150);
  turn(50);
  setTargetL( distanceToTicks(250)/202.0 );
  setTargetR( distanceToTicks(250)/202.0 );
  goToTargets(200,200);
  
  flag0EncoderRCount = encoderRCount;
  /*
  weights[0] = -350;
  weights[1] = -150;
  weights[6] = 50;
  weights[7] = 100;
  flags[5] = true;
  flag4EncoderLCount = encoderLCount - 1315.0;
  flags[7] = true;
  flag7EncoderLCount = encoderLCount - 2000;
  for(int i=0;i<8;i++){
    weights[i] = weights300[i];
  }
  baseRPM = 300;
  flags[11] = true;
  */
}

void resetMotionState() {
  encoderRCount = 0;
  encoderLCount = 0;
  targetR = 0;
  targetL = 0;
  speedCorrectionR = 1.0;
  speedCorrectionL = 1.0;
  previousErrorL = 0;
  previousErrorR = 0;
  nbrOssilationsL = 0;
  nbrOssilationsR = 0;
  previousValueL = 0;
  previousValueR = 0;
  previousSpeedErrorL = 0;
  previousSpeedErrorR = 0;
  targetSpeedR = 100;
  targetSpeedL = 100;
}

void loop(){

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

  // PHASE 2
  if( !flags[0] && (getValue(1)||getValue(2))&&(getValue(5)||getValue(6)) && (encoderRCount - flag0EncoderRCount) > distanceToTicks(720.0) ){
    digitalWrite(2, HIGH);
  
    weights[0] = -500;
    weights[1] = -400;
    weights[2] = -100;
    weights[5] = 0;
    weights[6] = 20;
    weights[7] = 100;
    /*stop();
    resetMotionState();
    turn(45.0);*/

    flags[0] = true;
  }
  if ( flags[0] && !flags[1] && !getValue(0) && !getValue(1) && !getValue(2) && !getValue(3) && !getValue(4) && !getValue(5) && !getValue(6) && !getValue(7) ){
    digitalWrite(2, LOW);
    for(int i=0;i<8;i++){
      weights[i] = weights170[i];
    }
    flags[1] = true;
  }
  if ( flags[1] && !flags[2] && getValue(0) ){
    digitalWrite(2, HIGH);
    weights[0] = 0;
    weights[6] = 150;
    weights[7] = 400;
    flag2EncoderRCount = encoderRCount ;
    flags[2] = true;
  }
  if ( flags[2] && !flags[3] && (getValue(1) + getValue(2) +getValue(3) + getValue(4) + getValue(5) + getValue(6) >3 ) && (encoderRCount - flag2EncoderRCount) >distanceToTicks(200.0) ){
    digitalWrite(2, LOW);
    weights[0] = 0;
    weights[6] = 60;
    weights[7] = 0;
    blackOnWhite = false;

    flags[3] = true;
  }
  if ( flags[3] && !flags[4] && (getValue(1) + getValue(2) + getValue(3) + getValue(4) + getValue(5) + getValue(6) >3 ) ){
    digitalWrite(2, HIGH);
    blackOnWhite = true;

    flag4EncoderLCount = encoderLCount;
    flags[4] = true;
  }
  if ( flags[4] && !flags[5] && (encoderLCount - flag4EncoderLCount) > distanceToTicks(200.0) ){
    digitalWrite(2, LOW);
    weights[0] = -350;
    weights[1] = -150;
    weights[6] = 50;
    weights[7] = 100;

    flags[5] = true;
  }
  // PHASE 3
  
  if ( flags[5] && !flags[6] && (getValue(0) + getValue(1) + getValue(2) + getValue(3) + getValue(4) + getValue(5) + getValue(6) + getValue(7) ) == 0 && (encoderLCount - flag4EncoderLCount) > distanceToTicks(3000.0)){
    digitalWrite(2, HIGH);
    resetMotionState();
    turn(-150);
    //setTargetL(distanceToTicks(200.0) /202.0 );
    //setTargetR(distanceToTicks(200.0) /202.0);
    //goToTargets(150, 150);

    flag6EncoderRCount = encoderRCount;

    flags[6] = true;
  }
  if ( flags[6] && !flags[7] && (getValue(0) + getValue(1) + getValue(2) + getValue(3) + getValue(4) + getValue(5) + getValue(6) + getValue(7) ) == 0 && (encoderRCount - flag6EncoderRCount) > distanceToTicks(200.0)){
    digitalWrite(2, LOW);
    resetMotionState();
    turn(-90);
    flag7EncoderLCount = encoderLCount;

    flags[7] = true;
  }
  // PHASE 4
  
  if ( flags[7] && !flags[8] && (encoderLCount - flag7EncoderLCount) > distanceToTicks(2200.0)){
    digitalWrite(2, HIGH);
    weights[0] = -100;
    weights[1] = -60;
    weights[6] = 100;
    weights[7] = 300;
    flag8EncoderLCount = encoderLCount;

    flags[8] = true;
  }
  if ( flags[8] && !flags[9] && (encoderLCount - flag8EncoderLCount) > distanceToTicks(500.0)){
    digitalWrite(2, LOW);
    weights[0] = -300;
    weights[1] = -100;
    weights[6] = 60;
    weights[7] = 100;
    flag9EncoderLCount = encoderLCount;

    flags[9] = true;
  }
  if ( flags[9] && !flags[10] && (getValue(0) + getValue(1) + getValue(2) + getValue(3) + getValue(4) + getValue(5) + getValue(6) + getValue(7) ) == 0  && (encoderLCount - flag9EncoderLCount) > distanceToTicks(400.0)){
    digitalWrite(2, HIGH);
    resetMotionState();
    turn(180);
    flag10EncoderLCount = encoderLCount;

    flags[10] = true;
  }
  if ( flags[10] && !flags[11] && (encoderLCount - flag10EncoderLCount) > distanceToTicks(1100.0)){
    digitalWrite(2, LOW);
    for(int i=0;i<8;i++){
      weights[i] = weights300[i];
    }
    baseRPM = 300;
    flags[11] = true;
  }
  if ( flags[11] && !flags[12] && (getValue(2) + getValue(3) + getValue(4) + getValue(5) ) == 4){
    digitalWrite(2, HIGH);
    keepWalking(55.0);
    stop();
    resetMotionState();
    delay(200);
    turn(-35);
    setTargetL(distanceToTicks(40.0)/202.0);
    setTargetR(distanceToTicks(40.0)/202.0);
    goToTargets(150, 150);
    weights[6] = 25;
    weights[7] = 30;
    flag12EncoderLCount = encoderLCount;

    flags[12] = true;
  }
  if ( flags[12] && !flags[13] && (encoderLCount - flag12EncoderLCount) > distanceToTicks(560.0)){
    digitalWrite(2, LOW);
    weights[6] = 80;
    weights[7] = 280;
    weights[0] = -25;
    weights[1] = -30;

    flags[13] = true;
  }

  if ( flags[13] && !flags[14] && (encoderLCount - flag12EncoderLCount) > distanceToTicks(1160.0)){
    digitalWrite(2, HIGH);
    weights[6] = 80;
    weights[7] = 280;
    weights[0] = -280;
    weights[1] = -80;

    flags[14] = true;
  }

  if ( flags[14] && !flags[15] && (getValue(0) + getValue(1) + getValue(2) + getValue(3) + getValue(4) + getValue(5) + getValue(6) + getValue(7)) == 8){
    digitalWrite(2, LOW);
    keepWalking(35);
    stop();
    while(1){};
    flags[15] = true;
  }
}

