#define PI 3.14159265358979323846

//ESP32 pin Layout
int sensors[8] = {35, 27, 26, 25, 14, 13, 12, 15}; // sensor's pins from left to right
int pushButton = 34; // pushbutton pin
int motorRA = 21; // right motor +
int motorRB = 19; // right motor -
int motorLA = 22; // left motor +
int motorLB = 23; // left motor -
int encoderRA = 33; // right encoder A phase
int encoderRB = 18; // right encoder B phase
int encoderLA = 5; // left encoder A phase
int encoderLB = 4; // left encpder B phase

volatile int encoderRCount = 0; // tick count of right encoder
volatile int previousEncoderRCount = 0; // previous tick count of right encoder
volatile double speedR = 0; // current speed of right motor
int currentVoltageR = 100; // current voltage applied to right motor (between -255 and 255)
double previousSpeedErrorR = 0; // previous speed error, needed for speed regulation PID
double newErrorR; // current speed error, needed for speed regulation PID
volatile unsigned long previousMeasureTimeR = 0; // time in microseconds associated to the last right encoder tick, needed to calculate speed
int targetR; // target encoder count of right motor
double targetSpeedR = 100; // target speed of right motor
double previousTargetSpeedR = 100; // previous target speed of right motor, needed to detect sudden jumps
float speedCorrectionR = 1.0; // correction factor by which the speed regulation PID corrects the position regulation PID

// same functionality as their Right equivilants
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

float kpE = 1; // proportional weight of position control PID
float kiE = 2.5; // integral weight of position control PID
float kdE = 3.0; // derivative weight of position control PID
float ksE = 0.1; // all in one weight of position control PID (s stands for speed)
int previousErrorR = 0; // previous position error, needed for position control PID
int previousErrorL = 0;

float kpS = 0.3;  // proportional weight of speed control PID
float kiS = 0.02; // integral weight of speed control PID
float kdS = 0.05; // derivative weight of speed control PID
float ksS = 0.13; // all in one weight of speed control PID

int maxSpeed = 170; // max PWM signal that the position control PID can apply
int minSpeed = 100; // min PWM signal that the position control PID can apply

int rightCorrection, leftCorrection; // dummy variable, stores correction term for position control PID
int nbrOssilationsL = 0; // number of ossilations done after going to a certain position
int previousValueL = 0; // previous PID value concerning position control
int nbrOssilationsR = 0;
int previousValueR = 0;
float full360 = 2.35; // number of rotations needed for each wheel to make a 360

int weights[8] = {-280,-80,-12,-10,10,12,80,280}; // current weights of sensors

int weights300[8] = {-700,-50,-20,-15,15,20,50,700}; // weights tuned for 300 RPM speed
int weights170[8] = {-280,-80,-12,-10,10,12,80,280}; // weights tuned for 170 RPM speed

bool flags[16] = { false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}; // array to store states of flags
int flag2EncoderRCount, flag0EncoderRCount, flag4EncoderLCount, flag6EncoderRCount, flag7EncoderLCount, flag8EncoderLCount, flag10EncoderLCount, flag9EncoderLCount, flag12EncoderLCount; // sometimes the position at which a flag was triggered is needed

int oldSums[8] = {0,0,0,0,0}; // old PID sums concerning sensor PIDing
int threashholds[8] = {0,0,0,0,0,0,0,0}; // array of threashholds for each sensor
float kp = 1; // proportional weight of sensor PIDing
float kd = 0.33; // derivative weight of sensor PIDing
float ki = 0; // integral weight of sensor PIDing
float ks = 0.3; // all in one weight of sensor PIDing
bool blackOnWhite = true; // is the robot currently following black lines on white background or not
float tmp; // just a dummy variable
int baseRPM = 155; // base RPM target for both motors

float getPIDValue(){ // returns correction needed to go back on line if off it
  int sum = 0; 
  for(int i=0; i<8; i++){
    sum += getValue(i) * weights[i] * 4; // summing current readings of all sensors multiplied by its corresponding weight, more positive means we need to go right and on like that
  }
  delayMicroseconds(10); // small delay tuned for stability

  float value =    kp*sum   +    kd*(sum - oldSums[4])   +  ki*(sum + oldSums[4]); // applying the PID formula
  // pushing back history of sums
  for(int i=1;i<4; i++){
    oldSums[i] = oldSums[i-1]; 
  }
  oldSums[0] = sum;
  return ks*value; // augmenting the effect with the ks factor
}

int getValue(int sensor){ // returns 1 if the given sensor is sensing black and 0 if not, gets the sensor's index as parameter
  if(blackOnWhite){
    return( analogRead(sensors[sensor]) > threashholds[sensor]);
  }else{
    return( analogRead(sensors[sensor]) < threashholds[sensor]);
  }
}

int getEncoderCorrectionR(){ // applying PID algorithm on position
  int error = targetR - encoderRCount; // distance which the wheel still needs to go, measured in ticks
  int value = (int)( ksE*( kpE*error + kdE*(error - previousErrorR) + kiE*(error + previousErrorR)) ); // PID formula
  // incrementing the number of oscillations if there was one
  if( (value > 0 && previousValueR <= 0) || (value < 0 && previousValueR >= 0)){
    nbrOssilationsR++;
    if (nbrOssilationsR>6){
      return (0);
    }
  }
  previousValueR = value;
  previousErrorR = error;
  if( abs(encoderRCount-targetR)<10 && abs(error - previousErrorR) < 2){ // detecting if the error is negligeable enough to return 0 instead
    return(0);
  }
  // constraining the returned value between the max and min
  if (value >= 0){
    return( max( minSpeed, min( maxSpeed, value ) ) );
  }
  return( min( (-1)*minSpeed, max( (-1)*maxSpeed, value ) ) );
}

int getEncoderCorrectionL(){ // all same as its right equivilant
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

void IRAM_ATTR encoderRISR() { // increments or dectrements encoder count based on the state of A and B phases
  if(digitalRead(encoderRB) == digitalRead(encoderRA)){ // plot A and B to further see why it works
    encoderRCount--;
  }else{
    encoderRCount++;
  }
  if(encoderRCount%2){ // recalculating speed in RPM once every 2 ticks (there are 202 ticks per wheel turn)
    unsigned long tmp = micros();
    speedR = ( ( (double)(encoderRCount - previousEncoderRCount)*(60000000/202) ) / (tmp - previousMeasureTimeR) );
    previousEncoderRCount = encoderRCount;
    previousMeasureTimeR = tmp;
  }
}

void IRAM_ATTR encoderLISR() { // same for its right equivilant
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
  setTargetR(full360*3); // planning on making 3 360s to calibrate
  setTargetL(-full360*3);
  int min[8] = {4000,4000,4000,4000,4000,4000,4000,4000}; // max value that can be returned by the sensors is 4096 to the ESP, so 4000 will serve for finding the minimal value
  int reading; // dummy variable to store the current sensor's reading
  rightCorrection = getEncoderCorrectionR();
  leftCorrection = getEncoderCorrectionL();

  while(leftCorrection != 0 || rightCorrection != 0){ // corrections will be 0 if the target is reached
    for(int i=0;i<8;i++){
      // finding the max and min value for each sensor, max will be the value on black and min will be the value on white
      reading = analogRead(sensors[i]);
      if(min[i] > reading ){
        min[i] = reading;
      }
      if(threashholds[i] < reading ){ //temporarely using the threashholds array as holder for the maxes
        threashholds[i] = reading;
      }
    }

    // correcting PWM signal of each motor to reach target smouthly
    speedRight(rightCorrection);
    speedLeft(leftCorrection);
    delayMicroseconds(1000); // small delay for stability
    // getting ready for the next iteration
    rightCorrection = getEncoderCorrectionR();
    leftCorrection = getEncoderCorrectionL();
  }
  stop();
  for(int i=0;i<8;i++){
    threashholds[i] = (threashholds[i] + min[i])/2 ; // calculating threashholds as the mean of max and min
  }
}

// applies given PWM signal to concerned motor
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

// sets the position target of left motor to some number of ticks given the number of rotations needed
void setTargetL(float nbrOfRotations){
  targetL += (int)(nbrOfRotations*202);
  // reinitializing some variables needed for position control
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

void turn(float degree){ // same code as the calibration function, but with no calibration
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

void goToTargets(double targetSpeedRight, double targetSpeedLeft){ // after setting tick count targets for both motors this function will make them reach those targets keeping a specific speed for each
  rightCorrection = getEncoderCorrectionR();
  leftCorrection = getEncoderCorrectionL();
  speedCorrectionR = 1;
  speedCorrectionL = 1;

  int i = 0; // counter to take care of ramping up to speed
  // setting the ramp length base on the max of given speeds (1200 cycles is the maximal ramp length)
  // it has to be proportional to the target distance and inversly proportional to speed, 60000 is just a tuning factor (ramplength is measured in milliseconds and it represents the duration of ramping, probably should've named it rampDuration)
  int rampLength = min(1200, (int)(max( ( ( (float)abs(targetR-encoderRCount)/202 )/targetSpeedRight )*60000, ( ( (float)abs(targetL-encoderLCount)/202 )/targetSpeedLeft )*60000 ) /1.0) );
  // steps to by which the speed will gradually increase (there is 40 in the formula because that's the lowest RPM the used motor can put out)
  float stepR = (float)(targetSpeedRight-40)/rampLength;
  float stepL = (float)(targetSpeedLeft-40)/rampLength;
  // initializing the starting speed for each motor
  targetSpeedR = 40 - stepR*10;
  targetSpeedL = 40 - stepL*10;
  
  bool breaking = false; // is the robot breaking right now or not
  double timeLeft; // calculates an estimate of the time left to complete this "goToTargets" call

  while(leftCorrection != 0 || rightCorrection != 0){ // while the postion is not yet correct 
    //ramping
    if (i <= rampLength){ // while still ramping
      if (i%10 == 0){ // increment target speeds every 10 cycles to give time for the motors to reach them
        targetSpeedR += stepR*10;
        targetSpeedL += stepL*10;
      }
      i++;
      // finally setting speeds to the correct targets to eliminate rounding errors that can happen due to just incrementing speed by a float step
      if (i>rampLength && abs(targetSpeedLeft - targetSpeedL) < 12 && abs(targetSpeedRight - targetSpeedR) < 12){
        targetSpeedR = targetSpeedRight;
        targetSpeedL = targetSpeedLeft;
      }
    }
    
    //breaking
    if (breaking == false){
      // calculating an estimate of timeLeft based on target speed and distance left
      timeLeft = max( ( ( (float)abs(targetR-encoderRCount)/202 )/targetSpeedRight )*60000, ( ( (float)abs(targetL-encoderLCount)/202 )/targetSpeedLeft )*60000 );
      if(timeLeft < rampLength*( (float)max(targetSpeedLeft, targetSpeedRight)/1000 ) ){
        breaking = true;
        // amplifying the steps but inversely proportional to the speeds because if the robot moves at a faster speed he has more momentum and thus should need longer to break so smaller steps
        stepR = stepR/( (float)max(targetSpeedLeft, targetSpeedRight)/1000 );
        stepL = stepL/( (float)max(targetSpeedLeft, targetSpeedRight)/1000 );
      }
    }else{
      targetSpeedR = max( 85.0, targetSpeedR - stepR); // while breaking the motor better not reach less than about 85.0 RPM
      targetSpeedL = max( 85.0, targetSpeedL - stepL);
    }

    //speed control
    if (abs(encoderRCount - targetR) > 20){ // does not correct speed when the robot is near its target, so not to worsen position control
      speedCorrectionR = max ( 0.2 , speedCorrectionR + (targetSpeedR - abs(speedR))*0.0004 ); // correcting for Speed, 0.0004 is just Kp in this case, Ki and Kd are 0
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
  }
  stop();
}

int distanceToTicks(float distance){
  return (int)( ( distance / (66.0*PI) )*202 ); // 66.0 is the wheel diameter in millimiters, so this formula returns the number of ticks for a given distance
}

void keepWalking(float distance){
  int targetEncoderRight = encoderRCount + distanceToTicks(distance); // controlling position only on the right motor cause the robot will walk in a straight line anyways
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
  // for some reason the motor driver does not want to stop the motors sometimes when given 0 PWM, but this fixes it
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
  // each time a change happens in phase A of the encoder, the function encoderRISR gets triggered
  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISR, CHANGE);
  
  encoderRCount = 0;
  encoderLCount = 0;
  digitalWrite(2, HIGH); // onboard LED
  calibrate();
  delay(2000);
  digitalWrite(2, LOW);
  
  // PHASE 1 (consists of orthogonal segments, the robot will be line-followin blind here, just following instructions)
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

void resetMotionState() { // reinitialisation of all variables
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

  //speed control
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
  // PHASE 5
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