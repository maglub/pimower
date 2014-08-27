#define __brushlessCutter__ // Brushed or brushless cutter motor
//#define __accelerometer__ // Accelerometer available
//#define __LCD__           // LCD Available
//#define __FWD__           // Front Wheel Drive
//#define __CutterCurrentSensing__  // Adjust speed through current sensing or voltage drop

#ifdef __brushlessCutter__
  #include <Servo.h>
  Servo cutter;
#endif

int debug     = 0;
int verbosity = 1;


const int motorLeft = A0;                     //  Current in motor
const int motorRight = A1;                    //  Current in motor
const int batterySOCPin = A2;       //  Analog reading of SOC (Battery Voltage)

const int cutterMotorOutPin = 6;

// Shield type. 
// 0 For Ardumoto, TinyOs and Hinduino motor shield
// 1 for Pololu motor Shield
const int shieldType = 0;

const int shieldPin[2][5] = {
                            {13,12,11,3,-1},
                            {7,8,9,10,4}
                            };

// Digital pin number designations for different shields                                                                 ArduMoto  Pololu
const int motorLeftDirection = shieldPin[shieldType][0];            //  13        7
const int motorRightDirection = shieldPin[shieldType][1];           //  12        8
const int motorLeftPWM = shieldPin[shieldType][2];                  //  11        9
const int motorRightPWM = shieldPin[shieldType][3];                //  3        10
const int enableShield = shieldPin[shieldType][4];                  //  -1        4

// Maximum allowed wheel motor current        Ardumoto  Pololu
const int maxWheelLoad = 40;                // 40     85
const int triggerWheelLoad = 6;            // 30     75


// Cutter states
const int CUTTER_ON = 1;
const int CUTTER_OFF = 0;
int cutter_state;

// Robot states
const int MOWING = 0;
const int LAUNCHING = 1;
const int TRACKING = 2;
const int DOCKING = 3;
const int CHARGING = 4;
const int DEBUG = 5;
const int CONFIGURE = 6;
const int STARTUP = 7;
const int IDLE = 8;
const int STOP = 9;
const int LEFT = 10;
const int RIGHT = 11;
const int BUMP = 12;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

const int fullSpeed = 255;
const int reducedSpeed = 150;
const int slowSpeed = 100;

int cutterSpeed = 150;

int state;
int counter = 0;


//=============================================
// stopCutter()
//=============================================
void stopCutter() {
    cutter.write(90);
    cutter_state = CUTTER_OFF;
}

//=============================================
// startCutter()
//=============================================
void startCutter() {
    Serial.println("Starting cutter...");
    for (int i=70; i< cutterSpeed; i+=2)
    {
      cutter.write(i);
      delay(50);
    } 
    cutter_state = CUTTER_ON;
}


int getCutterState(){
  return cutter_state;
}

void toggleCutter(){
  Serial.println(getCutterState());

  int retVal = cutter_state ^ 0x01;
  Serial.print("  - cutter_state: ");
  Serial.println(cutter_state);

  cutter_state = retVal;

  Serial.print("  - cutter_state: ");
  Serial.println(cutter_state);
  
}

void toggleDebug(){
  debug = debug ^ 0x01; 
}

//=============================================
//=============================================
// Motor driver routines
//=============================================
//=============================================

int getBatterySOC() {
  return analogRead(batterySOCPin);
}

unsigned int getLeftMotorCurrent() {
  return analogRead(motorLeft);
}
  
unsigned int getRightMotorCurrent() {
  return analogRead(motorRight);
}

int getLeftMotorSpeed() {
  return(leftMotorSpeed);
}

int getRightMotorSpeed() {
  return(rightMotorSpeed);
}


void setLeftMotorDirection(boolean fwd){
   digitalWrite(motorLeftDirection, fwd); 
}

void setRightMotorDirection(boolean fwd){
   digitalWrite(motorRightDirection, fwd); 
}


void setLeftMotorSpeed(int val){
  leftMotorSpeed = val;
  analogWrite(motorLeftPWM, abs(val));
  setLeftMotorDirection(val>0);
}

void setRightMotorSpeed(int val){
  rightMotorSpeed = val;
  analogWrite(motorRightPWM, abs(val));
  setRightMotorDirection(val>0);
}

void stopMower(){
      setLeftMotorSpeed(0);
      setRightMotorSpeed(0);  
      delay(200);
}

//=============================================
// slowStart()
//=============================================
void slowStart(){
      Serial.println ("  - Slow start");
      Serial.print ("    - ");
       //slow start
      for (int i = 0; i<fullSpeed; i+=5) {
        setLeftMotorSpeed(i);
        setRightMotorSpeed(i);
        delay(15);
      }
      setLeftMotorSpeed(fullSpeed);
      setRightMotorSpeed(fullSpeed);
}

void reverseDirection(){

  Serial.println("* reverseDirection()");
  
  int ls = getLeftMotorSpeed();
  int rs = getLeftMotorSpeed();
  int cur_fullSpeed;
  
  int new_leftDirection = ls > 0 ? -1 : 1;
  int new_rightDirection = rs > 0 ? -1: 1;

  Serial.print("  - new_leftDirection: ");
  Serial.print(new_leftDirection);
  Serial.print(" new_rightDirection: ");
  Serial.print(new_rightDirection);
  Serial.print(" leftMotorSpeed: ");
  Serial.print(ls);
  Serial.print(" rightMotorSpeed: ");
  Serial.print(rs);
  Serial.println();

  if (ls > rs ) {
    cur_fullSpeed = abs(ls);
  } else {
    cur_fullSpeed = abs(rs);
  }
  
  for (int i = 0; i<fullSpeed; i+=5) {
    setLeftMotorSpeed(  i * new_leftDirection  );
    setRightMotorSpeed( i * new_rightDirection );

    Serial.print("  - leftMotorSpeed: ");
    Serial.print(i * new_leftDirection);
    Serial.print(" rightMotorSpeed: ");
    Serial.print(i * new_rightDirection);
    Serial.println();

    delay(15);
  }

      setLeftMotorSpeed(cur_fullSpeed * new_leftDirection );
      setRightMotorSpeed(cur_fullSpeed * new_rightDirection);
    
}

//=============================================
// backUp()
//=============================================
void backUp(int backUpTime){
  Serial.print("  - Backing up ");
  Serial.print(backUpTime);
  Serial.print(" seconds.");
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  delay(1000);
  
  setLeftMotorSpeed(-fullSpeed);
  setRightMotorSpeed(-fullSpeed);
  delay(backUpTime);
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  Serial.println("  - Done backing up");
  delay(1000);
}

//=============================================
// backUpWithTwist()
//=============================================
void backUpWithTwist(int backUpTime){
  Serial.print("  - Backing up with twist (one wheel reduced speed) ");
  Serial.print(backUpTime);
  Serial.print(" seconds.");
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
//  delay(200);
//  delay(200);
  
  setLeftMotorSpeed(-fullSpeed);
//  setRightMotorSpeed(-slowSpeed);
  setRightMotorSpeed(-fullSpeed);
  delay(backUpTime / 2);
  setRightMotorSpeed(fullSpeed);
  delay(backUpTime / 2);
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  Serial.println("  - Done backing up");
  delay(100);
}

//===============================================
// stopIfFault
//===============================================

void stopIfFault(int es)
{
  Serial.println("fault");

  stopMower();
  stopCutter();

  Serial.print("Error number: ");
  Serial.println(es);

  while(1) {
    delay(100);
  }
}


//=============================================
//=============================================
// Debug info
//=============================================
//=============================================

void printState(){

  switch(state){
    case MOWING: Serial.print("MOWING") ; break ;
    case LAUNCHING: Serial.print("LAUNCHING") ; break ;
    case TRACKING: Serial.print("TRACKING") ; break ;
    case DOCKING: Serial.print("DOCKING") ; break ;
    case CHARGING: Serial.print("CHARGING") ; break ;
    case DEBUG: Serial.print("DEBUG") ; break ;
    case CONFIGURE: Serial.print("CONFIGURE") ; break ;
    case STARTUP: Serial.print("STARTUP") ; break ;
    case IDLE: Serial.print("IDLE") ; break ;
    case STOP: Serial.print("STOP") ; break ;
    case LEFT: Serial.print("LEFT") ; break ;
    case RIGHT: Serial.print("RIGHT") ; break ;    
  }  
}


void printStatus()
{
  
  if ( verbosity == 1 ) {
    Serial.print ("Counter: ");
    Serial.print (counter);
    Serial.print (" LC: ");
    Serial.print (getLeftMotorCurrent());
    Serial.print (" RC: ");
    Serial.print (getRightMotorCurrent());
    Serial.print (" LeftMotorSpeed: ");
    Serial.print (getLeftMotorSpeed());
    Serial.print (" RightMotorSpeed: ");
    Serial.print (getRightMotorSpeed());
    Serial.print (" Cutter state: ");
    Serial.print (getCutterState());

   //#--- R = R1 + R2 = 330k + 100k = 430k
   //#--- U = R * I
   //#--- U1 = 330k * I
   //#--- U2 = 100k * I
   //#--- Vin = U1 + U2
   //#--- Vin = 330k * I + U2
   //#--- I = Vin / (330k + 100k)
   //#--- Vin = 330k * Vin / (430k) + U2
   //#--- U2 = 100/430 * Vin
   //#--- Vin = 430/100 * U2
   //#--- Analogue A2 -> Voltage (U2) = 5/1024 * A2
   //#--- Vin = 4.3 * U2
   //#--- Vin = 4.3 * 5 / 1024 * A2 = 21.5 * getBatterySOC() / 1024
   
    Serial.print (" Battery: ");
    Serial.print (21.5 * getBatterySOC() / 1024, 2);
  
    Serial.print(" State: "); printState();  
    Serial.print(" Debug: ");
    Serial.print(debug);
  
    Serial.println();  
  }

}

void printDebug(){
   /*
      Serial.print("LSens: ");
      Serial.print(getLeftSensor());
      Serial.print(" RSens: ");
      Serial.print(getRightSensor());
  */
      Serial.print ("Counter: ");
      Serial.print (counter);
      Serial.print(" LMot: ");
      Serial.print(getLeftMotorCurrent());
      Serial.print(" RMot: ");
      Serial.print(getRightMotorCurrent());
  /*
      Serial.print(" SOC: ");
      Serial.print(getBatterySOC());
      Serial.print(" Docked: ");
      Serial.print(getdockingPin());
      Serial.print(" MustCharge: ");
      Serial.print(mustCharge());
      
      Serial.print(" AccX: ");
      Serial.print(getXacc());
      Serial.print(" AccY: ");
      Serial.print(getYacc());
      Serial.print(" AccZ: ");
      Serial.print(getZacc());
  
      Serial.print(" AnglXZ: "); 
      Serial.print(getXZang());
      Serial.print(" AnglYZ: ");
      Serial.print(getYZang());
  */
    
      Serial.println();
 
}

//=============================================
// Startup
//=============================================

void StartUp(){
  
      Serial.println ();
      Serial.println ("============================================");
      Serial.println ("* Starting up");
      Serial.println ("============================================");
      
      Serial.println ("  - Stopping engines");
      Serial.print ("    - ");
      printStatus();
      
      setLeftMotorSpeed(0);
      setRightMotorSpeed(0);
      delay(1000);
      // Serial.println ("  - Backing up");
      // Serial.print ("    - ");
      // printStatus();
      
      // setLeftMotorSpeed(-fullSpeed);
      // setRightMotorSpeed(-fullSpeed);
      // delay(1000);
      // slowStart();      

      // setLeftMotorSpeed(fullSpeed);
      // setRightMotorSpeed(fullSpeed);
}

//=============================================
// Setup()
//=============================================
void setup()
{
  
  state = STARTUP;

  Serial.begin(115200); 

  pinMode(cutterMotorOutPin, OUTPUT);

  pinMode(motorLeftDirection, OUTPUT);
  pinMode(motorRightDirection, OUTPUT);
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);

  //Initialize cutter motor

  cutter.attach(cutterMotorOutPin);
  
  // One time arming and calibration of the motor controller
  // One time arming and calibration of the motor controller
  if (state == CONFIGURE) {
      cutter.write(180);
      Serial.println("Configuring: Full throtle + delay");
      delay(3000);
      Serial.println("Configuring: 0");
      cutter.write(0);
      delay(3000);
      state = DEBUG;
  }

  Serial.println("Setting cutter_state=0");
  cutter_state = 0;
  
  Serial.println("Configuring: 3 seconds low");

  // 3 sekunder - låg
  for (int i=0; i<3000; i++) {
    cutter.write(90);
    delay(1);
  }

  if (cutterSpeed > 150) {
      cutterSpeed = 150;
  }
  
  stopCutter();
  //startCutter();


  //Initialize wheels
  analogWrite(motorLeftPWM, 200);        
  //set both motors to run at (100/255 = 39)% duty cycle (slow)  
  analogWrite(motorRightPWM, 200);

  digitalWrite(motorLeftDirection, LOW);  //Set motor direction, 1 low, 2 high
  digitalWrite(motorRightDirection, LOW);  //Set motor direction, 3 high, 4 low

}

//===============================================
//===============================================
// MAIN
//===============================================
//===============================================

void loop()
{
  
  switch (state) {
    case MOWING:

      // Hit something
      if ( debug == 0 && (getLeftMotorCurrent() > triggerWheelLoad || getRightMotorCurrent() > triggerWheelLoad)) {
        state = BUMP;
      }   
      break; // end state MOWING
  
    case BUMP:
        stopCutter();
        Serial.println("* Motor current threshold reached");
        backUpWithTwist(1500);
        slowStart();
        //startCutter();
        state = MOWING;
        break;
        
    case STARTUP:
      StartUp();
      state = IDLE;  
      break;
 
    case DEBUG:
      int Q;
      //stopCutter();
      //startCutter();
      //stopMower();
      for (int i=0; i<10; i++) {
        printDebug();
        delay(500);
      }
      break; // End of DEBUG

    case LEFT:
      int oldLeft;
      oldLeft = getLeftMotorSpeed();
      setLeftMotorSpeed(0);
      delay(1000);
      setLeftMotorSpeed(oldLeft);
      state = MOWING;
      break;
      
    case RIGHT:
      int oldRight;
      oldRight = getRightMotorSpeed();
      setRightMotorSpeed(0);
      delay(1000);
      setRightMotorSpeed(oldRight);
      state = MOWING;
      break;
      
    case STOP:
      stopMower();
      state = IDLE;
      break;

    case IDLE:
      break;
  }

  // commands from the serial interface
  if (Serial.available())  {
     char inputChar = Serial.read();
     Serial.println("Caught serial"); // anything from serial line
     Serial.print("  - Received: ");
     Serial.println(inputChar);
    
     if (inputChar == 's' && state != MOWING ) { Serial.println("  - Starting robot"); state = MOWING; slowStart(); inputChar=0 ;}
     if (inputChar == 'd' && state == MOWING ) { Serial.println("  - Reversing robot") ; reverseDirection(); inputChar=0 ;}     
     if (inputChar == 's' && state == MOWING ) { Serial.println("  - Stopping robot"); state = STOP; inputChar=0;}
     if (inputChar == 'l' && state == MOWING ) { Serial.println("  - Turning left"); state = LEFT; inputChar=0;}
     if (inputChar == 'r' && state == MOWING ) { Serial.println("  - Turning right"); state = RIGHT; inputChar=0;}
     if (inputChar == 'c' && state == MOWING ) { Serial.println("  - Toggle cutter"); toggleCutter(); state = MOWING; inputChar=0;}
     if (inputChar == 'D' ) { Serial.println("  - Toggling debug flag"); toggleDebug(); inputChar=0; inputChar = 0; }
     if (inputChar == 'p' ) { Serial.println("  - Toggling print status output"); verbosity = verbosity ^ 0x01; }
     if (inputChar == 'b' && state == MOWING ) { Serial.println("  - Bump"); state = BUMP; inputChar=0;}
  }

  printStatus();
  counter++;
  delay(100);
 
}
