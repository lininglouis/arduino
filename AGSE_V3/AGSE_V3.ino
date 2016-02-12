
/*
This is the version 3 of the Arduino code for the UND USLI Rocket Team by Sofiane Chaieb

****************
* AGSE Version *
****************

The actions are:
0- Start Button
1- Payload Gripper Down
2- Claw Grips Payload
3- Payload Gripper UP
4- Payload Gripper Turns ClockWise
5- Claw Releases Payload
6- Payload Gripper Turns CounterClockWise
7- Payload Gripper Down
*/

//Include library for motor shield, servo and others
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "DualMC33926MotorShield.h"
#include <Servo.h> 
#include "leOS2.h" //include the scheduler
leOS2 myOS, myOSPause; //create a new istance

// Macros for the actions
#define OpenCloseClaw 0
#define MoveClawUp 2 
#define MoveClawDown 3 
#define PositionClaw 8
#define Pause 4
#define MoveRocketUp 5
#define MoveRocketDown 6
#define InsertIgniter 7
#define TEST 20
#define End 30
#define golight 91
#define Blink 99

// Actions
//unsigned int Action[40] = {PositionClaw, Pause, OpenCloseClaw, Pause, MoveClawDown, Pause, OpenCloseClaw, Pause, MoveClawUp, Pause, PositionClaw,  Pause, OpenCloseClaw, Pause, PositionClaw, Pause, OpenCloseClaw, Pause, MoveClawDown, Pause, MoveRocketUp, Pause, InsertIgniter, golight, Pause, End};
//unsigned int Valeur[40] = {85          , 10   , 150          , 10   , 0           , 10   , 0            , 10   , 0         , 10   , 35          ,  20   , 180          , 20   , 100         , 10   , 30           , 10   , 0           , 20   , 20          , 20   , 30           , 0      , 200  , 0  };
unsigned int Action[40] = {Pause, MoveRocketUp, Pause, InsertIgniter, golight, Pause, End};
unsigned int Valeur[40] = {20   , 20          , 20   , 30           , 0      , 500  , 0  };

/* Test Actions */
// Linear Actuator
//unsigned int Action[40] = {Pause, MoveRocketUp, Pause, MoveRocketDown, End};
//unsigned int Valeur[40] = {10   , 0           , 100   , 0             , 0};

//unsigned int Action[40] = {MoveRocketDown, End};
//unsigned int Valeur[40] = {0             , 0};

// Sequence initialisation
int step=0;

// Save actual value of servo
int clawServoActualAngle, rotateClawServoActualAngle;

// Pausing definition
const int switchInterruption =  19; // Pause switch that will trigger an interruption (interrupt 5) - pin used for waking up
// Boolean interruption
int PauseVar = false;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Setting of pin numbers for actuators
Servo clawServo; // Servo motor responsible for opening and closing the gripper/claw (PWM)
Servo rotateClawServo; // Servo motor responsible for rotating the gripper/claw (PWM)
DualMC33926MotorShield LinearMotorRocketLift; // Linear Actuator responsible for rising the rocket to 85° (PWM)
Adafruit_DCMotor *MotorClawSlider = AFMS.getMotor(2); // Gear motor responsible for moving up/down the gripper/claw (PWM)
Adafruit_DCMotor *MotorIgniter = AFMS.getMotor(3); // Gear motor responsible for inserting the igniter (PWM)

// Filtering
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 700;    // the debounce time; increase if the output flickers
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin


// Pins
const int switchTopClaw = 32; 
const int switchBottomClaw = 34; 
const int switchTopRocket = 36;
const int switchBottomRocket = 38; 
const int disableLinearActuator = 4;

// LED
int ledState = LOW, ledStateGO = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
int flashLed = 50, LEDGO = 51, interval = 1000;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("UND Frozen Team AGSE Start!");
  
  myOS.begin(); //initialize the scheduler
  //myOSPause.begin(); //initialize the scheduler
  myOS.addTask(blink, myOS.convertMs(1000));
  //myOSPause.addTask(PauseCode, myOSPause.convertMs(200));

  // Internal PullUp for switches
  pinMode(switchTopClaw, INPUT_PULLUP); 
  pinMode(switchBottomClaw, INPUT_PULLUP); 
  pinMode(switchTopRocket, INPUT_PULLUP); 
  pinMode(switchBottomRocket, INPUT_PULLUP); 
  pinMode(switchInterruption, INPUT_PULLUP); 
  pinMode(flashLed, OUTPUT);
  digitalWrite(flashLed, ledState);
  pinMode(LEDGO, OUTPUT);
  digitalWrite(LEDGO, ledStateGO);
  
  // Interruption
  //attachInterrupt(4, PauseCode(), CHANGE); // Pin 19
  //attachInterrupt(4, wakeUpNow, LOW); // Pin 19
  
  AFMS.begin();  // create with the default frequency 1.6KHz OR with a different frequency, say 1KHz: AFMS.begin(1000);
  
  // Assign LinearMotorRocketLift
  LinearMotorRocketLift.init();
  digitalWrite(disableLinearActuator, LOW);
  // Assign MotorClawSlider
  MotorClawSlider->setSpeed(180);
  MotorClawSlider->run(RELEASE);
  // Assign MotorIgniter
  MotorIgniter->setSpeed(255);
  MotorIgniter->run(RELEASE);
  // Assign Servo
  clawServo.attach(10);
  rotateClawServo.attach(9);  
  clawServoActualAngle = 180;
  rotateClawServoActualAngle = 85;
  clawServo.write(clawServoActualAngle); // Open Claw
  rotateClawServo.write(rotateClawServoActualAngle); // Position Claw

}

void loop(){
  
  // Initialising value of switches
  int switchValueTopClaw = digitalRead(switchTopClaw); 
  int switchValueBottomClaw = digitalRead(switchBottomClaw); 
  int switchValueTopRocket = digitalRead(switchTopRocket); 
  int switchValueBottomRocket = digitalRead(switchBottomRocket); 
  
  // Starting sequence
  for (; Action[step] != End; step++) {
    
      switch (Action[step]) {   
              case OpenCloseClaw :
                delay(200);
                while(!PauseCode());  
                if (Valeur[step] > clawServoActualAngle) {       
                for (int k=clawServoActualAngle; k<Valeur[step]; k++){
                  while(!PauseCode());  
                  clawServo.write(k);
                  delay(10);
                } 
                } else { 
                for (int k=clawServoActualAngle; k>Valeur[step]; k--){
                  while(!PauseCode());  
                  clawServo.write(k);
                  //Serial.println(k);
                  delay(10);
                }  
               } 
                delay(100);
                clawServoActualAngle = Valeur[step];
                while(!PauseCode());  
              break;
              case PositionClaw :
              while(!PauseCode());  
                if (Valeur[step] > rotateClawServoActualAngle) {       
                for (int k=rotateClawServoActualAngle; k<=Valeur[step]; k++){
                  while(!PauseCode());  
                  rotateClawServo.write(k);
                  delay(20);
                } 
                } else { 
                for (int k=rotateClawServoActualAngle; k>Valeur[step]; k--){
                  while(!PauseCode());  
                  rotateClawServo.write(k);
                  delay(20);
                }   
                }
                delay(100);
                rotateClawServoActualAngle = Valeur[step];
                while(!PauseCode());  
              break;
              case MoveClawUp :
                while(!PauseCode());
                switchValueTopClaw = digitalRead(switchTopClaw);                 
                while (switchValueTopClaw != LOW) {
                    while(!PauseCode()) {MotorClawSlider->run(RELEASE);}
                    //Serial.("switchTopClaw");                    
                    MotorClawSlider->run(FORWARD);
                    switchValueTopClaw = digitalRead(switchTopClaw); 
                }
                MotorClawSlider->run(RELEASE);  
                while(!PauseCode());
              break;
              case MoveClawDown :
                while(!PauseCode());
                switchValueBottomClaw = digitalRead(switchBottomClaw);
                while (switchValueBottomClaw != LOW) {
                    while(!PauseCode()) {MotorClawSlider->run(RELEASE);}
                    //Serial.println("switchBottomClaw");
                    MotorClawSlider->run(BACKWARD);  
                    switchValueBottomClaw = digitalRead(switchBottomClaw);
                }
                MotorClawSlider->run(RELEASE); 
                while(PauseVar);
              break;
              case MoveRocketUp :
                while(!PauseCode());           
                digitalWrite(disableLinearActuator, HIGH); 
                switchValueTopRocket = digitalRead(switchTopRocket);
                while (switchValueTopRocket != LOW) {
                    while(!PauseCode()) {LinearMotorRocketLift.setM1Speed(0);};                
                    LinearMotorRocketLift.setM1Speed(400);
                    //stopIfFault();
                    //Serial.print("LinearMotorRocketLift current: ");
                    //Serial.println(LinearMotorRocketLift.getM1CurrentMilliamps());
                    //Serial.println("switchTopRocket");
                    switchValueTopRocket = digitalRead(switchTopRocket);
                }
                LinearMotorRocketLift.setM1Speed(0); 
                digitalWrite(disableLinearActuator, LOW); 
                while(!PauseCode());  
              break;
              case MoveRocketDown :
                while(!PauseCode());
                digitalWrite(disableLinearActuator, HIGH); 
                switchValueBottomRocket = digitalRead(switchBottomRocket); 
                while (switchValueBottomRocket != LOW) {  
                    while(!PauseCode()) {LinearMotorRocketLift.setM1Speed(0);}
                    LinearMotorRocketLift.setM1Speed(-400); 
                    //stopIfFault();                    
                    //Serial.println("switchBottomRocket");   
                    switchValueBottomRocket = digitalRead(switchBottomRocket); 
                }
                LinearMotorRocketLift.setM1Speed(0); 
                digitalWrite(disableLinearActuator, LOW);
                while(!PauseCode());
              break;
              case InsertIgniter :
                while(!PauseCode());
                MotorIgniter->run(BACKWARD); 
                for (int k=0; k<=Valeur[step]*1000; k++){
                  while(!PauseCode()) {MotorIgniter->run(RELEASE);};
                  delay(1);
                }
                MotorIgniter->run(RELEASE); 
                while(!PauseCode());
              break;
              case Pause : // Delay en ms
                while(!PauseCode());
                 for (int j=0; j<=Valeur[step]; j++){
                  delay(100);
                }
              break;              
              case golight :
              digitalWrite(LEDGO, HIGH);
              break;
              /*
              default:
              unsigned long currentMillis = millis();
                if(currentMillis - previousMillis > interval) {
                  // save the last time you blinked the LED
                  previousMillis = currentMillis;  
              
                  // if the LED is off turn it on and vice-versa:
                  if (ledStateGO == HIGH || PauseVar == true)
                    ledStateGO = LOW;
                  else
                    ledStateGO = HIGH;
              
                  // set the LED with the ledState of the variable:
                  digitalWrite(LEDGO, ledStateGO);
                }
              break;
              */
      } 
  }
}

// Pololu shield fault detection
void stopIfFault()
{
  if (LinearMotorRocketLift.getFault())
  {
    Serial.println("fault");
    //while(1);
  }
}

int PauseCode() {
    // read the state of the switch into a local variable:
    int reading = digitalRead(switchInterruption);
    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH),  and you've waited
    // long enough since the last press to ignore any noise:  
    // If the switch changed, due to noise or pressing: 
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) < debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
    }
    if (buttonState == LOW) {
        PauseVar = true;
        Serial.println("Stopped");
      } else {
        PauseVar = false;
        Serial.println("continue"); 
        }
  } 
   lastButtonState = reading;
   return buttonState;
}  

void blink(){
  (!ledState || PauseVar) ? ledState=HIGH : ledState=LOW;
  digitalWrite(flashLed,ledState);
}
