#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int initpos =55, pos;    // variable to store the servo position 

int switch1 = 11, switch2 = 12, LED = 13, interval = 100;
boolean alreadyDone = false;

int ledState = HIGH;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

void setup() 
{ 
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  
  for(pos =110 ; pos>=initpos; pos-=1)     // goes from 110 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(20);                       // waits 15ms for the servo to reach the position 
  }

  // Internal PullUp for switches
  pinMode(switch1, INPUT_PULLUP); 
  pinMode(switch2, INPUT_PULLUP);
  pinMode(LED, OUTPUT); 
} 
 
 
void loop() 
{
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;  

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    //Serial.print(ledState);

    // set the LED with the ledState of the variable:
    digitalWrite(LED, ledState);
  }  
  
  int switch1Triggered = digitalRead(switch1); 
  int switch2Triggered = digitalRead(switch2); 

  if ((switch1Triggered == LOW || switch2Triggered == LOW) && !alreadyDone) {    
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(2000);
    alreadyDone = true;
    for(pos = initpos; pos < 110; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(20);                       // waits 15ms for the servo to reach the position 
  }
}
} 

