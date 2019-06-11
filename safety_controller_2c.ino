#include <ServoDecode.h>

char * stateStrings[] = {"NOT_SYNCHED", "ACQUIRING", "READY", "in Failsafe"};
  
// Define readable non-backwards states for relays.
// This is because relays are active LOW.  That means you can ground the pin and activate the relay.
const byte relayOn = 0;
const byte relayOff = 1;

// Give each relay friendly name and map to Arduino Digital I/O pin number
const byte relay1 = 2;  // wire 1 of the engine kill actuator.  Pulse to allow engine to run.
const byte relay2 = 3;  // wire 2 of the engine kill actuator.  Pulse to kill engine.
const byte relay3 = 4;  // brake controller
const byte relay4 = 5;  // currently unused

// For reading/feedback purposes, compare to the on off constants above
byte relay1State;
byte relay2State;
byte relay3State;
byte relay4State;
                                    // seconds * milliseconds, for readibility
const unsigned long idleEngineKill = 10 * 1000; // delay to kill the motor 
const unsigned long idleBrakesEngage = 2 * 1000;
const unsigned long engineStartPeriod = 15 * 1000; // length of time allowed on a new connection to allow for starting the mower engine
const unsigned long engineActuatorPulse = .5 * 1000; // length of time to pulse the actuator
long whatTimeIsIt = 0;
long userInputTimer = 0;
long engineKillPulseTimer = 0; // timer used to engine kill actuator
long engineAllowPulseTimer = 0;
boolean timedOut = false;

// Initialize to zero to give it an obvious out of range starting value
// valid values will be from 1000 - 2000 with maybe some spillage in either direction

int curVal[7] = {0, 0, 0, 0, 0, 0, 0}; // Current values of the channel signals
int prevVal[7] = {0, 0, 0, 0, 0, 0, 0}; // Previous values of the channel signals
int arraySize = 7;
int numChannels = 6; // size of both arrays, used for feedback purposes

const int chChangeThreshold = 15; // Used to determine if there was user input on a given channel
boolean firstTimeOn = true;
boolean firstTimeOff = true;
boolean firstInput = true;
boolean engineShouldBeOff = true;
boolean brakesShouldBeOff = false;
boolean engineKillMidPoint = false;
boolean engineAllowMidPoint = false;
byte throttleChannel = 3; // sets the throttle channel to remove it from idle calculations.  Set to 0 to include all channels in idle calculations.  This is selectable as the transmitter is programmable.

char start = 's';
char finish = 'f';

void setup(){
  
  //ServoDecode setup
  ServoDecode.begin();
  ServoDecode.setFailsafe(3,1234); // set channel 3 failsafe pulse width
  
  // Relay Setup Initialize Pins so relays are inactive at reset
  digitalWrite(relay1, relayOff);
  digitalWrite(relay2, relayOff);
  digitalWrite(relay3, relayOff);
  digitalWrite(relay4, relayOff);
  relay1State = relayOff;
  relay2State = relayOff;
  relay3State = relayOff;
  relay4State = relayOff;
  
  // THEN set pins as outputs
  pinMode(relay1, OUTPUT);   
  pinMode(relay2, OUTPUT);  
  pinMode(relay3, OUTPUT);  
  pinMode(relay4, OUTPUT);    
  delay(1000); // Check that all relays are inactive at Reset
  
  // Prevent motor from starting and engage breaks
  engineKill(start);
  brakesEngage();

} // end setup()


void loop(){

  if ( (millis() >= engineKillPulseTimer + engineActuatorPulse) && engineKillMidPoint){
    engineKill(finish);
  } // end if 
  
  if ( (millis() >= engineAllowPulseTimer + engineActuatorPulse) && engineAllowMidPoint){
    engineAllow(finish);
  } // end if

  if(ServoDecode.getState()!= READY_state) { // transmitter is off or out of range
    if(firstTimeOff){
      engineKill(start);
      brakesEngage(); 
      whatTimeIsIt = 0; 
      timedOut = false;
      firstTimeOff = false;
      firstTimeOn = true;
    } // end if(firsTimeOff)
  } // end if() transmitter not ready
  
  if(ServoDecode.getState()== READY_state) { // transmitter is on
    
    if (firstTimeOn){
      firstTimeOn = false;
      firstTimeOff = true;
      engineAllow(start);
      brakesEngage();
      timedOut = false;
    } //enf if() firstTimeOn
    
    if(!userInput() && !timedOut){
      if(firstInput){// test if first entry into this path
        userInputTimer = millis(); // engage a timer
        firstInput = false;
      } //end firstInput
      
      if ( (millis() >= (userInputTimer + idleBrakesEngage)) && !firstTimeOn ){ // if idleBrakesEngage seconds has passed since no user input engage the brakes
        brakesEngage();
        // userInputTimer = 0;
      } // end  
      
      if ( (millis() >= (userInputTimer + idleEngineKill)) && !firstTimeOn ){ // if idleEngineKill seconds has passed since no user input, kill the motor and engage the brakes
        engineKill(start);
        brakesEngage();
        userInputTimer = 0;
        firstInput = true;
        timedOut = true;
      } // end 
      
    } // end if no input
    
    if(userInput()){
      userInputTimer = 0;
      firstInput = true;
      brakesDisengage();
      timedOut = false;
    } // end if() userInput()
    
  } // end if() transmitter ready
  
} // end main loop()

void engineKill(){
  
  if(relay1State == relayOn){ // just in case it was on for some random reason
    digitalWrite(relay1, relayOff); // prevents frying relays and/or actuators
    relay1State = relayOff;
  } // end safety check
  
  if(relay2State == relayOff){ // if it's already on, no need to do anything, so only do something if its off
    relay2State = relayOn;
    pulseOutLow(relay2, 500);
    relay2State = relayOff;
  } // end if()
  engineShouldBeOff = true;
  //digitalWrite(relay2, relayOn); // eventually pulse it, but for now just latch it
} // end engineKill

void engineKill(char state){
  switch (state){
    
    case 's': //rising edge of pulsing the actuator
      engineKillPulseTimer = millis();
      if(relay1State == relayOn){ // just in case it was on for some random reason, turn it off
        digitalWrite(relay1, relayOff); // prevents frying relays and/or actuators
        relay1State = relayOff;
      } // end safety check
      if(relay2State == relayOff){ // if it's already on, no need to do anything, so only do something if its off
        relay2State = relayOn;
        digitalWrite(relay2, relayOn);
        } // end if()
      engineKillMidPoint = true;
    break; // end case start
    
    case 'f': // falling edge of pulsing the actuator
      digitalWrite(relay2, relayOff);
      relay2State = relayOff;
      engineShouldBeOff = true;
      engineKillPulseTimer = 0;
      engineKillMidPoint = false;
    break; // end case finish
    
    default: ; // do nothing 
    
  } // end case(state){} 
} // end engineKill(String state)


void engineAllow(){
  if(relay2State == relayOn){
    digitalWrite(relay2, relayOff); // just in case it was on for some random reason; prevents frying relays and/or actuators
    relay2State = relayOff;
  } // end safety check
  
  if(relay1State == relayOff){ // if it's already on, no need to do anything, so only do something if its off
    relay1State = relayOn;
    pulseOutLow(relay1, 500);
    relay1State = relayOff;
  } // end if()
  engineShouldBeOff = false;
  //digitalWrite(relay1, relayOn); // eventually pulse it, but for now just latch it
} // end instantEngineKill

void engineAllow(char state){
  
  switch (state){
    
    case 's': // rising edge of pulsing the actuator
      engineAllowPulseTimer = millis();
      if(relay2State == relayOn){
        digitalWrite(relay2, relayOff); // just in case it was on for some random reason; prevents frying relays and/or actuators
        relay2State = relayOff;
      } // end safety check
      if(relay1State == relayOff){ // if it's already on, no need to do anything, so only do something if its off
        relay1State = relayOn;
        digitalWrite(relay1, relayOn);
      } // end if()
      engineAllowMidPoint = true;
    break; // end case start
    
    case 'f': // falling edge of pulsing the actuator
        digitalWrite(relay1, relayOff); // turns relay 1 off, completion of pulsing the actuator
        relay1State = relayOff;
        engineShouldBeOff = false;
        engineAllowPulseTimer = 0;
        engineAllowMidPoint = false;
    break; // end case stop
    
    default: ; // do nothing 
    
  } // end case(state){} 
} // end engineAllow(String state)


void brakesEngage(){
  
  if(relay3State == relayOn){ // if it's already onn, no need to do anything, so only do something if its off
    digitalWrite(relay3, relayOff); // engages the brakes by removing power, allowing the mower to move
    relay3State = relayOff;
  } // end if()
  brakesShouldBeOff = true;
} // end brakesEngage


void brakesDisengage(){
  
  if(relay3State == relayOff){ // if it's already on, no need to do anything, so only do something if its on
    digitalWrite(relay3, relayOn); // disengages the brakes by applying voltage
    relay3State = relayOn;
  } // end if()
  brakesShouldBeOff = false;
} // end brakesDisengage

void pulseOutLow(int pin, long duration){ // pulses a digital output for a certain length of time, meant for active low relays
  unsigned long timeStart = millis();
  boolean exitThis = false;
  digitalWrite(pin, LOW);
  while(!exitThis){
    if(millis() >= (timeStart + duration)){
      digitalWrite(pin, HIGH);
      exitThis = true;
    } // end if
  } // end while()
} // end pulseOutLow

void pulseOutHigh(int pin, long duration){ // pulses a digital output for a certain length of time
  unsigned long timeStart = millis();
  boolean exitThis = false;
  digitalWrite(pin, HIGH);
  while(!exitThis){
    if(millis() >= (timeStart + duration)){
      digitalWrite(pin, LOW);
      exitThis = true;
    } // end if
  } // end while()
} // end pulseOutHIGH

boolean userInput(){ // tests for user input via the transmitter
  boolean returnVal = false;
  boolean curPrevChange[arraySize]; // used an array to test for ignore channel.  May not be needed but makes it simpler
  
  // move current values to previous array, sample current values, populate current array
  for ( int i = 1; i <= numChannels; i++ ){
    prevVal[i] = curVal[i];
    curVal[i] = ServoDecode.GetChannelPulseWidth(i); // GetChannelPulseWidth technically requires a byte, but the number won't get high enough to cause problems
  } // end move old values to proper variable
  
     // Compute if there was a change
  for ( int i = 1; i <= numChannels; i++ ){
    if(i==throttleChannel){
      curPrevChange[i] =  abs(curVal[i] - 1500) > chChangeThreshold; // 1500 is the zero value in the range of the throttle, so this test
                                                                     // is to see if the mower is moving at a fixed speed in either direction, greater than the change threshold.
    } // end if      
    else{ curPrevChange[i] =  abs(curVal[i] - prevVal[i]) > chChangeThreshold;  }
         //end else
    returnVal = returnVal | curPrevChange[i]; // if anything changes, the value will go true
    } // end for() of compare channel values from each array and populate curPrevChange array
  return returnVal;
} // end userInput



