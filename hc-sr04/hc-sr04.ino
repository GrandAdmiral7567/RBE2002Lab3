#include <Zumo32U4.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>

#include "params.h"
#include "serial_comm.h"
#include "button.h"
#include "eventTimer.h"  //include your shiny, new event timer class


#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

bool useEmitters = true;

uint8_t selectedSensorIndex = 0;




volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;


//define the states for the echo capture
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};

//and initialize to IDLE
volatile PULSE_STATE pulseState = PLS_IDLE;

//this may be any appropriate pin, connect the pin to Trig on the sensor
const uint8_t trigPin = 14;

//for scheduling pings
uint32_t lastPing = 0;
uint32_t PING_INTERVAL = 17; //ms



volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows

const uint8_t buttonPin = 17; //button C on the Romi

Button buttonC(buttonPin, 10);


Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4Encoders encoders;
volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

static int16_t sumLeft = 0;
static int16_t sumRight = 0;

static int16_t usSumError = 0;

static int16_t lastUsError = 0;
static int16_t rateError = 0;
int currentTime = 0;
int elapsedTime = 0;
int previousTime = 0;


//declare the robot states here
enum robotStates {IDLE_STATE, WAITING_STATE, DRIVING_STATE, SPIN_STATE};
robotStates robotState;


struct segment {
  int leftTarget;
  int rightTarget;
  int dist;
};

const int numberOfSegments = 3;
segment segments[numberOfSegments];
eventTimer zumoTimer;
int iSeg;
int countLimit;
void capturePulse();
void PID(int l, int r);
int currentDist;
int targetDist = 30;




void setup(){
Serial.begin(115200);
  //while(!Serial) {}  //IF YOU DON'T COMMENT THIS OUT, YOU MUST OPEN THE SERIAL MONITOR TO START




  //while(!Serial) {} //you must open the Serial Monitor to get past this step!
  Serial.println("Hi!");

  noInterrupts(); //disable interupts while we mess with the control registers
  
  //sets timer 3 to normal mode (16-bit, fast counter)
  TCCR3A = 0; 
  
  interrupts(); //re-enable interrupts

  //note that the Arduino machinery has already set the prescaler elsewhere
  //so we'll print out the value of the register to figure out what it is
  Serial.print("TCCR3B = ");
  Serial.println(TCCR3B, HEX);

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)
  Button buttonC(buttonPin, 10);
  lastPing = millis();


  

  lineSensors.initFiveSensors();

  loadCustomCharacters();


  
  Serial.println("Hello.");
  iSeg = 0;
  robotState = IDLE_STATE;
  buttonC.Init(); //don't forget to call Init()!
  segments[0] = {30,30,400};
  segments[1] = {0,30,30};
  segments[2] = {30,30,35};

  countLimit += segments[iSeg].dist *50;

  noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 140;   //TOP goes in OCR4C
  TIMSK4 = 0x04; //enable overflow interrupt
  
  interrupts(); //re-enable interrupts

  //pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}
















void loop() 
{

  if(robotState == IDLE_STATE){
    motors.setSpeeds(0,0);
    //Serial.println("Press the Button");
    if(buttonC.CheckButtonPress()) {
      robotState = WAITING_STATE;
      Serial.println("Entering Waiting State");
      zumoTimer.start(1300);
    }
  }

  else if(robotState == WAITING_STATE){
    
    if(zumoTimer.checkExpired() == true){
      Serial.println("Entering Driving State");
      robotState = DRIVING_STATE;
    }
    
    
  }

  else if(robotState == SPIN_STATE){
    motors.setSpeeds(-300,300);

    
    if(zumoTimer.checkExpired() == true){
      Serial.println("Entering Idle State");
      robotState = IDLE_STATE;
    }
    
  }


 else if(robotState == DRIVING_STATE){



  static uint16_t lastSampleTime = 0;

  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();

    // Read the line sensors.
    lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  }
/*
    Serial.print('\t');
    Serial.print(lineSensorValues[0]);
    Serial.print('\t');
    Serial.print(lineSensorValues[4]);
    Serial.print('\n');
*/



 if( lineSensorValues[0]> 700 || lineSensorValues[4]> 700){
        iSeg++;
        //Serial.println("Updating iSeg");
        Serial.println(iSeg);
        robotState = SPIN_STATE;
        Serial.println("Entering SPIN state");
        zumoTimer.start(1300);
      }







  
  //schedule pings roughly every PING_INTERVAL milliseconds
  if(millis() - lastPing > PING_INTERVAL)
  {
    lastPing = millis();
    CommandPing(trigPin); //command a ping
  }
  
  if(pulseState == PLS_CAPTURED) //we got an echo
  {
    //update the state to IDLE
    pulseState = PLS_IDLE;

    /*
     * Calculate the length of the pulse (in timer counts!). Note that we turn off
     * interrupts for a VERY short period so that there is no risk of the ISR changing
     * pulseEnd or pulseStart. As noted in class, the way the state machine works, this
     * isn't a problem, but best practice is to ensure that no side effects can occur.
     */
    noInterrupts();
    uint16_t pulseLengthTimerCounts = pulseEnd - pulseStart;
    interrupts();
    
    //EDIT THIS LINE: convert pulseLengthTimerCounts, which is in timer counts, to time, in us
    //You'll need the clock frequency and the pre-scaler to convert timer counts to time
    
    uint32_t pulseLengthUS = 64.0000/16.0000 * pulseLengthTimerCounts; //pulse length in us


    //EDIT THIS LINE AFTER YOU CALIBRATE THE SENSOR
    float distancePulse = (pulseLengthUS - 133)/52.3;    //distance in cm
    boolean lessThan = false;
    /*
    if(distancePulse<30){
      lessThan = true;
      PID(0,30);
    }
    else{
      PID(30,0);
    }
    */

    currentTime = millis();
    elapsedTime = currentTime - previousTime;

    
    int16_t usError = 30 - distancePulse;

    if(usSumError + (usError *elapsedTime) <= 20){
          usSumError += usError *elapsedTime;
    }
    rateError = (usError - lastUsError)/elapsedTime;

    
    int rightSpeed = 20;
    int leftSpeed = 20;
    float speedValue = (Kp2 * usError); //+ (Ki2 * usSumError);  //+ (Kd2*rateError);
   
    lastUsError = usError;
    previousTime = currentTime;







    if(distancePulse<30){
      lessThan = true;
      leftSpeed = 20;
      rightSpeed = 20 + speedValue;
    }
    else{
      lessThan = false;
      leftSpeed = 20+abs(speedValue);
      rightSpeed = 20;
    }
 
 
    PID(leftSpeed,rightSpeed);



    Serial.print(millis());
    Serial.print('\t');
    Serial.print(pulseLengthTimerCounts);
    Serial.print('\t');
    Serial.print(pulseLengthUS);
    Serial.print('\t');
    Serial.print(distancePulse);
    Serial.print('\t');
    Serial.print(usError);
    Serial.print('\t');
    Serial.print(leftSpeed);
    Serial.print('\t');
    Serial.print(rightSpeed);
    Serial.print('\n');
  }

/*
  int targetLeft = 30;
  int targetRight = 30;
  if(distancePulse<30){
    PID(0,0);
  }
  else if(distancePulse>30){
    PID(30,30);
  }
  */

  

  
}




}




















/*
 * Commands the MaxBotix to take a reading
 */
void CommandPing(int trigPin)
{
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;
  
  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //we'll allow a delay here for convenience; it's only 10 us
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

/*
 * ISR for input capture on pin 13. We can precisely capture the value of TIMER3
 * by setting TCCR3B to capture either a rising or falling edge. This ISR
 * then reads the captured value (stored in ICR3) and copies it to the appropriate
 * variable.
 */
ISR(TIMER3_CAPT_vect)
{
  if(pulseState == PLS_WAITING_LOW) //we're waiting for a rising edge
  {
    pulseStart = ICR3; //copy the input capture register (timer count)
    TCCR3B &= 0xBF;    //now set to capture falling edge on pin 13
    pulseState = PLS_WAITING_HIGH;
  }

  else if(pulseState == PLS_WAITING_HIGH) //waiting for the falling edge
  {
    pulseEnd = ICR3;
    pulseState = PLS_CAPTURED; //raise a flag to indicate that we have data
  }
}



/*
 * ISR for timing. Basically, raise a flag on overflow. Timer4 is set up to run with a pre-scaler 
 * of 1024 and TOP is set to 249. Clock is 16 MHz, so interval is dT = (1024 * 250) / 16 MHz = 16 ms.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}


void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  lcd.loadCustomCharacter(levels + 0, 0);  // 1 bar
  lcd.loadCustomCharacter(levels + 1, 1);  // 2 bars
  lcd.loadCustomCharacter(levels + 2, 2);  // 3 bars
  lcd.loadCustomCharacter(levels + 3, 3);  // 4 bars
  lcd.loadCustomCharacter(levels + 4, 4);  // 5 bars
  lcd.loadCustomCharacter(levels + 5, 5);  // 6 bars
  lcd.loadCustomCharacter(levels + 6, 6);  // 7 bars
}







void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, 255};
  lcd.print(barChars[height]);
}






void printReadingsToLCD()
{
  // On the first line of the LCD, display the bar graph.
  lcd.gotoXY(0, 0);
  for (uint8_t i = 0; i < 5; i++)
  {
    uint8_t barHeight = map(lineSensorValues[i], 0, 2000, 0, 8);
    printBar(barHeight);
  }

  // Print "E" if the emitters are on, "e" if they are off.
  lcd.gotoXY(7, 0);
  lcd.print(useEmitters ? 'E' : 'e');

  // On the second line of the LCD, display one raw reading.
  lcd.gotoXY(0, 1);
  lcd.print(selectedSensorIndex);
  lcd.print(F(": "));
  lcd.print(lineSensorValues[selectedSensorIndex]);
  lcd.print(F("    "));
}







// Prints a line with all the sensor readings to the serial
// monitor.
void printReadingsToSerial()
{
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d %c\n",
    lineSensorValues[0],
    lineSensorValues[1],
    lineSensorValues[2],
    lineSensorValues[3],
    lineSensorValues[4],
    useEmitters ? 'E' : 'e'
  );
  //Serial.print(buffer);
}









void PID(int l, int r){
  targetLeft = l;
  targetRight = r;
    if(readyToPID) //timer flag set
  {
    //clear the timer flag
    readyToPID = 0;

    //for tracking previous counts
    static int16_t prevLeft = 0;
    static int16_t prevRight = 0;
    
    //error sum
    sumLeft = 0;
    sumRight = 0;

    /*
     * Do PID stuffs here. Note that we turn off interupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
     */
    noInterrupts();
    int16_t speedLeft = countsLeft - prevLeft;
    int16_t speedRight = countsRight - prevRight;

    prevLeft = countsLeft;
    prevRight = countsRight;
    interrupts();

    int16_t errorLeft = targetLeft - speedLeft;
    int16_t errorRight = targetRight - speedRight;



    if (bufferIndexRight < 99) {
      bufferCountRight[bufferIndexRight] = errorLeft;
      bufferIndexRight++;
    } else bufferIndexRight = 0;

    for (int i = 0; i < 99; i++) {
      sumRight += bufferCountRight[i];
    }

    if(sumRight > 170){
      sumRight = 170;
    }
    else if(sumLeft < -135){
      sumRight = -135;
    }

  
    if (bufferIndexLeft < 99) {
      bufferCountLeft[bufferIndexLeft] = errorLeft;
      bufferIndexLeft++;
    } else bufferIndexLeft = 0;


    for (int i = 0; i < 99; i++) {
      sumLeft += bufferCountLeft[i];
    }

    if(sumLeft > 170){
      sumLeft = 170;
    }
    else if(sumLeft < -135){
      sumLeft = -135;
    }

    
    
    float effortLeft = Kp * errorLeft + Ki * sumLeft;
    float effortRight = Kp * errorRight + Ki * sumRight;

    
    motors.setSpeeds(effortLeft, effortRight); //up to you to add the right motor
}
}
