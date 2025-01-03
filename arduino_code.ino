#include <EEPROM.h>

const int trigPin1 = 8;
const int echoPin1 = 9;
const int trigPin2 = A1;
const int echoPin2 = A0;
const int trigPin3 = 12;
const int echoPin3 = 13;
const int enA =10;
const int enB =11;
int irpin =5;

const byte npulse = 3;
const bool sound = true;
const bool debug = true;

const byte pin_pulse=A4;
const byte pin_cap  =A5;
// defining variables
long duration1;
long duration2;
long duration3;
int distanceleft;
int distancefront;
int distanceright;
int a=0;
int motorSpeed=100; 

const int maxPathLength = 20; // Maximum allowed path length
int currentPathLength = 0;    // Current path length

// Motor driver pins
const int leftMotor1 = 2;
const int leftMotor2 = 3;
const int rightMotor1 = 6;
const int rightMotor2 = 7;


char command; // Variable to store Bluetooth commands

bool storingPath = false; // Flag to indicate whether we are currently storing a path
bool manualMode = true;  // Flag to indicate whether the robot is in manual mode
bool emergencyPaused = false; // Flag to indicate emergency pause status

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);// Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(irpin, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  if (debug) Serial.begin(9600);
  pinMode(pin_pulse, OUTPUT); 
  digitalWrite(pin_pulse, LOW);
  pinMode(pin_cap, INPUT); 

  analogWrite(enA, motorSpeed); 
  analogWrite(enB, motorSpeed); 
 
}

const int nmeas=176;  //measurements to take
long int sumsum=0; //running sum of 64 sums 
long int skip=0;   //number of skipped sums
long int diff=0;        //difference between sum and avgsum
long int flash_period=0;//period (in ms) 
long unsigned int prev_flash=0; //time stamp of previous flash


void loop() {
  if (Serial.available() > 0) {
    command = Serial.read();
    executeCommand(command);
  }

  if (!manualMode && !emergencyPaused) {
    // Run automatic function continuously until 'm' is pressed
    runAutomaticFunction();
    
  }
  if (!manualMode) {
    metalDetect();
  }
}

void executeCommand(char cmd) {
  if (cmd == 'm') {
    toggleMode();
    return; // Exit the function after toggling mode
  }
   if (cmd == 'e') {
    toggleEmergencyPause();
    return; // Exit the function after toggling emergency pause
  }

  if (manualMode) {
    // In manual mode, execute commands for manual control
    switch (cmd) {
      case 'f':
      case 'b':
      case 'l':
      case 'r':
      case 's':
        if (storingPath) {
          move(cmd);
          savePathToEEPROM(cmd);
          if (cmd == 's') {
            storingPath = false;
            Serial.println("Path stored. Press 'g' to replay normally or 'h' for reverse and opposite replay.");
          }
        } else {
          move(cmd);
        }
        break;
      case 'p':
        startStoringPath();
        break;
      case 'g':
        replayPathFromEEPROM(false); // Normal replay
        break;
      case 'h':
        replayPathFromEEPROM(true); // Reverse and opposite replay
        break;
      case 'd':
        displayEEPROMContents();
        break;
      case 'c':
        clearAllPathsInEEPROM();
        break;
      default:
        break;
    }
  }
}

void move(char cmd) {
  switch (cmd) {
    case 'f':
      moveForward();
      break;
    case 'b':
      moveBackward();
      break;
    case 'l':
      turnLeft();
      break;
    case 'r':
      turnRight();
      break;
    case 's':
      stopMoving();
      break;
    default:
      break;
  }
}
void moveForward() {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  delay(1000); // Delay for 1 second
  stopMoving(); // Stop after the specified duration
}

void moveBackward() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  delay(1000); // Delay for 1 second
  stopMoving(); // Stop after the specified duration
}

void turnLeft() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  delay(500); // Delay for 1 second
  stopMoving(); // Stop after the specified duration
}

void turnRight() {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  delay(500); // Delay for 1 second
  stopMoving(); // Stop after the specified duration
}

void stopMoving() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}

void savePathToEEPROM(char cmd) {
  if (currentPathLength < maxPathLength) {
    EEPROM.write(currentPathLength, cmd);
    currentPathLength++;
  }
}

void replayPathFromEEPROM(bool reverseAndOpposite) {
  int start = reverseAndOpposite ? currentPathLength - 1 : 0;
  int end = reverseAndOpposite ? -1 : currentPathLength;
  int step = reverseAndOpposite ? -1 : 1;

  for (int i = start; i != end; i += step) {
    char cmd = reverseAndOpposite ? reverseCommand(EEPROM.read(i)) : EEPROM.read(i);
    move(cmd);
    delay(1000); // Delay between each step (adjust as needed)
  }
  stopMoving();
}

void displayEEPROMContents() {
  Serial.println("EEPROM Contents:");
  for (int i = 0; i < currentPathLength; i++) {
    char cmd = EEPROM.read(i);
    Serial.print(cmd);
    Serial.print(" ");
  }
  Serial.println();
}

void clearAllPathsInEEPROM() {
  // Clear all saved paths in EEPROM
  for (int i = 0; i < maxPathLength; i++) {
    EEPROM.write(i, 0); // Write 0 to clear the EEPROM cell
  }
  currentPathLength = 0; // Reset current path length
}

void startStoringPath() {
  currentPathLength = 0;
  storingPath = true;
  Serial.println("Start storing path. Enter commands ('f', 'b', 'l', 'r', 's'). Press 's' to stop storing.");
}

void toggleMode() {
  // Toggle between manual mode and additional mode
  manualMode = !manualMode;

  if (manualMode) {
    Serial.println("Switched to Manual Mode");
  } else {
    Serial.println("Switched to Additional Mode");
  }
}

char reverseCommand(char cmd) {
  switch (cmd) {
    case 'f':
      return 'b';
    case 'b':
      return 'f';
    case 'l':
      return 'r';
    case 'r':
      return 'l';
    default:
      return cmd;
  }
}

void toggleEmergencyPause() {
  // Toggle emergency pause state
  emergencyPaused = !emergencyPaused;

  if (emergencyPaused) {
    Serial.println("Emergency Pause Activated. Motors Stopped.");
    stopMoving();
    
  } else {
    Serial.println("Emergency Pause Deactivated. Resuming previous operation.");
  }
}

void metalDetect(){
int minval=1023;
  int maxval=0;
  
  //perform measurement
  long unsigned int sum=0;
  for (int imeas=0; imeas<nmeas+2; imeas++){
    //reset the capacitor
    pinMode(pin_cap,OUTPUT);
    digitalWrite(pin_cap,LOW);
    delayMicroseconds(20);
    pinMode(pin_cap,INPUT);
    //apply pulses
    for (int ipulse = 0; ipulse < npulse; ipulse++) {
      digitalWrite(pin_pulse,HIGH); //takes 3.5 microseconds
      delayMicroseconds(3);
      digitalWrite(pin_pulse,LOW);  //takes 3.5 microseconds
      delayMicroseconds(3);
    }
    //read the charge on the capacitor
    int val = analogRead(pin_cap); //takes 13x8=104 microseconds
    minval = min(val,minval);
    maxval = max(val,maxval);
    sum+=val;
 
    //determine if LEDs should be on or off
    long unsigned int timestamp=millis();
    byte ledstat=0;
    if (timestamp<prev_flash+10){
      if (diff>0)ledstat=1;
      if (diff<0)ledstat=2;
    }
    if (timestamp>prev_flash+flash_period){
      if (diff>0)ledstat=1;
      if (diff<0)ledstat=2;
      prev_flash=timestamp;   
    }
    if (flash_period>1000)ledstat=0;

    //switch the LEDs to this setting
    /*if (ledstat==0){
      digitalWrite(pin_LED1,LOW);
      digitalWrite(pin_LED2,LOW);
      if(sound)noTone(pin_tone);
    }
    if (ledstat==1){
      digitalWrite(pin_LED1,HIGH);
      digitalWrite(pin_LED2,LOW);
      if(sound)tone(pin_tone,2000);
    }
    if (ledstat==2){
      digitalWrite(pin_LED1,LOW);
      digitalWrite(pin_LED2,HIGH);
      if(sound)tone(pin_tone,500);
    }*/
  
  }

  //subtract minimum and maximum value to remove spikes
  sum-=minval; sum-=maxval;
  
  //process
  if (sumsum==0) sumsum=sum<<6; //set sumsum to expected value
  long int avgsum=(sumsum+32)>>6; 
  diff=sum-avgsum;
  if (abs(diff)<avgsum>>10){      //adjust for small changes
    sumsum=sumsum+sum-avgsum;
    skip=0;
  } else {
    skip++;
  }
  if (skip>64){     // break off in case of prolonged skipping
    sumsum=sum<<6;
    skip=0;
  }

  // one permille change = 2 ticks/s
  if (diff==0) flash_period=1000000;
  else flash_period=avgsum/(2*abs(diff));    
    
  /*if (debug){
    Serial.print(nmeas); 
    Serial.print(" ");
    Serial.print(minval); 
    Serial.print(" ");
    Serial.print(maxval); 
    Serial.print(" ");
    Serial.print(sum); 
    Serial.print(" ");
    Serial.print(avgsum); 
    Serial.print(" ");
    Serial.print(diff); 
    Serial.print(" ");
    Serial.print(flash_period); 
    Serial.println();
  }*/
  if(flash_period<=200){
  Serial.println("Metal Detected");
  delay(40);
  }
}
void runAutomaticFunction() {
  //metalDetect();
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distanceleft = duration1 * 0.034 / 2;
  //Serial.print("Distance1: ");
 // Serial.println(distanceleft);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distancefront = duration2 * 0.034 / 2;
  //Serial.print("Distance2: ");
  //Serial.println(distancefront);
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distanceright = duration3 * 0.034 / 2;
  //Serial.print("Distance3: ");
  //Serial.println(distanceright);
  int s = digitalRead(irpin);
  if(s==HIGH)
  { 
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
      delay(1000);
    a=1;
    }
  if ((a==0)&&(s==LOW)&&(distanceleft <= 17 && distancefront > 17 && distanceright <= 17) || (a==0)&&(s==LOW)&&(distanceleft > 17 && distancefront > 17 && distanceright > 17))
  {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2,LOW);
  }
  if ((a==1)&&(s==LOW)||(s==LOW)&&(distanceleft <= 17 && distancefront <= 17 && distanceright > 17)||(s==LOW)&&(distanceleft <= 17 && distancefront <= 17 && distanceright > 17)||(s==LOW)&& (distanceleft <= 17 && distancefront > 17 && distanceright > 17)||(distanceleft <= 17 && distancefront > 17 && distanceright > 17))
  {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    delay(500);
    a=0;
  }
  if ((s==LOW)&&(distanceleft > 17 && distancefront <= 17 && distanceright <= 17) ||(s==LOW)&& (distanceleft > 17 && distancefront > 17 && distanceright <= 17) ||(s==LOW)&& (distanceleft > 17 && distancefront <= 17 && distanceright > 17) )
  {
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  } 
}



