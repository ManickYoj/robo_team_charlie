#include <Servo.h>

//Hind Brain Test
int ledPin = 13;
int actOutPin = 4;
int eStopPin = 5;
int delayPeriod = 100;
char command = 'g';
char hBrainStatus ='r';
int motorpin1= 6; //PWM pins for right and left motors
int motorpin2=7; 
Servo motor1;
Servo motor2;

void setup() {
  // put your setup code here, to run once:
  pinMode (ledPin, OUTPUT);
  pinMode (actOutPin, OUTPUT);
  pinMode (eStopPin, INPUT);
  motor1.attach(motorpin1);
  motor2.attach(motorpin2);
  
  Serial.begin(9600);
  Serial.println("Have Midbrain send 1 or 0 to Hinbrain pin 4");
}

void loop() {
  // Read midbrain commands
  if (Serial.available()){
    command = Serial.read();
    Serial.println("midbrain sent output:");
    Serial.println(command);
  }
  // Sense: Read Robot Sensors
  if (readEstop()==1){
    hBrainStatus = 'e';
  }
  else hBrainStatus = 'r';
  //Think:Run low level cognition ad safety code
  if (command!='s'){
    blink();
  }
  //Act: run actuators and behavior lights
  toggleActPin4(command);

  //Act: run motors
  if (command == 'g'){
    int pos=0;
      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
      motor1.write(pos);              // tell servo to go to position in variable 'pos'
      motor2.write(pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  // write status data up to midbrain
  if (command == 's'){
    Serial.println("hind brain stopped");
  }
  if (hBrainStatus == 'e'){
    Serial.println("hind brain E-stopped");
  }
}

boolean readEstop(){
  boolean eStopTriggered = digitalRead(eStopPin);
  return eStopTriggered;
}

void blink(){
    digitalWrite (ledPin, HIGH);
    delay (delayPeriod);
    digitalWrite (ledPin, LOW);
    delay (delayPeriod);
    }
void toggleActPin4 (char pinState){
  if (pinState=='1'){
    digitalWrite(actOutPin, HIGH);
  }
  else if (pinState!='1'){
    digitalWrite(actOutPin, LOW);
  }
}



