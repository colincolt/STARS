//#include <SPI.h>
//#include <Wire.h>
#include <IRremote.h>

#define PIN_IR 9
#define PIN_IR2 12

#define SENSORPIN1 10 //Sensor 1 is on the front bottom
#define SENSORPIN2 11 //Sensor 2 is on the back bottom
#define SENSORPIN3 12 //Sensor 3 is on the front top
#define SENSORPIN4 8 //Sensor 4 is on the back top

#define REDPIN 49
#define GREENPIN 51
#define BLUEPIN 53

IRsend irsend;



const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

int dataNumber = 0;             // new for this version



long end_time; // When Sensor 2 is triggered
long start_time; // When Sensor 1 is triggered
long elapsed_time; // End time minus start time

float ball_speed1; // Speed calculated
float ball_speed2; // Speed calculated

int trigger1 = 0; // Sensor 1
int trigger2 = 0; // Sensor 2
int trigger3 = 0; // Sensor 3
int trigger4 = 0; // Sensor 4

int target;

int sensor1State; // Sensor 1 status
int sensor2State; // Sensor 2 status
int sensor3State; // Sensor 3 status
int sensor4State; // Sensor 4 status

void setup() {
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  pinMode(SENSORPIN1, INPUT); // Sensor 1 as input
//  digitalWrite(SENSORPIN1, HIGH); // Turn on the pullup
  pinMode(SENSORPIN2, INPUT); // Sensor 2 as input
//  digitalWrite(SENSORPIN2, HIGH); // Turn on the pullup
  pinMode(SENSORPIN3, INPUT); // Sensor 2 as input
  pinMode(SENSORPIN4, INPUT); // Sensor 2 as input
  Serial.begin(9600);
//  while (! Serial);
  Serial.println("Target 1 = 1; Target 2 = 2");
  irsend.enableIROut(38);
  irsend.mark(0);

}

// Function to determine speed
void speed()
{
  // subtract end time from start time to get total time
  elapsed_time = (end_time - start_time);

  // convert mm/s to m/s
  ball_speed1 = ((224000.00 / elapsed_time));
  ball_speed2 = ball_speed1*3.6;

    Serial.println("GOAL!");
    Serial.print("Ball Speed (m/s) = ");
    Serial.println(ball_speed1);
    Serial.print("Ball Speed (km/h) = ");
    Serial.println(ball_speed2);
    Serial.println();
}

void loop() {

  recvWithEndMarker();
//  showNewNumber();
  
  if (target == 1)
  {  
  digitalWrite(REDPIN, LOW);
  digitalWrite(GREENPIN, LOW); 
  digitalWrite(BLUEPIN, HIGH); 
  
  // Read the state of the IR sensor 1;   
  sensor1State = digitalRead(SENSORPIN1);
  sensor3State = digitalRead(SENSORPIN3);
  
  // See if IR beam of sensor 1 has been broken
  if (sensor1State == HIGH || sensor3State == HIGH) {

    // Check to make sure both sensors have not triggered
    if ((trigger1 == 0 && trigger2 == 0) && (trigger3 == 0 && trigger4 == 0)) {

      // Save time when sensor 1 was triggered
      start_time = micros();

      // Prevent sensor 1 from triggering again
      trigger1 = 1;
      trigger3 = 1;
    }
  }

  // Read the state of the IR sensor 2:
  sensor2State = digitalRead(SENSORPIN2);
  sensor4State = digitalRead(SENSORPIN4);
  
  // See if IR beam of sensor 2 has been broken
  if (sensor2State == HIGH || sensor4State == HIGH) {

    // Check to make sure sensor 1 has triggered but not sensor2
    if ((trigger2 == 0 && trigger1 == 1) && (trigger4 == 0 && trigger3 == 1)) {

      // Save time when sensor 2 was triggered
      end_time = micros();

      digitalWrite(REDPIN, LOW);
      digitalWrite(GREENPIN, HIGH); 
      digitalWrite(BLUEPIN, LOW);  
 
      // Run speed function
      speed();

      // Prevent sensor 2 from triggering again
      trigger2 = 1;
      trigger4 = 1;
    }
    delay(2000);

    // Reset both sensors
    trigger1 = 0;
    trigger2 = 0;
    trigger3 = 0;
    trigger4 = 0;
  }
  }

  else{
  digitalWrite(REDPIN, HIGH);
  digitalWrite(GREENPIN, LOW); 
  digitalWrite(BLUEPIN, LOW); 
  }
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
            dataNumber = 0;             // new for this version
            dataNumber = atoi(receivedChars);   // new for this version
            target = dataNumber;
            newData = false;
        }
    }
}

//void showNewNumber() {
//    if (newData == true) {
//        dataNumber = 0;             // new for this version
//        dataNumber = atoi(receivedChars);   // new for this version
////        Serial.print("This just in ... ");
////        Serial.println(receivedChars);
////        Serial.print("Data as Number ... ");    // new for this version
////        Serial.println(dataNumber);     // new for this version
//        target = dataNumber;
//        newData = false;
//    }
//}
