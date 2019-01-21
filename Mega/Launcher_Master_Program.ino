/********************************************************************/
// ---------------------------- LIDAR ----------------------------- //

unsigned long pulseWidth;

#define TRIGGER 22 //GREEN
#define MONITOR 23 //BLUE


/********************************************************************/
// -------------------------- TEMPERATURE ------------------------- //

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 29

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

 float Celcius = 0;
// float Fahrenheit=0;


/********************************************************************/
// -------------------------- BALL FEEDER ------------------------- //

#include <AccelStepper.h>

// Define a stepper and the pins it will use

#define STEP 24
#define DIR 25

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);

int Signal = 0;
int interval = 3600;
int pos;
int initial = 0;



/********************************************************************/
// --------------------- SIMPLE BALL FEEDER ----------------------- //

/*     Simple Stepper Motor Control Exaple Code
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
// defines pins numbers
const int stepPin = 39; 
const int dirPin = 38; 


/********************************************************************/
// ------------------------- SERIAL INPUT ------------------------- // 

// Example 5 - Receive with start- and end-markers combined with parsing

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

boolean newData = false;

int data_receive = 0;


/********************************************************************/
// ------------------------ MOTOR CONTROL ------------------------- // 

#define IN1 4 // Arduino pin 4 is connected to MDDS30 pin IN1.
#define AN1 5 // Arduino pin 5 is connected to MDDS30 pin AN1.
#define AN2 6 // Arduino pin 6 is connected to MDDS30 pin AN2.
#define IN2 7 // Arduino pin 7 is connected to MDDS30 pin IN2.


int Motor1_Speed = 0;
int Motor2_Speed = 0;

int Controlled = 0;

//==================================================================//

int Initialize = 0;

unsigned long start_time;











void setup() {

/********************************************************************/
// ---------------------------- LIDAR ----------------------------- //

  pinMode(TRIGGER, OUTPUT); // Set pin 2 as trigger pin 
  digitalWrite(TRIGGER, LOW); // Set trigger LOW for continuous read

  pinMode(MONITOR, INPUT); // Set pin 3 as monitor pin 


/********************************************************************/
// -------------------------- TEMPERATURE ------------------------- //

  sensors.begin();


/********************************************************************/
// -------------------------- BALL FEEDER ------------------------- //

  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);


/********************************************************************/
// --------------------- SIMPLE BALL FEEDER ----------------------- //

  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);


/********************************************************************/
// ------------------------- SERIAL INPUT ------------------------- // 

    Serial.begin(9600);
    Serial.println("This demo expects 2 pieces of data - Motor 1 Speed, Motor 2 Speed");
    Serial.println("Enter data in this style <2000,3000>  ");
    Serial.println();


/********************************************************************/
// ------------------------ MOTOR CONTROL ------------------------- // 

  pinMode(AN1, OUTPUT);
  pinMode(AN2, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(AN1, LOW);
  digitalWrite(AN2, LOW);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

//==================================================================//

void loop() {
  // put your main code here, to run repeatedly:
  
if (Initialize == 0){
 getTemp();
 Initialize = 1; 
}

else{
LIDAR();
Serial_Input(); 

if (data_receive == 1){
  if (Controlled == 0){
  Motor_Control();
 }
 else if ((millis() - start_time) > 5000){
  simple_FeedBall();
  Motor1_Speed = 0;
  Motor2_Speed = 0;
  Motor_Control();

  Initialize = 0;
  data_receive = 0;
  Controlled = 0; 
 }
 else{}
 
}
}
}








//==================================================================//

unsigned long LIDAR()
{
  pulseWidth = pulseIn(MONITOR, HIGH); // Count how long the pulse is high in microseconds

  // If we get a reading that isn't zero, let's print it
  if(pulseWidth != 0)
  {
    pulseWidth = pulseWidth / 10; // 10usec = 1 cm of distance
    Serial.println(pulseWidth); // Print the distance
//    delay(500);
  }
}


//==================================================================//

float getTemp()
{
  sensors.requestTemperatures(); 
  Celcius = sensors.getTempCByIndex(0);
//  Fahrenheit = sensors.toFahrenheit(Celcius);
  Serial.print(Celcius);
  Serial.println(" C");

//  Serial.print(" F  ");
//  Serial.println(Fahrenheit);
  delay(1000);
}


//==================================================================//

void FeedBall()
{

if (Signal = 1)
{
  
 if (stepper.distanceToGo() == 0)
  {
    if (initial == 0)
    {
      pos = interval;
      stepper.moveTo(pos);
      initial = 1;
    }

    else if (initial == 1)
    {
      pos = 0;
      stepper.moveTo(pos);
      initial = 2;
    }

    else
    {
      initial = 0;
      Signal = 0;
      return;
    }
    
  }

  stepper.run();   
}
  
}




//==================================================================//

void simple_FeedBall(){
  
     digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 1200; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
  delay(1000); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 1200; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(500);
  }
  delay(1000); 
}




//==================================================================//

void Serial_Input() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
        data_receive = 1;
        start_time = millis();
    }
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    Motor1_Speed = atoi(strtokIndx);     // convert this part to an integer
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Motor2_Speed = atoi(strtokIndx);     // convert this part to an integer

}

//============

void showParsedData() {
    Serial.print("Motor 1 Speed = ");
    Serial.println(Motor1_Speed);
    Serial.print("Motor 2 Speed = ");
    Serial.println(Motor2_Speed);
}



//==================================================================//

void Motor_Control()
{
      analogWrite(AN1, Motor1_Speed);    
      analogWrite(AN2, Motor2_Speed); 
      Controlled = 1;
}


//=============================== END ==============================//
