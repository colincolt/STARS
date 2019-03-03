/********************************************************************/
// ---------------------------- LIDAR ----------------------------- //

unsigned long pulseWidth;
int lidar_2_Distance;

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


///********************************************************************/
//// -------------------------- BALL FEEDER ------------------------- //
//
//#include <AccelStepper.h>
//
//// Define a stepper and the pins it will use
//
//#define STEP 24
//#define DIR 25
//
//AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
//
//int Signal = 0;
//int interval = 3600;
//int pos;
//int initial = 0;



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
int ballFeed;
unsigned long Motor_Start_Time;


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

//#define IN1 4 // Arduino pin 4 is connected to MDDS30 pin IN1.
#define AN1 5 // Arduino pin 5 is connected to MDDS30 pin AN1.
#define AN2 6 // Arduino pin 6 is connected to MDDS30 pin AN2.
//#define IN2 7 // Arduino pin 7 is connected to MDDS30 pin IN2.


int Motor1_Speed = 0;
int Motor2_Speed = 0;

int Motor_Initialize = 0;


/********************************************************************/
// -------------------------- WIRELESS ---------------------------- // 

/*************************************************************************
 * Master-unit - nRF24L01+ radio communications                          *
 *      A program to operate as a master unit that transmits to and      *
 *      receives data from three remote slave devices. Each slave        *
 *      also has an nrf24l01+ transceiver, and replies with data         *
 *      using the Acknowledgement payload facility of the Enhanced       *
 *      ShockBurst packet structure.                                     *
 *                                                                       *
 *      Author: B.D. Fraser                                              *
 *                                                                       *
 *        Last modified: 25/06/2017                                      *
 *                                                                       *
 *************************************************************************/

//there is still an issue if the difficulty = 6

// nRF24l01 Pin Layout for Arduino Mega
// VCC - 5V
// GND - GND
// CE - 9
// CSN - 10
// SCK - 52
// MOSI - 51
// MISO - 50
// (CE and CSN can be user defined)


// MultiTxAckPayload - the master or the transmitter
//   works with two Arduinos as slaves
//     each slave should the SimpleRxAckPayload program
//       one with the adress {'R','x','A','A','A'}
//         and the other with {'R','x','A','A','B'}

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN   9
#define CSN_PIN 10

const byte numSlaves = 5;
const byte numTargets = 4;

const byte slaveAddress[numSlaves][5] = {
        // each slave needs a different address
                            {'R','x','A','A','A'}, //Target 1
                            {'R','x','A','A','B'}, //Target 2
                            {'R','x','A','A','C'}, //Target 3
                            {'R','x','A','A','D'}, //Target 4
                            {'R','x','A','A','E'}  //Player
                        };

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio


//const byte numChars = 32;
//char receivedChars[numChars];
//char tempChars[numChars];        // temporary array for use when parsing

int targetChoice;
int difficulty;
int voiceCommand = -1;
int Wireless_Stage = 0;
int Speed_Poll = 7;
int VR_poll = 1;

unsigned long time_of_flight;
unsigned long wirelessDelay;

float targetBallSpeed;

unsigned long targetTiming;

//boolean newData = false;


int blank;

int dataNumber = 0;             // new for this version
//~ char dataToSend[10] = "Message 0";
char dataToSend[10] = "ToSlvN  0";
char txNum = '0';

//int ackData[2] = {-1, -1}; // to hold the two values coming from the slave

float ackData;
bool new_data = false;
bool new_data2 = false;

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per second

//==================================================================//

int Process_Stage = 0;


unsigned long Initial_Time;
unsigned long Launch_Start_Time;

unsigned long Drill_Start_Time;
unsigned long Drill_End_Time;
unsigned long Drill_Total_Time; 

unsigned long Base_Drill_Time = 10000; // Base time for a drill (10 seconds)
unsigned long Allowed_Time;            // Total drill time which is a multiple of the base time
            

//int voiceCommand = -1;
float ballSpeed;
//unsigned long timing = 0;
int temperature;
int distance;











void setup() {

/********************************************************************/
// ---------------------------- LIDAR ----------------------------- //

  pinMode(TRIGGER, OUTPUT); // Set pin 2 as trigger pin 
  digitalWrite(TRIGGER, LOW); // Set trigger LOW for continuous read

  pinMode(MONITOR, INPUT); // Set pin 3 as monitor pin 


/********************************************************************/
// -------------------------- TEMPERATURE ------------------------- //

  sensors.begin();


///********************************************************************/
//// -------------------------- BALL FEEDER ------------------------- //
//
//  stepper.setMaxSpeed(3000);
//  stepper.setAcceleration(1000);


/********************************************************************/
// --------------------- SIMPLE BALL FEEDER ----------------------- //

  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);


/********************************************************************/
// ------------------------- SERIAL INPUT ------------------------- // 

    Serial.begin(115200);
//    Serial.println("This demo expects 6 pieces of data - MotorSpeed1 (RPM), MotorSpeed2 (RPM), targetChoice, difficulty, ballFeed, time_of_flight (half the TOF)");
//    Serial.println("Enter data in this style <2000,3000,1,5,1,5000>  ");
//    Serial.println();


/********************************************************************/
// ------------------------ MOTOR CONTROL ------------------------- // 

  pinMode(AN1, OUTPUT);
  pinMode(AN2, OUTPUT);

//  pinMode(IN1, OUTPUT);
//  pinMode(IN2, OUTPUT);

  digitalWrite(AN1, LOW);
  digitalWrite(AN2, LOW);

//  digitalWrite(IN1, LOW);
//  digitalWrite(IN2, LOW);


/********************************************************************/
// -------------------------- WIRELESS ---------------------------- // 

//    Serial.begin(115200);
//    Serial.println("This demo expects 2 pieces of data - target, difficulty");
//    Serial.println("Enter data in this style <2,3>  ");
    Serial.println();
    Serial.println(F("Source File = /mnt/SGT/SGT-Prog/Arduino/ForumDemos/nRF24Tutorial/MultiTxAckPayload.ino "));
    Serial.println("SimpleTxAckPayload Starting");

    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.setPALevel(RF24_PA_MAX);

    radio.enableAckPayload();

    radio.setRetries(15,10); // delay, count
        // radio.openWritingPipe(slaveAddress); -- moved to loop()

//    radio.setCRCLength(RF24_CRC_16);

    Initial_Time = millis();
}





  

//==================================================================//




void loop() {
  // put your main code here, to run repeatedly:

  receiveVR_Data();
  lidar_2_Distance = LIDAR();

  Serial_Output();
  
  Serial_Input();

  if (difficulty == 6){
    target_shutdown();
  }
  
  if (Process_Stage == 1)
    {
     temperature = getTemp();

     if (difficulty == -1){
     target_startup();
     }
     
     Process_Stage = 2; 
    }

  if (Process_Stage == 2)
    {
    if ((Motor1_Speed > 0) && (Motor2_Speed > 0))
      {
        if (Motor_Initialize == 0)
             {
              Motor_Control();  
             }
         else if (((millis() - Motor_Start_Time) >= 5000) && (ballFeed == 1))
             {
              //Drill_Start_Time is set in the ball feed function
              simple_FeedBall();
//              Serial.print("Ball Fed\n");
              Motor1_Speed = 0;
              Motor2_Speed = 0;
              Motor_Control();
            
              Process_Stage = 3;
              //data_receive = 0;
              Motor_Initialize = 0; 
              ballFeed = 0;
             }
         else
            {}
       
      }
    }

   if (Process_Stage == 3)
   {

      if ((targetChoice > 0) && ((difficulty > 0) && (difficulty <= 5))){

        if (millis() - Drill_Start_Time >= wirelessDelay){
           send(); 
        }
      }    
   }

   if (Process_Stage == 4)
   {
    poll();
    //reset inside the poll() function

    if ((millis() - Drill_Start_Time) >= Allowed_Time)
    {
       Drill_End_Time = millis();
       targetTiming = Drill_End_Time - Drill_Start_Time;
       
       //Reset Variables      
       Process_Stage = 0;
       targetChoice = 0;
       difficulty = 0;

    }

  }


}








//==================================================================//


void target_startup() {

        // call each slave in turn
    for (byte n = 0; n < numTargets; n++){

            // open the writing pipe with the address of a slave
        radio.openWritingPipe(slaveAddress[n]);

            // include the slave number in the message
//        dataToSend[5] = n + '0';

        bool rslt;
        rslt = radio.write( &difficulty, sizeof(difficulty) );
            // Always use sizeof() as it gives the size as the number of bytes.
            // For example if dataToSend was an int sizeof() would correctly return 2

        Serial.print("  ========  For Target ");
        Serial.print(n);
        Serial.println("  ========");
        Serial.print("  Data Sent ");
        Serial.println(difficulty);
        if (rslt) {
            if ( radio.isAckPayloadAvailable() ) {
                radio.read(&ackData, sizeof(ackData));
                new_data = true;
                Serial.println(" Confirmation of data send ");
                updateMessage();
            }
            Serial.print(" Target ");
            Serial.print(n);
            Serial.println(" activated");
            }

        else {
            Serial.println("  Tx failed");
        }

 }
  difficulty = 0;
  Serial.print("Difficulty = ");
  Serial.println(difficulty);
}




void target_shutdown() {

        // call each slave in turn
    for (byte n = 0; n < numTargets; n++){

            // open the writing pipe with the address of a slave
        radio.openWritingPipe(slaveAddress[n]);

            // include the slave number in the message
//        dataToSend[5] = n + '0';

        bool rslt;
        rslt = radio.write( &difficulty, sizeof(difficulty) );
            // Always use sizeof() as it gives the size as the number of bytes.
            // For example if dataToSend was an int sizeof() would correctly return 2

        Serial.print("  ========  For Target ");
        Serial.print(n);
        Serial.println("  ========");
        Serial.print("  Data Sent ");
        Serial.println(difficulty);
        if (rslt) {
            if ( radio.isAckPayloadAvailable() ) {
                radio.read(&ackData, sizeof(ackData));
                new_data = true;
                Serial.println(" Confirmation of data send ");
                updateMessage();
            }
            Serial.print(" Target ");
            Serial.print(n);
            Serial.println(" deactivated");

            }

        else {
            Serial.println("  Tx failed");
        }

 }
  difficulty = 0;
  Serial.print("Difficulty = ");
  Serial.println(difficulty);
}



void send() {

        // call each slave in turn
//    for (byte n = 0; n < numSlaves; n++){

            // open the writing pipe with the address of a slave
        radio.openWritingPipe(slaveAddress[targetChoice - 1]);

            // include the slave number in the message
//        dataToSend[5] = n + '0';

        bool rslt;
        rslt = radio.write( &difficulty, sizeof(difficulty) );
            // Always use sizeof() as it gives the size as the number of bytes.
            // For example if dataToSend was an int sizeof() would correctly return 2

        Serial.print("  ========  For Slave ");
        Serial.print(targetChoice);
        Serial.println("  ========");
        Serial.print("  Data Sent ");
        Serial.println(difficulty);
        if (rslt) {
            if ( radio.isAckPayloadAvailable() ) {
                radio.read(&ackData, sizeof(ackData));
                new_data = true;
                Allowed_Time = (Base_Drill_Time) - (difficulty)*(1000);
                Serial.print("Allowed Time = ");
                Serial.println(Allowed_Time);                
                Serial.println(" Confirmation of data send ");
                updateMessage();
                Process_Stage = 4;

                Serial.print(" Process_Stage = ");
                Serial.println(Process_Stage);


            }
            else {
                Serial.println("  Acknowledge but no data ");
            }
        }
        else {
            Serial.println("  Tx failed");
        }
        showData();
        Serial.print("\n");
        prevMillis = millis();
 }




void poll() {

        // call each slave in turn
//    for (byte n = 0; n < numSlaves; n++){

            // open the writing pipe with the address of a slave
        radio.openWritingPipe(slaveAddress[targetChoice - 1]);

            // include the slave number in the message
//        dataToSend[5] = n + '0';

        bool rslt2;
        rslt2 = radio.write( &Speed_Poll, sizeof(Speed_Poll) );
//        rslt2 = radio.write( &blank, sizeof(blank));
            // Always use sizeof() as it gives the size as the number of bytes.
            // For example if dataToSend was an int sizeof() would correctly return 2

        Serial.print("  ========  For Slave ");
        Serial.print(targetChoice);
        Serial.println("  ========");
        Serial.print("  Data Sent = ");
        Serial.println(Speed_Poll);
        if (rslt2) {
            if ( radio.isAckPayloadAvailable() ) {
                radio.read(&targetBallSpeed, sizeof(targetBallSpeed));
                new_data2 = true;
        
                Drill_End_Time = millis();
                Serial.print("Drill End Time = ");
                Serial.println(Drill_End_Time);
                targetTiming = Drill_End_Time - Drill_Start_Time;
                Serial.print("targetTiming = ");
                Serial.println(targetTiming);
               
                //Reset Variables      
                Process_Stage = 0;
                targetChoice = 0;
                difficulty = 0;

                Serial.println(" Confirmation of data send ");
                updateMessage();

                

            }
            else {
                Serial.println("  Acknowledge but no data ");
            }
        }
        else {
            Serial.println("  Tx failed");
        }
        showData2();
        Serial.print("\n");
        prevMillis = millis();
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
    return pulseWidth;
  }
}



//==================================================================//

float getTemp()
{
  sensors.requestTemperatures(); 
  Celcius = sensors.getTempCByIndex(0);
//  Fahrenheit = sensors.toFahrenheit(Celcius);
  Serial.print(Celcius);
//  Serial.write(Celcius);
  Serial.println(" C");

//  Serial.print(" F  ");
//  Serial.println(Fahrenheit);
  delay(1000);
  return Celcius;
}



//==================================================================//

//void FeedBall()
//{
//
//if (Signal = 1)
//{
//  
// if (stepper.distanceToGo() == 0)
//  {
//    if (initial == 0)
//    {
//      pos = interval;
//      stepper.moveTo(pos);
//      initial = 1;
//    }
//
//    else if (initial == 1)
//    {
//      pos = 0;
//      stepper.moveTo(pos);
//      initial = 2;
//    }
//
//    else
//    {
//      initial = 0;
//      Signal = 0;
//      return;
//    }
//    
//  }
//
//  stepper.run();   
//}
//  
//}




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
  
  Drill_Start_Time = millis();
  Serial.print("Drill Start Time = ");
  Serial.println(Drill_Start_Time);
  
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

void Motor_Control()
{
      analogWrite(AN1, Motor1_Speed);    
      analogWrite(AN2, Motor2_Speed); 
      Motor_Initialize = 1;
}



//==================================================================//

///* Function: receiveNodeData
// *    Make a radio call to each node in turn and retreive a message from each
// */
//void receiveNodeData(int node)
//{
////    Serial.print("[*] Master unit has successfully sent and received data ");
////    Serial.print(masterSendCount);
////    Serial.println(" times.");
//    
//    // make a call for data to each node in turn
//    //for (byte node = 0; node < 3; node++) {
//        
//        // setup a write pipe to current sensor node - must match the remote node listening pipe
//        radio.openWritingPipe(slaveAddress[node]);
//        
//        Serial.print("[*] Attempting to transmit data to node ");
//        Serial.println(node + 1);
//        Serial.print("[*] The master unit signal being sent is: ");
//        Serial.println(Speed_Poll);
//        
//        // boolean to indicate if radio.write() tx was successful
//        bool tx_sent;
//        tx_sent = radio.write( &Speed_Poll, sizeof(Speed_Poll) );
//        
//        // if tx success - receive and read slave node ack reply
//        if (tx_sent) {
//            if (radio.isAckPayloadAvailable()) {
//                
//                // read ack payload and copy data to relevant remoteNodeData array
//                radio.read(&ballSpeed, sizeof(ballSpeed));
//                
//                Serial.print("[+] Successfully received data from node: ");
//                Serial.println(node);
//                Serial.print("  ---- The speed received was: ");
//                Serial.println(ballSpeed);
//
//                if (ballSpeed > -1.0)
//                {
//                  Drill_End_Time = millis();
//                  Drill_Time = Drill_End_Time - Drill_Start_Time;
//                  Process_Stage = 0;  
//                  node = -1;
//                  difficulty = 0;
//                }
//
//                
////                // iterate master device count - keeps data changing between transmissions
////                if (masterSendCount < 500) {
////                    masterSendCount++;
////                } else {
////                    masterSendCount = 1;
////                }
//            }
//        }
//        else {
//            Serial.print("[-] The transmission to the selected node failed.");
//        }
//    
//    Serial.println("--------------------------------------------------------");
//}


//==================================================================//

void receiveVR_Data()
{

        // setup a write pipe to current sensor node - must match the remote node listening pipe
        radio.openWritingPipe(slaveAddress[4]);
        
        // boolean to indicate if radio.write() tx was successful
        bool tx_sent;
        tx_sent = radio.write( &VR_poll, sizeof(VR_poll) );
        
        // if tx success - receive and read slave node ack reply
        if (tx_sent) {
            if (radio.isAckPayloadAvailable()) {
                
                // read ack payload and copy data to relevant remoteNodeData array
                radio.read(&voiceCommand, sizeof(voiceCommand));
                
                Serial.print("[+] Successfully received data from node: ");
                Serial.println(5);
                Serial.print("  ---- The VR instruction received was: ");
                Serial.println(voiceCommand);

                Process_Stage = 0;
                
            }
        }
        else {
//            Serial.print("[-] The transmission to the selected node failed.");
        }
    
//    Serial.println("--------------------------------------------------------");  
}


//==================================================================//

//void InitializeDrill(int node)
//{
//
////    Serial.print("[*] Master unit has successfully sent and received data ");
////    Serial.print(masterSendCount);
////    Serial.println(" times.");
//    
//    // make a call for data to each node in turn
//    //for (byte node = 0; node < 3; node++) {
//        
//        // setup a write pipe to current sensor node - must match the remote node listening pipe
//        radio.openWritingPipe(nodeAddresses[node]);
//        
//        Serial.print("[*] Attempting to transmit data to node ");
//        Serial.println(node + 1);
//        Serial.print("[*] The master unit signal being sent is: ");
//        Serial.println(difficulty);
//        
//        // boolean to indicate if radio.write() tx was successful
//        bool tx_sent;
//        tx_sent = radio.write( &difficulty, sizeof(difficulty) );
//        
//        // if tx success - receive and read slave node ack reply
//        if (tx_sent) {
////            if (radio.isAckPayloadAvailable()) {
////                
////                // read ack payload and copy data to relevant remoteNodeData array
////                radio.read(&remoteNodeData[node], sizeof(remoteNodeData[node]));
////                
//                Serial.print("[+] Successfully transmitted data to node: ");
//                Serial.println(node + 1);
//                Process_Stage  = 3;
////                Serial.print("  ---- The speed received was: ");
////                Serial.println(remoteNodeData[node][1]);
//                
////                // iterate master device count - keeps data changing between transmissions
////                if (masterSendCount < 500) {
////                    masterSendCount++;
////                } else {
////                    masterSendCount = 1;
////                }
////            }
//        }
//        else {
//            Serial.print("[-] The transmission to the selected node failed.");
//        }
//    
//    Serial.println("--------------------------------------------------------");
// 
//}



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
//        data_receive = 1;
//        start_time = millis();
        wirelessDelay = time_of_flight - (time_of_flight/10)*difficulty;
        Serial.print("wirelessDelay = ");
        Serial.println(wirelessDelay);
        Process_Stage = 1;
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
    //will want to change this to atof (since PID input is double)
    Motor1_Speed = atoi(strtokIndx);     // convert this part to an integer
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    //will want to change this to atof (since PID input is double)
    Motor2_Speed = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    targetChoice = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    difficulty = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    ballFeed = atoi(strtokIndx);     // convert this part to an integer
    Motor_Start_Time = millis();
    
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    time_of_flight = atoi(strtokIndx);     // convert this part to an integer   
}



//=================

void showData() {
    if (new_data == true) {
        Serial.print("  ackData = ");
//        Serial.print(ackData[0]);
//        Serial.print(", ");
        Serial.println(ackData);
//        Serial.println();
        new_data = false;
    }
}

void showData2() {
    if (new_data2 == true) {
        Serial.print("  Ball Speed = ");
//        Serial.print(ackData[0]);
//        Serial.print(", ");
        Serial.println(targetBallSpeed);
//        Serial.println();
        new_data2 = false;
    }
}

//================

void updateMessage() {
        // so you can see that new data is being sent
    txNum += 1;
    if (txNum > '9') {
        txNum = '0';
    }
    dataToSend[8] = txNum;
}



//============

void showParsedData() {
    Serial.print("Motor 1 Speed = ");
    Serial.println(Motor1_Speed);
    Serial.print("Motor 2 Speed = ");
    Serial.println(Motor2_Speed);
    Serial.print("targetChoice = ");
    Serial.println(targetChoice);
    Serial.print("difficulty = ");
    Serial.println(difficulty);
    Serial.print("ballFeed = ");
    Serial.println(ballFeed);
    Serial.print("time_of_flight = ");
    Serial.println(time_of_flight);
}






//==================================================================//

void Serial_Output(){ 
   //Serial.print("<":distance:" ":temperature:" ":VR_Data:" ":timing:" ":ballSpeed:">");
   Serial.print('<');
   Serial.print(lidar_2_Distance);
   Serial.print(',');
   Serial.print(temperature);
   Serial.print(',');
   Serial.print(voiceCommand);   
   Serial.print(',');
   Serial.print(targetTiming);
   Serial.print(',');
   Serial.print(targetBallSpeed);
   Serial.println('>');

   //reset sent data
   lidar_2_Distance = 0;
   temperature = 0;
   voiceCommand = -1;
   targetTiming = 0;
   targetBallSpeed = 0;
}
   
//=============================== END ==============================//
