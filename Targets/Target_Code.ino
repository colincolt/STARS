#include <SPI.h>
#include <Wire.h>
#include <IRremote.h>

#define PIN_IR 9

#define SENSORPIN1 2 //Sensor 1 is on the front plane
#define SENSORPIN2 3 //Sensor 2 is on the back plane

//#define SENSORPIN3 12 //Sensor 3 is on the front top
//#define SENSORPIN4 8 //Sensor 4 is on the back top

#define REDPIN 4
#define GREENPIN 5
#define BLUEPIN 6

IRsend irsend;

volatile unsigned long Plane1_Break_Time = 0; // When plane 1 is triggered
volatile unsigned long Plane2_Break_Time = 0; // When plane 1 is triggered
volatile unsigned long Elapsed_Time = 0;      // Time it takes for ball to cross both planes

float ball_speed1 = 0.0; // Speed calculated
float ball_speed2 = 0.0; // Speed calculated

int Difficulty = 0;

long Base_Drill_Time = 10000; // Base time for a drill (5 seconds)
long Drill_Time;             // Total drill time which is a multiple of the base time

long Drill_Start_Time;
//long Drill_End_Time;




// nRF24L01 radio transceiver external libraries
//#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>

// chip select and RF24 radio setup pins
#define CE_PIN 7
#define CSN_PIN 10
RF24 radio(CE_PIN,CSN_PIN);

// set this to appropriate slave device number minus 1 (e.g. 0 for device 1)
#define NODE_ID 0 

// setup radio pipe addresses for each sensor node
const byte nodeAddresses[5][5] = {
    {'N','O','D','E','1'},
    {'N','O','D','E','2'},
    {'N','O','D','E','3'},
    {'N','O','D','E','4'},
    {'N','O','D','E','5'},
};

// simple integer array for each remote node data, in the form [node_id, returned_count]
//int remoteNodeData[3][2] = {{1, 1,}, {2, 1}, {3, 1}};

float remoteNodeData[4];

// integer to store count of successful transmissions
int Difficulty = 0;
float reset_data = -1.0;
int speed_poll = 0;



void setup() {


  // setup serial communications for basic program display
  Serial.begin(9600);
  Serial.println("[*][*][*] Beginning nRF24L01+ ack-payload slave device program [*][*][*]");

  // ----------------------------- RADIO SETUP CONFIGURATION AND SETTINGS -------------------------// 
  
  radio.begin();
  
  // set power level of the radio
  radio.setPALevel(RF24_PA_LOW);

  // set RF datarate
  radio.setDataRate(RF24_250KBPS);

  // set radio channel to use - ensure it matches the target host
  radio.setChannel(0x76);

  // open a reading pipe on the chosen address for selected node
  radio.openReadingPipe(1, nodeAddress[NODE_ID]);     

 radio.setAutoAck(false);

  // enable ack payload - slave reply with data using this feature
//  radio.enableAckPayload();

  // preload the payload with initial data - sent after an incoming message is read
//  radio.writeAckPayload(1, &remoteNodeData[NODE_ID], sizeof(remoteNodeData[NODE_ID]));

  // print radio config details to console
  printf_begin();
  radio.printDetails();

  // start listening on radio
 // radio.startListening();
  
  // --------------------------------------------------------------------------------------------//


  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  pinMode(SENSORPIN1, INPUT); // Sensor 1 as input
//  digitalWrite(SENSORPIN1, HIGH); // Turn on the pullup
  pinMode(SENSORPIN2, INPUT); // Sensor 2 as input
//  digitalWrite(SENSORPIN2, HIGH); // Turn on the pullup
  pinMode(SENSORPIN3, INPUT); // Sensor 2 as input
  pinMode(SENSORPIN4, INPUT); // Sensor 2 as input
//  Serial.begin(9600);
//  while (! Serial);
  Serial.println("Target 1 = 1; Target 2 = 2");
  irsend.enableIROut(38);
  irsend.mark(0);
}




void loop() {

radio.startListening();

// wait for an initialization signal from the master
while (Difficulty == 0){
  radioInitialize();

}


  
Drill_Start_Time = millis();
Drill_Time = (Base_Drill_Time) - (Difficulty)*(1000);

//Initialize the target
Initialize();


//Wait for ball to be detected and then buffer the speed value
while (Difficulty > 0){
  
  if (plane2_break_time > 0)
  {
  Ball_Speed();
  radio.writeAckPayload(1, &ball_speed1, sizeof(ball_speed1));
  
    while(ball_speed1 > 0.0)
    {
      Wireless();
    }

    delay(2000);
    //  deactivate target (RED)
  digitalWrite(REDPIN, HIGH);
  digitalWrite(GREENPIN, LOW); 
  digitalWrite(BLUEPIN, LOW); 
  }
  
  else{
    if ((millis()-Drill_Start_Time)>= Drill_Time){
      Reset();
    }
    else{
      }
  }
 Reset();
}
  
}




// Function to determine speed
void Ball_Speed()
{
  // subtract end time from start time to get total time
  Elapsed_Time = (Plane2_Break_Time - Plane1_Break_Time);

//  activate target (GREEN)
  digitalWrite(REDPIN, LOW);
  digitalWrite(GREENPIN, HIGH); 
  digitalWrite(BLUEPIN, LOW); 

  // convert mm/s to m/s
  ball_speed1 = ((224000.00 / Elapsed_Time));
  ball_speed2 = ball_speed1*3.6;

    Serial.println("GOAL!");
    Serial.print("Ball Speed (m/s) = ");
    Serial.println(ball_speed1);
    Serial.print("Ball Speed (km/h) = ");
    Serial.println(ball_speed2);
    Serial.println();
}


void Initialize()
{
   EIFR |= (1 << INTF0);    // clear any outstanding interrupt 0
   EIFR |= (1 << INTF1);    // clear any outstanding interrupt 1
   
//  attach interrupts
  attachInterrupt(digitalPinToInterrupt(SENSORPIN_1), Plane1_Break_Time_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSORPIN_2), Plane2_Break_Time_ISR, RISING);

  
//  activate target (BLUE)
  digitalWrite(REDPIN, LOW);
  digitalWrite(GREENPIN, LOW); 
  digitalWrite(BLUEPIN, HIGH); 

  //  set autoack True
  radio.setAutoAck(true);

  // enable ack payload - slave reply with data using this feature
  radio.enableAckPayload();
 
}



void Reset()
{
//  detach interrupts

  detachInterrupt(digitalPinToInterrupt(SENSORPIN_1));
  detachInterrupt(digitalPinToInterrupt(SENSORPIN_2));
  
//  deactivate target (RED)

  digitalWrite(REDPIN, HIGH);
  digitalWrite(GREENPIN, LOW); 
  digitalWrite(BLUEPIN, LOW); 
  
//  set autoack(false)

  radio.setAutoAck(false);

// Reset variables
  Difficulty = 0;
  Plane1_Break_Time = 0;
  Plane2_Break_Time = 0;
  ball_speed1 = 0.0;
  ball_speed2 = 0.0;
//  Elapsed_Time = 0;

  
}



/* Function: loop
 *    main loop program for the slave node - repeats continuously during system operation
 */
void Wireless() {
  
  // transmit current preloaded data to master device if message request received
  radioCheckAndReply();
}

/* Function: updateNodeData
 *    updates the count variable for the node and stores it in the nRF24L01+ radio
 *    preloaded ack payload ready for sending on next received message
 */
void updateNodeData(void) 
{
//  // increment node count - reset to 1 if exceeds 500
//  if (remoteNodeData[NODE_ID][1] < 500) {
//    remoteNodeData[NODE_ID][1]++;
//  } else {
//    remoteNodeData[NODE_ID][1] = 1;
//  }

  // set the ack payload ready for next request from the master device
  radio.writeAckPayload(1, &reset_data, sizeof(reset_data));
}


/* Function: radioCheckAndReply
 *    sends the preloaded node data over the nrf24l01+ radio when
 *    a message is received by the master
 */

 
void radioCheckAndReply(void)
{
    // check for radio message and send sensor data using auto-ack
    if ( radio.available() ) {
          radio.read( &speed_poll, sizeof(speed_poll));
          Serial.println("Received request from master - sending preloaded data.");
          Serial.print("The received signal from the master was: ");
          Serial.println(speed_poll);
          Serial.println("--------------------------------------------------------");

          radio.stopListening();
          // update the node count after sending ack payload - provides continually changing data
          
//          Difficulty = 0;
          
          // update the node count after sending ack payload - provides continually changing data
//          updateNodeData();
    }
}



void radioInitialize(void)
{
    // check for radio message and send sensor data using auto-ack
    if ( radio.available() ) {
          radio.read( &Difficulty, sizeof(Difficulty) );
          Serial.println("Received request from master - sending preloaded data.");
          Serial.print("The received signal from the master was: ");
          Serial.println(Difficulty);
          Serial.println("--------------------------------------------------------");
//
//          radio.stopListening();
//          // update the node count after sending ack payload - provides continually changing data
//          
//          Difficulty = 0;
//          
//          // update the node count after sending ack payload - provides continually changing data
//          updateNodeData();
    }
}


void Plane1_Break_Time_ISR()
{
  Plane1_Break_Time = micros();
}


void Plane2_Break_Time_ISR()
{
  Plane2_Break_Time = micros();
}
