/********************************************************************/
// ---------------------------- WIRELESS -------------------------- //

/*************************************************************************
 * Remote node - nRF24L01+ radio communications                          *
 *      A program to operate a remote-node slave device that sends       *
 *      data to a command unit on a given request message. The radio     *            
 *      transceiver used is the nRF24L01+, and it operates using the     *
 *      TMRh20 RF24 library.                                             *
 *                                                                       *
 *      Author: B.D. Fraser                                              *
 *                                                                       *
 *        Last modified: 27/06/2017                                      *
 *                                                                       *
 *************************************************************************/

// nRF24L01 radio transceiver external libraries
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>

// chip select and RF24 radio setup pins
#define CE_PIN 7
#define CSN_PIN 10
RF24 radio(CE_PIN,CSN_PIN);

// set this to appropriate slave device number minus 1 (e.g. 0 for device 1)
#define NODE_ID 4 

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

//float remoteNodeData[4];



/* Function: setup
 *    Initialises the system wide configuration and settings prior to start
 */


/********************************************************************/
// ---------------------------- EASY_VR --------------------------- //

#include "Arduino.h"
#if !defined(SERIAL_PORT_MONITOR)
  #error "Arduino version not supported. Please update your IDE to the latest version."
#endif

#if defined(__SAMD21G18A__)
  // Shield Jumper on HW (for Zero, use Programming Port)
  #define port SERIAL_PORT_HARDWARE
  #define pcSerial SERIAL_PORT_MONITOR
#elif defined(SERIAL_PORT_USBVIRTUAL)
  // Shield Jumper on HW (for Leonardo and Due, use Native Port)
  #define port SERIAL_PORT_HARDWARE
  #define pcSerial SERIAL_PORT_USBVIRTUAL
#else
  // Shield Jumper on SW (using pins 12/13 or 8/9 as RX/TX)
  #include "SoftwareSerial.h"
  SoftwareSerial port(8, 9);
  #define pcSerial SERIAL_PORT_MONITOR
#endif

#include "EasyVR.h"

EasyVR easyvr(port);

//Groups and Commands
enum Groups
{
  GROUP_0  = 0,
  GROUP_1  = 1,
};

enum Group0 
{
  G0_STARS = 0,
};

enum Group1 
{
  G1_INCREASE_SPEED = 0,
  G1_REDUCE_SPEED = 1,
  G1_PAUSE = 2,
  G1_BEGIN = 3,
  G1_CHANGE_DRILL = 4,
  G1_MANUAL = 5,
};

//Grammars and Words
enum Wordsets
{
  SET_1  = -1,
  SET_2  = -2,
  SET_3  = -3,
};

enum Wordset1 
{
  S1_ACTION = 0,
  S1_MOVE = 1,
  S1_TURN = 2,
  S1_RUN = 3,
  S1_LOOK = 4,
  S1_ATTACK = 5,
  S1_STOP = 6,
  S1_HELLO = 7,
};

enum Wordset2 
{
  S2_LEFT = 0,
  S2_RIGHT = 1,
  S2_UP = 2,
  S2_DOWN = 3,
  S2_FORWARD = 4,
  S2_BACKWARD = 5,
};

enum Wordset3 
{
  S3_ZERO = 0,
  S3_ONE = 1,
  S3_TWO = 2,
  S3_THREE = 3,
  S3_FOUR = 4,
  S3_FIVE = 5,
  S3_SIX = 6,
  S3_SEVEN = 7,
  S3_EIGHT = 8,
  S3_NINE = 9,
  S3_TEN = 10,
};


// use negative group for wordsets
int8_t group, idx;
int VR_command = 0;

// integer to store count of successful transmissions
int dataFromMaster = 0;

int buffered;

unsigned long current_time;





 
void setup() {

/********************************************************************/
// ---------------------------- EASY_VR --------------------------- //

  // setup PC serial port
  pcSerial.begin(9600);
bridge:
  // bridge mode?
  int mode = easyvr.bridgeRequested(pcSerial);
  switch (mode)
  {
  case EasyVR::BRIDGE_NONE:
    // setup EasyVR serial port
    port.begin(9600);
    // run normally
    pcSerial.println(F("Bridge not requested, run normally"));
    pcSerial.println(F("---"));
    break;
    
  case EasyVR::BRIDGE_NORMAL:
    // setup EasyVR serial port (low speed)
    port.begin(9600);
    // soft-connect the two serial ports (PC and EasyVR)
    easyvr.bridgeLoop(pcSerial);
    // resume normally if aborted
    pcSerial.println(F("Bridge connection aborted"));
    pcSerial.println(F("---"));
    break;
    
  case EasyVR::BRIDGE_BOOT:
    // setup EasyVR serial port (high speed)
    port.begin(115200);
    pcSerial.end();
    pcSerial.begin(115200);
    // soft-connect the two serial ports (PC and EasyVR)
    easyvr.bridgeLoop(pcSerial);
    // resume normally if aborted
    pcSerial.println(F("Bridge connection aborted"));
    pcSerial.println(F("---"));
    break;
  }

  // initialize EasyVR  
  while (!easyvr.detect())
  {
    pcSerial.println(F("EasyVR not detected!"));
    for (int i = 0; i < 10; ++i)
    {
      if (pcSerial.read() == '?')
        goto bridge;
      delay(100);
    }
  }

  pcSerial.print(F("EasyVR detected, version "));
  pcSerial.print(easyvr.getID());

  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, LOW); // Shield 2.0 LED off

  if (easyvr.getID() < EasyVR::EASYVR)
    pcSerial.print(F(" = VRbot module"));
  else if (easyvr.getID() < EasyVR::EASYVR2)
    pcSerial.print(F(" = EasyVR module"));
  else if (easyvr.getID() < EasyVR::EASYVR3)
    pcSerial.print(F(" = EasyVR 2 module"));
  else
    pcSerial.print(F(" = EasyVR 3 module"));
  pcSerial.print(F(", FW Rev."));
  pcSerial.println(easyvr.getID() & 7);

  easyvr.setDelay(0); // speed-up replies

  easyvr.setTimeout(5);
  easyvr.setLanguage(0); //<-- same language set on EasyVR Commander when code was generated

//  group = EasyVR::TRIGGER; //<-- start group (customize)
  group = GROUP_0;



/********************************************************************/
// ---------------------------- WIRELESS -------------------------- //

//  pinMode(10, OUTPUT);           // set pin to input

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
  radio.openReadingPipe(1, nodeAddresses[NODE_ID]);     

  // enable ack payload - slave reply with data using this feature
  radio.enableAckPayload();

  // preload the payload with initial data - sent after an incoming message is read
  radio.writeAckPayload(1, &VR_command, sizeof(VR_command));

  // print radio config details to console
  printf_begin();
  radio.printDetails();

  // start listening on radio
//  radio.startListening();
  
}




/********************************************************************/



void loop() 
{

if (VR_command == 0)
{
  Easy_VR();

}

else
{
    if (buffered == 0)
    {
      radio.startListening();
      radio.writeAckPayload(1, &VR_command, sizeof(VR_command));
      buffered = 1;
    }
    Wireless();
    
}

}





/********************************************************************/
// ---------------------------- WIRELESS -------------------------- //


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
//  radio.writeAckPayload(1, &reset_data, sizeof(reset_data));

  radio.writeAckPayload(1, &VR_command, sizeof(VR_command));
}


/* Function: radioCheckAndReply
 *    sends the preloaded node data over the nrf24l01+ radio when
 *    a message is received by the master
 */
void radioCheckAndReply(void)
{
    // check for radio message and send sensor data using auto-ack
    if ( radio.available() ) {
          radio.read( &dataFromMaster, sizeof(dataFromMaster) );
//          current_time = millis();
          Serial.println("Received request from master - sending preloaded data.");
          Serial.print("The received signal from the master was: ");
          Serial.println(dataFromMaster);
          Serial.println("--------------------------------------------------------");
          Serial.println(VR_command);
      
          radio.stopListening();
          // update the node count after sending ack payload - provides continually changing data
          VR_command = 0;
          buffered = 0;
          updateNodeData();
    }
}



/********************************************************************/
// ---------------------------- EASY_VR --------------------------- //

void Easy_VR()
{
  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, HIGH); // LED on (listening)

  if (group < 0) // SI wordset/grammar
  {
    pcSerial.print("Say a word in Wordset ");
    pcSerial.println(-group);
    easyvr.recognizeWord(-group);
  }
  else // SD group
  {
   // pcSerial.print("Say a command in Group ");  //Can comment out here if we want
    pcSerial.println(group);
    easyvr.recognizeCommand(group);
  }

  do
  {
    // allows Commander to request bridge on Zero (may interfere with user protocol)
    if (pcSerial.read() == '?')
    {
      setup();
      return;
    }
    // <<-- can do some processing here, while the module is busy
  }
  while (!easyvr.hasFinished());
  
  if (easyvr.getID() < EasyVR::EASYVR3)
    easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off

  idx = easyvr.getWord();
//  if (idx == 0 && group == EasyVR::TRIGGER)
  if (idx == 0 && group == GROUP_0)
  {
    // beep
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // print debug message
 //   pcSerial.println("Word: STARS");  // Prints STARs to show recognition, changes to G1
    // write your action code here
    group = GROUP_1; // <-- jump to another group or wordset
    return;
  }
  else if (idx >= 0)
  {
    // beep
    easyvr.playSound(1, EasyVR::VOL_FULL);
    // print debug message
    uint8_t flags = 0, num = 0;
    char name[32];
    pcSerial.print("Word: ");
    pcSerial.print(idx);
    if (easyvr.dumpGrammar(-group, flags, num))
    {
      for (uint8_t pos = 0; pos < num; ++pos)
      {
        if (!easyvr.getNextWordLabel(name))
          break;
        if (pos != idx)
          continue;
  //      pcSerial.print(F(" = "));
  //      pcSerial.println(name);
        break;
      }
    }
    // perform some action
//    action();
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    // beep
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // print debug message
    uint8_t train = 0;
    char name[32];
//    pcSerial.print("Command: "); //prints identifier upon recognition
//    pcSerial.print(idx);           // prints numeric value for command
    if (easyvr.dumpCommand(group, idx, name, train))
    {
     // pcSerial.print(" = ");      // THIS IF STATEMENT SHOULD SOLELY BE RESPONSIBLE FOR 
     // pcSerial.println(name);     // OUTPUTTING ADDITIONAL DESCRIPTIONS TO SERIAL UPON
    }                             // RECOGNITION. THEREFORE UNNECESSARY
    else
      pcSerial.println();
    // perform some action
    action();
  }
  else // errors or timeout
  {
    if (easyvr.isTimeout())
      pcSerial.println("Timed out, try again...");
    int16_t err = easyvr.getError();
    if (err >= 0)
    {
      pcSerial.print("Error ");
      pcSerial.println(err, HEX);
      group = GROUP_0;
    }
  }
}

void action()
{
  switch (group)
  {
  case GROUP_0:
    switch (idx)
    {
    case G0_STARS:
      // write your action code here
      group = GROUP_1;
      // group = GROUP_X\SET_X; <-- or jump to another group or wordset for composite commands
      break;
    }
    break;
  case GROUP_1:
    switch (idx)
    {
    case G1_INCREASE_SPEED:
      pcSerial.print("1");
      VR_command = 1;
      pcSerial.print(VR_command);
      group = GROUP_0;
      break;
    case G1_REDUCE_SPEED:
      pcSerial.print("2");
      VR_command = 2;
      group = GROUP_0;
      break;
    case G1_PAUSE:
      pcSerial.print("3");
      VR_command = 3;
      group = GROUP_0;
      break;
    case G1_BEGIN:
      pcSerial.print("4");
      VR_command = 4;
      group = GROUP_0;
      break;
    case G1_CHANGE_DRILL:
      pcSerial.print("5");
      VR_command = 5;
      group = GROUP_0;
      break;
    case G1_MANUAL:
      pcSerial.print("6");
      VR_command = 6;
      group = GROUP_0;
      break;
    }
    break;
  }
}
