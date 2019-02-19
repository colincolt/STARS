
  // put your setup code here, to run once:
 int In1= 8;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In2 = 7;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA = 9;// Pin 5(Uno) is connected to the motor controller ENA pin
 int angle = 0;
 int Speed = 0;
 int Barrier =24690;
 int Barrierend = -24690;
 long encodercount = 0;
 long oldposition = 0;
 long newposition = 0;
 int Direction = 0;
 const byte numChars = 32;
 char receivedChars[numChars];   
 boolean newData = false;
 int dataNumber = 0;
 
 int num = 0;
//  Encoder myEnc(2,3);//yellow 3 brown 2, also power red out1 black out2

void setup() {
 

  pinMode(In1,OUTPUT);// Pin 7 is an output 
  pinMode(In2,OUTPUT);// Pin 8 is an output
  pinMode(ENA,OUTPUT);// Pin 5 is an output
                     
 
  
  Serial.begin(9600);// Window Prompt
  while(!Serial);
  //Serial.println("angle of rotation (must be a value between 0˚ and 160˚)");// Asks user for angle of rotation value
 


  digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
  digitalWrite(In2,LOW);
}

void loop()
{
 recvWithEndMarker();
 showNewNumber();
}
// dir();

////void dir(){
//  // put your main code here, to run repeatedly:
// if (Serial.available()){
//  //int Direction = Serial.read() - '0';
// 
//  ///Serial.println("l");
// // if (Serial.available()){
////Serial.println("l2");
//  int Direction = Serial.parseInt();
////Serial.println(Direction);
//  //} 
// if (Direction == -49)
// {
//  Direction = 0;
//  Serial.println(Direction);
// }
// 
// if (Direction >0)
//      {
//        //Serial.println("positive");
//        digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
//        digitalWrite(In2,LOW);
//      }
// if (Direction<0)
//      {
//        //Serial.println("negative");
//        //Serial.println(abs(angle));
//        digitalWrite(In1,LOW);// Counter-Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
//        digitalWrite(In2,HIGH);
//      }
//
//      Speed = abs(Direction)*10;
//      Serial.println(Speed);
//      analogWrite(ENA,Speed);
//}}



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
        }
    }
}


void showNewNumber() {
    if (newData == true) {
        dataNumber = 0;             // new for this version
        dataNumber = atoi(receivedChars);   // new for this version
        Direction = dataNumber;
            // new for this version
            // new for this version
        newData = false;
        if (Direction >0)
      {
        //Serial.println("positive");
        digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,LOW);
      }
 if (Direction<0)
      {
        //Serial.println("negative");
        //Serial.println(abs(angle));
        digitalWrite(In1,LOW);// Counter-Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,HIGH);
      }

      Speed = abs(Direction);
      analogWrite(ENA,Speed);
    }
}
