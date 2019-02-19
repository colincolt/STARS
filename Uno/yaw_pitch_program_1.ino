
  // put your setup code here, to run once:
 int In1= 8;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In2 = 7;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA = 9;// Pin 5(Uno) is connected to the motor controller ENA pin
 int angle = 0;
 int Speed = 0;
 int Direction = 0;
 const byte numChars = 32;
 char receivedChars[numChars];   
 boolean newData = false;
 int dataNumber = 0;
//  Encoder myEnc(2,3);//yellow 3 brown 2, also power red out1 black out2
 char tempChars[numChars]; 
 int Motor1_Speed = 0;
 int Motor2_Speed = 0;
//***PITCH MOTOR CONTROL PINS
 int In3= 12;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In4 = 13;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA2 = 11;// Pin 5(Uno) is connected to the motor controller ENA pin
 int ref1 = A0;//Pin A0(Uno) is connected to the linear actuator position signal
 int last1 = 0;
 int Cposition = 0;

void setup() {
 

  pinMode(In1,OUTPUT);// Pin 7 is an output 
  pinMode(In2,OUTPUT);// Pin 8 is an output
  pinMode(ENA,OUTPUT);// Pin 5 is an output
                     
  pinMode(In3,OUTPUT);// Pin 7 is an output 
  pinMode(In4,OUTPUT);// Pin 8 is an output
  pinMode(ENA2,OUTPUT);// Pin 5 is an output
  pinMode(ref1,INPUT); //pin A0 is an inqput
  
  
  Serial.begin(9600);// Window Prompt
  while(!Serial);
  //Serial.println("angle of rotation (must be a value between 0˚ and 160˚)");// Asks user for angle of rotation value
 


  digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
  digitalWrite(In2,LOW);
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
    parseData();
 //   showParsedData();
    newData = false;
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
    Serial.println("tempchars:");
    Serial.println(tempChars);
    strtokIndx = strtok(tempChars,",");
    // get the first part - the string
    Motor1_Speed = atoi(strtokIndx);     // convert this part to an integer
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Motor2_Speed = atoi(strtokIndx);     // convert this part to an integer
    Serial.println("ms1");
    Serial.println(Motor1_Speed);
    Serial.println("ms2");
    Serial.println(Motor2_Speed);
    if (Motor1_Speed >0)
      {
        //Serial.println("positive");
        digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,LOW);
      }
    if (Motor1_Speed<0)
      {
        //Serial.println("negative");
        //Serial.println(abs(angle));
        digitalWrite(In1,LOW);// Counter-Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,HIGH);
      }

    Speed = abs(Motor1_Speed);
    analogWrite(ENA,Speed);

    int desiredpos= ((abs(Motor2_Speed)-1)/45.00)*(933)+48;
 
    
    if (abs(Motor2_Speed) > 0 && abs(Motor2_Speed) <=46)
  {
      if (Motor2_Speed == 10)
    {
        Cposition=analogRead(ref1);
        if (Cposition > 753)
      {
          angle = 35;
      }
        if (Cposition < 753)
      {
          angle = -35;
      }
    }
      if (Motor2_Speed == 20)
    {
        Cposition=analogRead(ref1);
        if (Cposition > 546)
      {
          angle = 25;
      }
        if (Cposition < 546)
      {
          angle = -25;
      }
    }
          if (Motor2_Speed == 30)
    {
        Cposition=analogRead(ref1);
        if (Cposition > 338)
      {
          angle = 15;
      }
        if (Cposition < 338)
      {
          angle = -15;
      }
    }
          if (Motor2_Speed == 40)
    {
        Cposition=analogRead(ref1);
        if (Cposition > 131)
      {
          angle = 5;
      }
        if (Cposition < 131)
      {
          angle = -5;
      }
    }
      if (Motor2_Speed == 45)
    {
      Cposition=analogRead(ref1);
        if (Cposition > 40)
      {
          angle = 1;
      }
     
    }
  }
  desiredpos= ((abs(angle)-1)/45.00)*(933)+48;
  if (last1 == Motor2_Speed)
  {
    digitalWrite(In3,HIGH);// Retract, Pin 7 outputing HIGH and pin 8 outputing LOW
    digitalWrite(In4,LOW);
    int Speed = 0;
    analogWrite(ENA2,Speed);
  }
  else if (angle >0)
      {
        
        
        digitalWrite(In3,HIGH);// Retract, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In4,LOW);
        int Speed = 255;
        analogWrite(ENA2,Speed);
      
        
        Cposition=analogRead(ref1);
        //Serial.println(Cposition);
       // Serial.println(desiredpos);
   
        
        if (Cposition<=desiredpos)
        {
        Cposition=analogRead(ref1);  
        //  Serial.println(Cposition);
        
        Speed = 0;
        analogWrite(ENA2,Speed);
        last1 = Motor2_Speed;
        }
        
       // Speed = 0;
       // analogWrite(ENA2,Speed);
       // Serial.println("done");
      }
  else if (angle <0)
      {
      
        
        
        digitalWrite(In3,LOW);// Extend, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In4,HIGH);
        
        int Speed = 255;
        analogWrite(ENA2,Speed);
       
        Cposition=analogRead(ref1);
       // Serial.println(Cposition);
        //Serial.println(desiredpos);
        
       
        if (Cposition>=desiredpos)
         {
          Cposition=analogRead(ref1);  
          Speed = 0;
          analogWrite(ENA2,Speed);
          last1 = Motor2_Speed;
       //   Serial.println(Cposition);
         }
//          Speed = 0;
          //analogWrite(ENA2,Speed);
         // Serial.println("done");
//        }
//       
//        
  }  
    }
//}
//void loop()
//{
// recvWithEndMarker();
// showNewNumber();
//}
//
//  void recvWithEndMarker() {
//    static byte ndx = 0;
//    char endMarker = '\n';
//    char rc;
//    
//    if (Serial.available() > 0) {
//        rc = Serial.read();
//       
//        if (rc != endMarker) {
//            receivedChars[ndx] = rc;
//            ndx++;
//            if (ndx >= numChars) {
//                ndx = numChars - 1;
//                
//            }
//        }
//        else {
//            receivedChars[ndx] = '\0'; // terminate the string
//            ndx = 0;
//            newData = true;
//        }
//    }
//}
//
//
//void showNewNumber() {
//    if (newData == true) {
//        dataNumber = 0;             // new for this version
//        dataNumber = atoi(receivedChars);   // new for this version
//        Direction = dataNumber;
//            // new for this version
//            // new for this version
//        newData = false;
//        if (Direction >0)
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
//      Speed = abs(Direction);
//      analogWrite(ENA,Speed);
//    }
//}
