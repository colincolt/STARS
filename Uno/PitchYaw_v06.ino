// LIBRARIES
 #include <Encoder.h>
 #include <PID_v1.h>
 #include <Wire.h>
 #include <MPU6050.h>//***YAW MOTOR CONTROL PINS 

MPU6050 mpu;

 int In1= 8;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In2 = 7;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA = 9;// Pin 5(Uno) is connected to the motor controller ENA pin
 
//***YAW MOTOR VARIABLES
 int angle = 0;
 int Stop = 0;
 int oldposition = 0;
 int Barrier = 7000;
 //int Barrierend = -14000;
// long encodercount = 0;
 long EncoderCount = 0;
 int Dir;
  int Yaw_Speed = 0;
 int Yaw_PWM;
 double Setpoint_2 = 0;
 double Input_2;
 double Output_2;
 Encoder myEnc(2,3);//2 - Brown; 3 - Orange , also power red out1 black out2
 PID myPID_Yaw(&Input_2, &Output_2, &Setpoint_2,0.01,0,0,DIRECT);
 
//***SERIAL COMMUNICATION VARIABLES
 const byte numChars = 32;
 char receivedChars[numChars];   
 boolean newData = false;
 int dataNumber = 0;
 char tempChars[numChars]; 

 
//***PITCH MOTOR CONTROL PINS
 int In3= 12;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In4 = 13;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA2 = 11;// Pin 5(Uno) is connected to the motor controller ENA pin
 int ref1 = A0;//Pin A0(Uno) is connected to the linear actuator position signal(blue)
 int last1 = 0;
 int Cposition = 0;
 int Pitch_Angle = 0;
 int angle1= 0;
 int desiredpos = 0;
 int Speed;
 
//***PID VARIABLES
 double Setpoint;
 double Input;
 double Output;
 PID myPID(&Input, &Output, &Setpoint,5,0,0,DIRECT);

//***ACCELEROMETER VARIABLES
 long accelX, accelY, accelZ;
 float gForceX, gForceY, gForceZ;
 int i = 0;
 int sum = 0;
 int offset = 0; //offset angle

//***Timing
long displayInterval = 2000;
long prevDisplayMillis = 0;



void setup() {
//Time = millis();
//***OUTPUT PINS
//*******Yaw motor output pins
  pinMode(In1,OUTPUT);// Pin 7 is an output 
  pinMode(In2,OUTPUT);// Pin 8 is an output
  pinMode(ENA,OUTPUT);// Pin 5 is an output
  
//*******Pitch motor output pins                    
  pinMode(In3,OUTPUT);// Pin 7 is an output 
  pinMode(In4,OUTPUT);// Pin 8 is an output
  pinMode(ENA2,OUTPUT);// Pin 5 is an output
  
//***INPUT PINS
//*******Pitch motor input pin
  pinMode(ref1,INPUT); //pin A0 is an input
  
//***SERIAL COMMUNICATIONS
    Serial.begin(115200);
    Serial.println("This demo expects 2 pieces of data - Motor 1 Speed, Motor 2 Speed");
    Serial.println("Enter data in this style <2000,3000>  ");
    Serial.println();
    
 // Wire.begin();
//setupMPU();
  
//***YAW MOTOR COUNTER-CLOCKWISE ROTATION INITIALIZATION 
  digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
  digitalWrite(In2,LOW);

  myPID.SetMode(AUTOMATIC);
  myPID_Yaw.SetMode(AUTOMATIC);
//  Time = millis();
// while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
//  {
//    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
//    delay(500);
//  }
}


void loop() {
//*** SERIAL COMMUNICATION (turning serial input into integer)
  //Serial.println("beginning of loop");
  recvWithStartEndMarkers();

  if (newData == true) 
  {
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;
  }
  
//   Serial.println("data has been split");

 if (Yaw_Speed <= 255)
 {
  yawMotor();
 }
  
 else if (Yaw_Speed == 300)
 {
  Reset();
 }
 
 
 if (millis() - prevDisplayMillis >= displayInterval) 
 {
       prevDisplayMillis += displayInterval;
       //recordAccelRegisters();
//       Serial.println("stuck before accelerometer loop");
//       accelerometerData();
     Serial.print("Encoder Count :  ");
     Serial.println(EncoderCount);
//     Reset();
     Serial.print("Setpoint 2 : ");
     Serial.println(Setpoint_2);
     Serial.print("Input 2 : ");
     Serial.println(Input_2);    
     Serial.print("Output 2 : ");
     Serial.println(Output_2);  
     Serial.print("Yaw Speed : ");
     Serial.println(Yaw_Speed); 
     Serial.print("Yaw PWM : ");
     Serial.println(Yaw_PWM); 
     Serial.println();

  }
  
  pitchMotor();
   
}



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
                    ndx = numChars - 1;}}
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;}}
        else if (rc == startMarker) {
            recvInProgress = true;}}}


            
void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(tempChars,",");// get the first part - the string
    Yaw_Speed = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Pitch_Angle = atoi(strtokIndx);     // convert this part to an integer
//*** PITCH and YAW INPUTS DEFINED FROM PI (Yaw_Speed, Pitch_Angle)
//    Serial.print("Yaw Speed : ");
//    Serial.println(Yaw_Speed);
//    Serial.print("Pitch Angle : ");
//    Serial.println(Pitch_Angle);
//    Serial.println();
}



void yawMotor(){
 //********* YAW SECTION
     EncoderCount = (myEnc.read());// number of countable events seen\
    
     int player_angle = EncoderCount/ ((2.96011)*(49.1684)); // players angle from orgin (zero position) 

 //*** DIRECTION OF YAW
     if (Yaw_Speed > 0){
         digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
         digitalWrite(In2,LOW);
         Dir = 0;
     }
     if (Yaw_Speed < 0){
         digitalWrite(In1,LOW);// Counter-Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
         digitalWrite(In2,HIGH);
         Dir = 1;
     }

     if (abs(EncoderCount) < Barrier) {
      
     //*** SPEED OF YAW
         Speed = abs(Yaw_Speed);
         analogWrite(ENA,Speed);
     }

     else if (EncoderCount >= Barrier){
      if(Dir == 0){
//        Speed = 0;
        analogWrite(ENA,Stop);
      }
      else{
        analogWrite(ENA,Yaw_Speed);
      }
     }  

     else if (EncoderCount <= -Barrier){
      if(Dir == 1){
//        Speed = 0;
        analogWrite(ENA,Stop);
      }
      else{
        analogWrite(ENA,Yaw_Speed);
      }
     } 

}



 void Reset()
 {
//*** RESET FUNCTION

     EncoderCount = (myEnc.read());      
//      Input_2 = -500;


     
//********ROTATION AMOUNT FROM RIGHT OF CENTRE POSITION->EncoderCount=0 (CLOCKWISE ROTATION )
        if (EncoderCount <= Setpoint_2)
        {
//           Input_2 = -Input_2;
           Input_2 = EncoderCount;
           myPID_Yaw.Compute();
//           Yaw_PWM = Output_2;
//           Serial.print("Output 2 : ");
//           Serial.println(Output_2);
           digitalWrite(In1,HIGH);// Pin 7 outputing HIGH and pin 8 outputing LOW
           digitalWrite(In2,LOW);
           Dir = 0;
           Yaw_PWM = map(Output_2, 0, 255, 80 , 255);
           if (Yaw_PWM <= 80){
           Yaw_PWM = 0; 
           }
//           analogWrite(ENA,Yaw_PWM);//initialize speed
        }


//********ROTATION AMOUNT FROM LEFT OF CENTRE POSITION->EncoderCount=0 (COUNTER-CLOCKWISE ROTATION )
        if (EncoderCount > Setpoint_2)
        {
           Input_2 = -EncoderCount;
           myPID_Yaw.Compute();
//           Yaw_PWM = Output_2;
//           Serial.print("Output 2 : ");
//           Serial.println(Output_2);           
           digitalWrite(In1,LOW);// Pin 7 outputing HIGH and pin 8 outputing LOW
           digitalWrite(In2,HIGH);
           Dir = 1;
           Yaw_PWM = map(Output_2, 0, 255, 80 , 255);
           if (Yaw_PWM <= 80){
           Yaw_PWM = 0; 
           }         
//           analogWrite(ENA,Yaw_PWM);//initialize speed
        }

     if (abs(EncoderCount) < Barrier) {

         analogWrite(ENA,Yaw_PWM);
     }

     else if (EncoderCount >= Barrier){
      if(Dir == 0){
        analogWrite(ENA,Stop);
      }
      else{
        analogWrite(ENA,Yaw_PWM);
      }
     }  

     else if (EncoderCount <= -Barrier){
      if(Dir == 1){
        analogWrite(ENA,Stop);
      }
      else{
        analogWrite(ENA,Yaw_PWM);
      }
     }      

 }
 
void pitchMotor()
{ 
 //*********PITCH SECTION
 //***ANGLE DEFINED
 //*** potentiometer value of the desired position for a given Pitch_Angle
     int desiredpos = 981.00 - 933.00*((Pitch_Angle)/45.00);
 //*** current position of the pitch (potentiometer reading)
     Cposition = analogRead(ref1);
     

     Setpoint = desiredpos;
     Input = Cposition;
     
//     if (Cposition > Setpoint){
//        int adjustment = 2*(Cposition - Setpoint);
//        Setpoint = Setpoint + adjustment;}

//     myPID.Compute();
//     Speed = map(Output, 0, 255, 80 , 255);
//     if (Speed <= 80){
//      Speed = 0; 
//     }


//***DIRECTION OF PITCH 
     if (Input >= Setpoint) // IF Current position is lower than desired position then lin_act needs to extend 
     {
      digitalWrite(In3,LOW);// retact, Pin 7 outputing HIGH and pin 8 outputing LOW
      digitalWrite(In4,HIGH);

      Setpoint = -Setpoint;
      Input = -Input;

      myPID.Compute();
      Speed = Output;
//      Speed = map(Output, 0, 255, 80 , 255);
      if (Speed <= 80){
      Speed = 0; 
      }

      analogWrite(ENA2,Speed);  
//      Serial.println("retracting");
     }
      
     else if (Input < Setpoint) // IF Current position is higher than desired position then lin_act needs to retract 
     {
      digitalWrite(In3,HIGH);// extend, Pin 7 outputing HIGH and pin 8 outputing LOW
      digitalWrite(In4,LOW);

      myPID.Compute();
      Speed = Output;
//      Speed = map(Output, 0, 255, 80 , 255);
      if (Speed <= 80){
      Speed = 0; 
      }
      
      analogWrite(ENA2,Speed); 
//      Serial.println("extending");
     }
     

//       Serial.print("Setpoint : ");
//       Serial.println(desiredpos);
//       Serial.print("Input : ");
//       Serial.println(Cposition);    
//       Serial.print("Output : ");
//       Serial.println(Output);  
//       Serial.print("Speed : ");
//       Serial.println(Speed); 
//       Serial.print("Desired Pos : ");
//       Serial.println(desiredpos); 
//       Serial.println(); 
     
//     digitalWrite(In3,LOW);// Extend, Pin 7 outputing HIGH and pin 8 outputing LOW
//     digitalWrite(In4,HIGH);
//     int val = map(Output,0,255,0,255);
     //analogWrite(ENA2,Output);
     

 }


 
void accelerometerData()
 {
   // Read normalized values 
   Vector normAccel = mpu.readNormalizeAccel();

   // Calculate Pitch & Roll
   offset = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
   //int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

   // Output
//   Serial.print(" Pitch offset = ");
//   Serial.print(offset);

//   Serial.println();

   delay(10);
}




















//***DIRECTION OF PITCH 
//     if (Cposition < oldposition)
//     {
//       angle = (-1)*angle;}
// //***PITCH WILL NOT MOVE IF IN SAME POSITION AS DESIRED POSITION
//   if (last1 == Pitch_Angle)
//   {
//     digitalWrite(In3,HIGH);// Retract, Pin 7 outputing HIGH and pin 8 outputing LOW
//     digitalWrite(In4,LOW);
//     int Speed = 0;
//     analogWrite(ENA2,Speed);}
// //***
//   else if (angle >0)
//       {
//         digitalWrite(In3,HIGH);// Retract, Pin 7 outputing HIGH and pin 8 outputing LOW
//         digitalWrite(In4,LOW);
//         int Speed = 255;
//         analogWrite(ENA2,Speed);//Output);
//         Cposition=analogRead(ref1);
//         oldposition = Cposition;
//         Serial.println("runnning2 ");
//         //Input = analogRead(ref1);
//        // Setpoint = desiredpos; 
//        // myPID.SetMode(AUTOMATIC);
   
// //***PITCH IS ON OR JUST PAST DESIRED POSITION(STOP)        
//         if (Cposition<=desiredpos)
//         {
//         Cposition=analogRead(ref1);  
//         Speed = 0;
//         analogWrite(ENA2,Speed);
//         last1 = Pitch_Angle;
//         oldposition = Cposition;
//         }
//       }
//   else if (angle <0)
//       {
//         digitalWrite(In3,LOW);// Extend, Pin 7 outputing HIGH and pin 8 outputing LOW
//         digitalWrite(In4,HIGH);
//         int Speed = 255;
//         analogWrite(ENA2,Speed);//Output);
//         Cposition=analogRead(ref1);
//         oldposition = Cposition;
//         Serial.println("runnning1");
//         //Input = analogRead(ref1);
//        // Setpoint = desiredpos; 
//        // myPID.SetMode(AUTOMATIC);
   
// //***PITCH IS ON OR JUST PAST DESIRED POSITION(STOP)         
//         if (Cposition>=desiredpos)
//          {
//           Cposition=analogRead(ref1);  
//           Speed = 0;
//           analogWrite(ENA2,Speed);
//           last1 = Pitch_Angle;
//           oldposition = Cposition;
//          }     
//   }
         
//    }
//void setupMPU(){
//  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
//  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
//  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
//  Wire.endTransmission();  
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
//  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
//  Wire.endTransmission(); 
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
//  Wire.write(0b00000000); //Setting the accel to +/- 2g
//  Wire.endTransmission();}
//
//  void recordAccelRegisters() {
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x3B); //Starting register for Accel Readings
//  Wire.endTransmission();
//  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
//  while(Wire.available() < 6);
//  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
//  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
//  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
//  processAccelData();}
//
//void processAccelData(){
//  gForceX = accelX / 16384.0;
//  gForceY = accelY / 16384.0; 
//  gForceZ = accelZ / 16384.0;
//  int bx = round( atan2 (gForceX, (-1)*gForceZ) * 180/3.14159265 );
//  i= i++;
//  sum = sum + bx;
//  if (i == 10)
//  {
//    offset =sum/10; 
//    i=0;
//    sum = 0;
//    }
//  
//  }
