// LIBRARIES
 #include <Encoder.h>
 #include <PID_v1.h>
 #include <Wire.h>
 #include <MPU6050.h>

 MPU6050 mpu;

 int In1= 8;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In2 = 7;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA = 9;// Pin 5(Uno) is connected to the motor controller ENA pin
 
//***YAW MOTOR VARIABLES
 int angle = 0;
 int Stop = 0;
 int oldposition = 0;
 int Barrier = 10000;

 long EncoderCount = 0;
 int Dir;
 int Yaw_Speed = 0;
 int Yaw_PWM;
 double Setpoint_2;
 double Input_2;
 double Output_2;

 int Num_Pixels = 3280;
 
 int Disparity;
 
 Encoder myEnc(2,3);//2 - Bro<wn; 3 - Orange , also power red out1 black out2
 PID myPID_Yaw(&Input_2, &Output_2, &Setpoint_2,0.1,0.015,0.04,DIRECT);
 
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
 
 int Cposition;
 int Pitch_Angle;

 int desiredpos;
 int Speed;
 
//***PID VARIABLES
 double Setpoint;
 double Input;
 double Output;

 //With these tuning factors, can step through 0-40 in 1 degreee increments
 PID myPID(&Input, &Output, &Setpoint,5,0,0,DIRECT);

//***ACCELEROMETER VARIABLES
 long accelX, accelY, accelZ;
 float gForceX, gForceY, gForceZ;
 int i = 0;
 int sum = 0;
 int offset = 0; //offset angle

 unsigned long Accel_Get_Interval = 3000;
 unsigned long Prev_Accel_Time;
 unsigned long Pot_Get_Interval = 1000;
 unsigned long Prev_Pot_Time;
 
//***Timing
 long displayInterval = 2000;
 long prevDisplayMillis = 0;

 float Counts_to_Move;

 // Need to account for Gear ratio
 float Gear_Ratio = 2.1;
 float EncoderCounts_per_Pixel = ((17700.624)/(360))*((62.2)/(Num_Pixels))*Gear_Ratio;


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
//    Serial.println("This demo expects 2 pieces of data - Motor 1 Speed, Motor 2 Speed");
//    Serial.println("Enter data in this style <2000,3000>  ");
//    Serial.println();
    
  
//***YAW MOTOR COUNTER-CLOCKWISE ROTATION INITIALIZATION 
  digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
  digitalWrite(In2,LOW);

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50);
  myPID.SetOutputLimits(-255,255);
  
  myPID_Yaw.SetMode(AUTOMATIC);
  myPID_Yaw.SetSampleTime(10);
  myPID_Yaw.SetOutputLimits(-255,255);
  
//  Time = millis();
 while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
//    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(100);
  }
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

// if (Yaw_Speed <= 255)
// {
//  yawMotor();
// }
//  
// else if (Yaw_Speed == 300)
// {
//  Reset();
// }


 if (Disparity <= 4999)
 {
  yawMotorPID();
 }
  
 else if (Disparity == 5000)
 {
  Reset();
 }



  if ((millis() - Prev_Accel_Time) >= Accel_Get_Interval){
    accelerometerData();
    Prev_Accel_Time = millis();
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
    
//    strtokIndx = strtok(tempChars,",");// get the first part - the string
//    Yaw_Speed = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(tempChars,",");// get the first part - the string
    Disparity = atoi(strtokIndx);     // convert this part to an integer
    
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Pitch_Angle = atoi(strtokIndx);     // convert this part to an integer

    if (Disparity <= 4999){

      // "+" or "-" determines direction of rotation
        Setpoint_2 = (myEnc.read()) + (Disparity)*(EncoderCounts_per_Pixel);
//        Serial.print("Setpoint 2 = ");
//        Serial.println(Setpoint_2);
    }

        
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
    
//     int player_angle = EncoderCount/ ((2.96011)*(49.1684)); // players angle from orgin (zero position) 

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



void yawMotorPID(){
  
 //********* YAW SECTION
    EncoderCount = (myEnc.read());// number of countable events seen
    Input_2 = EncoderCount;
    myPID_Yaw.Compute();
     
//********ROTATION AMOUNT FROM RIGHT OF CENTRE POSITION->EncoderCount=0 (CLOCKWISE ROTATION )
        if (Output_2 >= 0)
        {
           
           digitalWrite(In1,HIGH);// Pin 7 outputing HIGH and pin 8 outputing LOW
           digitalWrite(In2,LOW);
           Dir = 0;
           Yaw_PWM = map(Output_2, 0, 255, 110 , 255);
           
           if (Yaw_PWM <= 112){
           Yaw_PWM = 0; 
           }

        }


//********ROTATION AMOUNT FROM LEFT OF CENTRE POSITION->EncoderCount=0 (COUNTER-CLOCKWISE ROTATION )
        if (Output_2 < 0)
        {
                    
           digitalWrite(In1,LOW);// Pin 7 outputing HIGH and pin 8 outputing LOW
           digitalWrite(In2,HIGH);
           Dir = 1;
           Yaw_PWM = map(Output_2, 0, -255, 110 , 255);
           
           if (Yaw_PWM <= 112){
           Yaw_PWM = 0; 
           }         

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




 void Reset()
 {
//*** RESET FUNCTION

     EncoderCount = (myEnc.read());  
     Input_2 = EncoderCount;    
     Setpoint_2 = 0;
     myPID_Yaw.Compute();

     
//********ROTATION AMOUNT FROM RIGHT OF CENTRE POSITION->EncoderCount=0 (CLOCKWISE ROTATION )
        if (Output_2 >= 0)
        {

           digitalWrite(In1,HIGH);
           digitalWrite(In2,LOW);
           Dir = 0;
           Yaw_PWM = map(Output_2, 0, 255, 110 , 255);
           if (Yaw_PWM <= 112){
           Yaw_PWM = 0; 
           }
//           analogWrite(ENA,Yaw_PWM);//initialize speed
        }


//********ROTATION AMOUNT FROM LEFT OF CENTRE POSITION->EncoderCount=0 (COUNTER-CLOCKWISE ROTATION )
        if (Output_2 < 0)
        {
          
           digitalWrite(In1,LOW);
           digitalWrite(In2,HIGH);
           Dir = 1;
           Yaw_PWM = map(Output_2, 0, -255, 110 , 255);
           if (Yaw_PWM <= 112){
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

 //Max value = 967
 //Min value = 39
     int desiredpos = 967.00 - 928.00*((Pitch_Angle - offset)/45.00);
 //*** current position of the pitch (potentiometer reading)
     Cposition = analogRead(ref1);

//    if ((millis() - Prev_Pot_Time) >= Pot_Get_Interval){
//       Serial.println(Cposition);
//       Prev_Pot_Time = millis();
//    }


     Setpoint = desiredpos;
     Input = Cposition;
     
      myPID.Compute();


//***DIRECTION OF PITCH 
     if (Output <= 0) // IF Current position is lower than desired position then lin_act needs to extend 
     {
      digitalWrite(In3,LOW);// retact, Pin 7 outputing HIGH and pin 8 outputing LOW
      digitalWrite(In4,HIGH);

     Speed = map(Output, 0, -255, 0 , 255);
      if (Speed <= 40){
      Speed = 0; 
      }

      analogWrite(ENA2,Speed);  
//      Serial.println("retracting");
     }
      
     else if (Output > 0) // IF Current position is higher than desired position then lin_act needs to retract 
     {
      digitalWrite(In3,HIGH);// extend, Pin 7 outputing HIGH and pin 8 outputing LOW
      digitalWrite(In4,LOW);

      Speed = map(Output, 0, 255, 0 , 255);
      if (Speed <= 40){
      Speed = 0; 
      }
      
      analogWrite(ENA2,Speed); 

     }
     

 }




void accelerometerData()
 {
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  int offset = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
//  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  // Output
//  Serial.print(" Pitch = ");
//  Serial.print(offset);
//  Serial.print(" Roll = ");
//  Serial.print(roll);
  
//  Serial.println();
  
  delay(10);
}
