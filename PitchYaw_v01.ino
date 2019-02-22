// LIBRARIES
 #include <Encoder.h>
 #include <PID_v1.h>
// #include <Wire.h>
//***YAW MOTOR CONTROL PINS 
 int In1= 8;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In2 = 7;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA = 9;// Pin 5(Uno) is connected to the motor controller ENA pin
 
//***YAW MOTOR VARIABLES
 int angle = 0;
 int Speed = 0;
 int oldposition = 0;
 int Barrier =14000;
 //int Barrierend = -14000;
 long encodercount = 0;
 long newposition = 0;
 Encoder myEnc(2,3);//yellow 3 brown 2, also power red out1 black out2
 
//***SERIAL COMMUNICATION VARIABLES
 const byte numChars = 32;
 char receivedChars[numChars];   
 boolean newData = false;
 int dataNumber = 0;
 char tempChars[numChars]; 
 int Motor1_Speed = 0;
 
//***PITCH MOTOR CONTROL PINS
 int In3= 12;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In4 = 13;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA2 = 11;// Pin 5(Uno) is connected to the motor controller ENA pin
 int ref1 = A0;//Pin A0(Uno) is connected to the linear actuator position signal(blue)
 int last1 = 0;
 int Cposition = 0;
 int Motor2_Speed = 0;
 int angle1= 0;
 int desiredpos = 0;
 
//***PID VARIABLES
 double Setpoint;
 double Input;
 double Output;
 PID myPID(&Input, &Output, &Setpoint,2,0,0,DIRECT);

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
  Serial.begin(9600);// Window Prompt
  while(!Serial);
 // Wire.begin();
//setupMPU();
  
//***YAW MOTOR COUNTER-CLOCKWISE ROTATION INITIALIZATION 
  digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
  digitalWrite(In2,LOW);

  myPID.SetMode(AUTOMATIC);
//  Time = millis();
}

void loop() {
//*** SERIAL COMMUNICATION (turning serial input into integer)
  Input = analogRead(ref1);
 
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();
    newData = false;}
  if (millis() - prevDisplayMillis >= displayInterval) {
       prevDisplayMillis += displayInterval;
       //recordAccelRegisters();
   }
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
    Serial.println("tempchars:");
    Serial.println(tempChars);
    strtokIndx = strtok(tempChars,",");// get the first part - the string
    Motor1_Speed = atoi(strtokIndx);     // convert this part to an integer
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Motor2_Speed = atoi(strtokIndx);     // convert this part to an integer
//*** PITCH and YAW INPUTS DEFINED FROM PI (Motor1_Speed, Motor2_Speed)
    Serial.println("ms1");
    Serial.println(Motor1_Speed);
    Serial.println("ms2");
    Serial.println(Motor2_Speed);


    newposition= ((myEnc.read()));// number of countable events seen       
    int player_angle = newposition/ ((2.96011)*(49.1684)); // players angle from orgin (zero position) 
    
//********* YAW SECTION
//*** DIRECTION OF YAW
    if (Motor1_Speed >0){
        digitalWrite(In1,HIGH);// Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,LOW);}
    if (Motor1_Speed<0){
        digitalWrite(In1,LOW);// Counter-Clockwise rotation of the motor, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,HIGH);}
//*** SPEED OF YAW
    Speed = abs(Motor1_Speed);
    
    analogWrite(ENA,Speed);
    

    if (abs(newposition) >= Barrier){
        Speed = 0;
        analogWrite(ENA,Speed);}
        
//*** RESET FUNCTION
      if (Motor1_Speed == 300){

//********ROTATION AMOUNT FROM LEFT OF CENTRE POSITION->NEWPOSITION=0 ( CLOCKWISE ROTATION )
        if (newposition >0){
          digitalWrite(In1,LOW);// Pin 7 outputing HIGH and pin 8 outputing LOW
          digitalWrite(In2,HIGH);
          Speed = 255;
          analogWrite(ENA,Speed);//initialize speed
          
//***********READ COUNTABLE EVENTS FROM THE ENCODER CHANNELS (FOREVER, EXIT:BREAK OUT OF LOOP)  
          do{
          newposition= ((myEnc.read()));
          Serial.println(newposition);
          
//***********REACHED CENTRE POSITION, BREAK
          if (newposition <= 0) {
            Speed = 0;
            analogWrite(ENA,Speed);
            break;}
            }while (1>0);}

//********ROTATION AMOUNT FROM RIGHT OF CENTRE POSITION->NEWPOSITION=0 ( COUNTER-CLOCKWISE ROTATION )
        if (newposition <0){
          Serial.println("cp2");
          digitalWrite(In1,HIGH);// Pin 7 outputing HIGH and pin 8 outputing LOW
          digitalWrite(In2,LOW);
          Speed = 200;
          analogWrite(ENA,Speed);//initialize speed
          
//***********READ COUNTABLE EVENTS FROM THE ENCODER CHANNELS (FOREVER, EXIT:BREAK OUT OF LOOP)   
          do{
          newposition= ((myEnc.read()));
          Serial.println(newposition);

//***********REACHED CENTRE POSITION, BREAK
          if (newposition >= 0){
            Speed = 0;
            analogWrite(ENA,Speed);
            break;}
        }while (1>0);}
        }


    
//*********PITCH SECTION
//***ANGLE DEFINED
    angle1 = 45 - Motor2_Speed;
    angle = angle1 - offset;

    Cposition=analogRead(ref1);
 
    desiredpos= ((abs(angle)-1)/45.00)*(933)+48;
   
    Setpoint = desiredpos;
    myPID.Compute();
    Serial.println("Output:");
    Serial.println(Output);
 
//***DIRECTION OF PITCH 
    if (Cposition < oldposition)
    {
      angle = (-1)*angle;}
//***PITCH WILL NOT MOVE IF IN SAME POSITION AS DESIRED POSITION
  if (last1 == Motor2_Speed)
  {
    digitalWrite(In3,HIGH);// Retract, Pin 7 outputing HIGH and pin 8 outputing LOW
    digitalWrite(In4,LOW);
    int Speed = 0;
    analogWrite(ENA2,Speed);}
//***
  else if (angle >0)
      {
        digitalWrite(In3,HIGH);// Retract, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In4,LOW);
        int Speed = 255;
        analogWrite(ENA2,Speed);//Output);
        Cposition=analogRead(ref1);
        oldposition = Cposition;
        Serial.println("runnning2 ");
        //Input = analogRead(ref1);
       // Setpoint = desiredpos; 
       // myPID.SetMode(AUTOMATIC);
   
//***PITCH IS ON OR JUST PAST DESIRED POSITION(STOP)        
        if (Cposition<=desiredpos)
        {
        Cposition=analogRead(ref1);  
        Speed = 0;
        analogWrite(ENA2,Speed);
        last1 = Motor2_Speed;
        oldposition = Cposition;
        }
      }
  else if (angle <0)
      {
        digitalWrite(In3,LOW);// Extend, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In4,HIGH);
        int Speed = 255;
        analogWrite(ENA2,Speed);//Output);
        Cposition=analogRead(ref1);
        oldposition = Cposition;
        Serial.println("runnning1");
        //Input = analogRead(ref1);
       // Setpoint = desiredpos; 
       // myPID.SetMode(AUTOMATIC);
   
//***PITCH IS ON OR JUST PAST DESIRED POSITION(STOP)         
        if (Cposition>=desiredpos)
         {
          Cposition=analogRead(ref1);  
          Speed = 0;
          analogWrite(ENA2,Speed);
          last1 = Motor2_Speed;
          oldposition = Cposition;
         }     
  }
         
    }
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

