#include <Encoder.h>
#include <Wire.h>
/* Bibeson Krishnadasan, Krishb3, 001322125
 *  Final version series 6
 */
//***YAW MOTOR CONTROL PINS 
 int In1= 8;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In2 = 7;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA = 9;// Pin 5(Uno) is connected to the motor controller ENA pin

//*******Yaw motor variable definitions (W/Encoder)
 int angle = 0;
 int Speed = 0;
 int Barrier =24690;
 int Barrierend = -24690;
 long encodercount = 0;
 long oldposition = 0;
 long newposition = 0;
 Encoder myEnc(2,3);//yellow 3 brown 2, also power red out1 black out2
 
//***PITCH MOTOR CONTROL PINS
 int In3= 7;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In4 = 8;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA2 = 5;// Pin 5(Uno) is connected to the motor controller ENA pin
 
//*******Pitch motor variable definitions(W/Potentiometer and accerometer
 int ref1 = A0;//Pin A0(Uno) is connected to the linear actuator position signal
 int Cposition = 0;
 long accelX, accelY, accelZ;
 float gForceX, gForceY, gForceZ;
 long gyroX, gyroY, gyroZ;
 float rotX, rotY, rotZ;



 
 
void setup() {
 
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
  Serial.println("angle of rotation(yaw and pitch,must be a value between 0˚ and 160˚ or 0˚ and 45˚, respectively )");// Asks user for angle of rotation value
  Wire.begin();
  setupMPU();

//***YAW MOTOR COUNTER-CLOCKWISE ROTATION INITIALIZATION
  digitalWrite(In1,HIGH);// Pin 7 outputing HIGH and pin 8 outputing LOW
  digitalWrite(In2,LOW);

}







void loop()
{
//***MAIN PROGRAM FUNCTIONS
 YawMovement();
 PitchMovement();
 recordAccelRegisters();
 recordGyroRegisters();
 printData();
 delay(100);
}
void YawMovement(){
    for(int counter=0;counter<=255; counter++){ // loop created to constantly wait until a number is inputed 
      counter = 0;
      int Speed = 0;//intializes the speed of the motor to zero while waiting for an angle input
      analogWrite(ENA,Speed);
      
      
      int angle = Serial.parseInt();//reads serial comunications terminal for angle input
      
    
//      COUNTER-CLOCKWISE ROTATION, PHASE ONE: INITIALIZE POLARITY
      if (angle >0 && angle<161){
        digitalWrite(In1,HIGH);// Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,LOW);}

//      CLOCKWISE ROTATION, PHASE ONE: INITIALIZE POLARITY
      if (angle <0 && angle<161){
        digitalWrite(In1,LOW);//  Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In2,HIGH);}

//      RESET FUNCTION
      if (angle == 170){

//        ROTATION AMOUNT FROM LEFT OF CENTRE POSITION->NEWPOSITION=0 ( CLOCKWISE ROTATION )
        if (newposition >0){
          digitalWrite(In1,LOW);// Pin 7 outputing HIGH and pin 8 outputing LOW
          digitalWrite(In2,HIGH);
          Speed = 200;
          analogWrite(ENA,Speed);//initialize speed
          
//          READ COUNTABLE EVENTS FROM THE ENCODER CHANNELS (FOREVER, EXIT:BREAK OUT OF LOOP)  
          do{
          newposition= ((myEnc.read()));
          Serial.println(newposition);
          
//          REACHED CENTRE POSITION, BREAK
          if (newposition <= 0) {
            Speed = 0;
            analogWrite(ENA,Speed);
            break;}
            }while (1>0);}

//        ROTATION AMOUNT FROM RIGHT OF CENTRE POSITION->NEWPOSITION=0 ( COUNTER-CLOCKWISE ROTATION )
        if (newposition <0){
          Serial.println("cp2");
          digitalWrite(In1,HIGH);// Pin 7 outputing HIGH and pin 8 outputing LOW
          digitalWrite(In2,LOW);
          Speed = 200;
          analogWrite(ENA,Speed);//initialize speed
          
//          READ COUNTABLE EVENTS FROM THE ENCODER CHANNELS (FOREVER, EXIT:BREAK OUT OF LOOP)   
          do{
          newposition= ((myEnc.read()));
          Serial.println(newposition);

//          REACHED CENTRE POSITION, BREAK
          if (newposition >= 0){
            Speed = 0;
            analogWrite(ENA,Speed);
            break;}
        }while (1>0);}
        }


//     SPEED INITIALIZATION 
     if (abs(angle) > 0 && abs(angle) <161){// Only accepts angles 0-160
      
//       LARGE ANGLES CAN BE AT HIGHER SPEEDS AND BE ACCURATE
        if (abs(angle) >=10){
        Speed = 200;}
        
//       SMALL ANGLES REQUIRE LOWER SPEEDS TO BE ACCURATE 
        if (abs(angle) <10){
        Speed = 70;}
        analogWrite(ENA,Speed); // pin 5 is outputing the speed of the motor
         
//     NUMBER OF COUNTABLE EVENTS FOR A GIVEN ANGLE
      encodercount = (6.26847)*(49.1684)*(angle) + encodercount;
//      **equation for how many countable events to look for
//      ***countable events/per degree at the motor 
//         shaft for one rotation of the output 
//         shaft= 17700.624/360= 49.1684
//      ****platform/output shaft ratio= 6.26847 
    
//     COUNTER-CLOCKWISE ROTATION, PHASE TWO: ROTATE TO DESIRED POSITION (ENCODER COUNT)
      if (angle >0){
        
//       READ COUNTABLE EVENTS FROM THE ENCODER CHANNELS (FOREVER, EXIT:BREAK OUT OF LOOP)
        do{
          newposition= ((myEnc.read()));// number of countable events seen 
          
//         REACHED DESIRED POSITION, BREAK  
          if (newposition >= encodercount){
              break;}
              
//         REACHED BARRIER, BREAK
          if (newposition >= Barrier){
            encodercount= Barrier;
            break; }   
        }while (1>0);}
        
//    CLOCKWISE ROTATION, PHASE TWO: ROTATE TO DESIRED POSITION (ENCODER COUNT)
     if (angle <0){
      
//      READ COUNTABLE EVENTS FROM THE ENCODER CHANNELS (FOREVER, EXIT:BREAK OUT OF LOOP)   
       do{
          newposition= (myEnc.read()); // number of countable events seen 
          
//        REACHED DESIRED POSITION, BREAK     
         if (newposition <= encodercount){ // reached desired angle
              break;}
    
//        REACHED BARRIER, BREAK
           if (newposition <= Barrierend){
            encodercount= Barrierend;
            break;}
        }while (1>0);}

//    YAW MOVEMENT FUNCTION ALWAYS ENDS WITH MOTOR OFF
        Speed = 0;// stops motor after reaching desired point
        analogWrite(ENA,Speed);}}}
   
   
   
   
   
   void PitchMovement();{ 
      for(int counter=0;counter<=255; counter++){ // loop created to constantly wait until a number is inputed
      counter = 0;
      int angle = Serial.parseInt();
      
//      AMOUNT OF COUNTABLE EVENTS TO REACH THE DESIRED POSITION    
      int desiredpos= ((abs(angle)-1)/45.00)*(933)+48;
 
//        PITCH OF LAUNCHER, ANYWHERE BETWEEN 0˚ and 45˚
         if (abs(angle) > 0 && abs(angle) <=46){

//          MOVEMENT UPWARDS     
           if (angle >0){
            digitalWrite(In3,HIGH);// Retract linear actuator, Pin 7 outputing HIGH and pin 8 outputing LOW
            digitalWrite(In4,LOW);
            int Speed = 255;
            analogWrite(ENA2,Speed);
            Cposition=analogRead(ref1);

//            READ POTENTIOMETER UNTIL CURRENT POSITION HAS REACHED DESIRED POSITION
             while(Cposition>=desiredpos){
              Cposition=analogRead(ref1);}
        
//       PITCH MOVEMENT ENDS WITH MOTOR OFF
        Speed = 0;
        analogWrite(ENA2,Speed);}

//          MOVEMENT DOWNWARDS
       if (angle <0){
        digitalWrite(In3,LOW);// Extend linear actuator, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In4,HIGH);
        int Speed = 255;
        analogWrite(ENA2,Speed);
        Cposition=analogRead(ref1);

//            READ POTENTIOMETER UNTIL CURRENT POSITION HAS REACHED DESIRED POSITION
             while (Cposition<=desiredpos){
              Cposition=analogRead(ref1);}

//       PITCH MOVEMENT ENDS WITH MOTOR OFF
        Speed = 0;
        analogWrite(ENA2,Speed);}}}}}





        
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();}




void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();}





void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;}





void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();}






void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;}






void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);}


  
