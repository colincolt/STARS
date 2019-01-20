/* Bibeson Krishnadasan, Krishb3, 001322125
 *  Motor Controller - Linear Actuator 45˚
 */
//***PITCH MOTOR CONTROL PINS
 int In3= 12;// Pin 7(Uno) is connected to the motor controller In1 pin
 int In4 = 13;// Pin 8(Uno) is connected to the motor controller In2 pin
 int ENA2 = 11;// Pin 5(Uno) is connected to the motor controller ENA pin
 int ref1 = A0;//Pin A0(Uno) is connected to the linear actuator position signal

 int Cposition = 0;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(In3,OUTPUT);// Pin 7 is an output 
  pinMode(In4,OUTPUT);// Pin 8 is an output
  pinMode(ENA2,OUTPUT);// Pin 5 is an output
  pinMode(ref1,INPUT); //pin A0 is an inqput
  
  Serial.begin(9600);// Window Prompt
  while(!Serial);
  Serial.println("angle (must be a value between 1˚ and 46˚,negative- Extend, positive - retract)");// Asks user for speed value
  
}

void loop() {
  // put your main code here, to run repeatedly:

 for(int counter=0;counter<=255; counter++) // loop created to constantly wait until a number is inputed
    {
      counter = 0;
      int angle = Serial.parseInt();
     
      Serial.println("cp1");
      Serial.println(angle);
      int desiredpos= ((abs(angle)-1)/45.00)*(933)+48;
 
    
  if (abs(angle) > 0 && abs(angle) <=46)
  {
    if (angle == 25)
    {
      Cposition=analogRead(ref1);
      if (Cposition > 442)
      {
      angle = 20;
      }
      if (Cposition < 442)
      {
      angle = -20;
      }
    }
    if (angle == 45)
    {
      Cposition=analogRead(ref1);
      if (Cposition > 40)
      {
      angle = 1;
      }
     
    }
  int desiredpos= ((abs(angle)-1)/45.00)*(933)+48;
  if (angle >0)
      {
        
        
        digitalWrite(In3,HIGH);// Retract, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In4,LOW);
        int Speed = 255;
        analogWrite(ENA2,Speed);
      
        
        Cposition=analogRead(ref1);
        //Serial.println(Cposition);
        Serial.println(desiredpos);
   
        
        while(Cposition>=desiredpos)
        {
          Cposition=analogRead(ref1);  
          Serial.println(Cposition);
        
        }
        
        Speed = 0;
        analogWrite(ENA2,Speed);
        Serial.println("done");
      }
  if (angle <0)
      {
      
        
        
        digitalWrite(In3,LOW);// Extend, Pin 7 outputing HIGH and pin 8 outputing LOW
        digitalWrite(In4,HIGH);
        
        int Speed = 255;
        analogWrite(ENA2,Speed);
       
        Cposition=analogRead(ref1);
        Serial.println(Cposition);
        Serial.println(desiredpos);
        
       
        while (Cposition<=desiredpos)
         {
          Cposition=analogRead(ref1);  
          Serial.println(Cposition);
         }
          Speed = 0;
          analogWrite(ENA2,Speed);
          Serial.println("done");
        }
       
        
  }  
      }
    
   

}
