int val;
void setup(){
  Serial.begin(9600);
}
void loop(){
  if(Serial.available() > 0)
  {
    String str = Serial.readStringUntil('\n');
    val = str.toInt();
    Serial.println(val);
}
}
