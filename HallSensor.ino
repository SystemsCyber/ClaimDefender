int rpm= 240;
int missing_tooth = rpm/13;
int shaping = rpm;
void setup() {
  // put your setup code here, to run once:
pinMode(2,OUTPUT);
pinMode(13,OUTPUT);
digitalWrite(13, HIGH);
analogWriteFrequency(2, rpm);
analogWrite(2,128);
}

void loop() {
  // put your main code here, to run repeatedly:
//analogWriteFrequency(2, rpm);
analogWrite(2,128);
delay(shaping);
analogWrite(2,0);
delay(missing_tooth);
}
