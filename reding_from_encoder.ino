int val;
int encoder0PinA = 2;
int encoder0PinB = 3;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
float speedvalue=0;
float anglevalue=0;
int count;
int n = LOW;
float encpulses=0;

void setup() {
    Serial.begin (9600);
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), encoder, CHANGE);
}

void loop() {

  //Serial.print (encoder0Pos);
 //Serial.print ("\n");
     if (Serial.available()>0)
    {
      Serial.print("Enter the Rotation Angle ");
      anglevalue=Serial.parseInt();
      Serial.print("The Angle is ");
      Serial.println(anglevalue);
      encpulses=anglevalue*(1600.0/360.0);
      Serial.print("No. of encoder pulses");
      Serial.println(encpulses);
      
      }
      if(
    

}
void encoder ()
{
    n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos++;
      if(encoder0Pos<=encpulses)
    } else {
      encoder0Pos--;//CCW
    }
} 

}


