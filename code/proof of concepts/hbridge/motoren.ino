#define MotorLeftForward 3    //de PWM pin waar MotorLeftForward op aangesloten is, is 3
#define MotorLeftBackward 5   //de PWM pin waar MotorLeftBackward op aangesloten is, is 5
#define MotorRightForward 6  //de PWM pin waar MotorRightForward op aangesloten is, is 6
#define MotorRightBackward 9  //de PWM pin waar MotorRightBackward op aangesloten is, is 9



void setup()
{

  pinMode(MotorLeftForward, OUTPUT);
  pinMode(MotorLeftBackward, OUTPUT);
  pinMode(MotorRightForward, OUTPUT);
  pinMode(MotorRightBackward, OUTPUT);


  digitalWrite(MotorLeftForward, LOW);
  digitalWrite(MotorLeftBackward, LOW);
  digitalWrite(MotorRightForward, LOW);
  digitalWrite(MotorRightBackward, LOW);
}
void loop()
{
  for(int i = 0; i<= 255; i++)
  {
   analogWrite(MotorLeftForward, i);
   analogWrite(MotorRightForward, i);
   delay(50);
  }
  digitalWrite(MotorLeftForward, LOW);
  digitalWrite(MotorLeftBackward, LOW);
  digitalWrite(MotorRightForward, LOW);
  digitalWrite(MotorRightBackward, LOW);
  delay(1000);

    for(int i = 0; i<= 255; i++)
  {
   analogWrite(MotorLeftBackward, i);
   analogWrite(MotorRightBackward, i);
   delay(50);
  }
  digitalWrite(MotorLeftForward, LOW);
  digitalWrite(MotorLeftBackward, LOW);
  digitalWrite(MotorRightForward, LOW);
  digitalWrite(MotorRightBackward, LOW);
  delay(1000);
}
