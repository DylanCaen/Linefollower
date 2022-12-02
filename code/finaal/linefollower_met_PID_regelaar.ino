#include "SerialCommand.h"
#include "EEPROMAnything.h"

#define SerialPort Serial1
#define Baudrate 9600
#define MotorLeftForward 3    //de PWM pin waar MotorLeftForward op aangesloten is, is 6
#define MotorLeftBackward 5  //de PWM pin waar MotorLeftBackward op aangesloten is, is 9
#define MotorRightForward 6   //de PWM pin waar MotorRightForward op aangesloten is, is 3
#define MotorRightBackward 9  //de PWM pin waar MotorRightBackward op aangesloten is, is 5

SerialCommand sCmd(SerialPort);

bool debug;
bool run;
int ledPin = 8;       //de led is aangesloten op digitale pin 8
int interruptPin = 2; //de drukknop is aangesloten op digitale pin 2
int state = LOW;
unsigned long previous, calculationTime, lastInterrupt;

const int sensor[] = {A0, A1, A2, A3, A6, A10}; //de QTR-8A sensor is aangesloten op deze analoge pinnen
int normalised[6];
float debugPosition;
float lastErr;

struct param_t //alle variabelen die opgenomen moeten worden in het eeprom geheugen worden hierin verzameld
{
  unsigned long cycleTime;
  int black[6]; //in dit array zitten de gecalibreerde zwart waarden 
  int white[6]; //in dit array zitten de gecalibreerde wit waarden
  int power;    //parameter Proportionele regelaar
  float diff;   //parameter Proportionele regelaar
  float kp;     //parameter Proportionele regelaar
  float ki;
  float kd;
} params;

void setup()
{
  SerialPort.begin(Baudrate);

  sCmd.addCommand("set", onSet);                //commando set toegevoegd, als set verstuurd wordt, wordt de functie onSet aangeroepen
  sCmd.addCommand("debug", onDebug);            //commando debug toegevoegd, als debug verstuurd wordt, wordt de functie onDebug aangeroepen
  sCmd.addCommand("calibrate", onCalibrate);    //commando calibrate toegevoegd, als calibrate verstuurd wordt, wordt de functie onCalibrate aangeroepen
  sCmd.addCommand("run", onRun);                //commando debug toegevoegd, als run verstuurd wordt, wordt de functie onRun aangeroepen
  sCmd.setDefaultHandler(onUnknownCommand);     //als er een commando gestuurd wordt die niet gekend is wordt de functie onUnknownCommand aangeroepen 

  EEPROM_readAnything(0, params);

  pinMode(ledPin, OUTPUT);                //de pin waar de ledPin op aangesloten is wordt gedefineerd als een output
  pinMode(interruptPin, INPUT_PULLUP);    //de pin waar de drukknop op aangesloten is wordt gedefinieerd als een input_pullup (geen fysieke weerstand nodig)
  pinMode(MotorLeftForward, OUTPUT);      //de pin waar MotorLeftForward op aangesloten is wordt gedefinieerd als een output
  pinMode(MotorLeftBackward, OUTPUT);     //de pin waar MotorLeftBackward op aangesloten is wordt gedefinieerd als een output
  pinMode(MotorRightForward, OUTPUT);     //de pin waar MotorRightForward op aangesloten is wordt gedefinieerd als een output
  pinMode(MotorRightBackward, OUTPUT);    //de pin waar MotorRightBackward op aangesloten is wordt gedefinieerd als een output

  digitalWrite(MotorLeftForward, LOW);    //de pin waar MotorLeftForward op aangesloten is wordt LOW gezet zodat de motor zeker niet begint te draaien wanneer er gewoon spanning wordt aangelegd
  digitalWrite(MotorLeftBackward, LOW);   //de pin waar MotorLeftBackward op aangesloten is wordt LOW gezet zodat de motor zeker niet begint te draaien wanneer er gewoon spanning wordt aangelegd
  digitalWrite(MotorRightForward, LOW);   //de pin waar MotorRightForward op aangesloten is wordt LOW gezet zodat de motor zeker niet begint te draaien wanneer er gewoon spanning wordt aangelegd
  digitalWrite(MotorRightBackward, LOW);  //de pin waar MotorRightBackward op aangesloten is wordt LOW gezet zodat de motor zeker niet begint te draaien wanneer er gewoon spanning wordt aangelegd

  digitalWrite(ledPin, LOW);

  lastInterrupt = millis();
  attachInterrupt(digitalPinToInterrupt(interruptPin), toggleState, CHANGE); //de interruptpin wordt hier gecreëerd, wanneer de status van de interruptpin veranderd wordt de functie toggleState uitgevoerd
  SerialPort.println("ready");
}

void loop()
{
  sCmd.readSerial();

  if(state)
  {
    run = true;
    digitalWrite(ledPin, HIGH);
  }
  else if (!state)   
  {
    run = false;
    digitalWrite(ledPin, LOW);
  }
  
  unsigned long current = micros();
  if (current - previous >= params.cycleTime)
  {
    previous = current;

    /* code die cyclisch moet uitgevoerd worden programmeer je hier ... */

    /* normaliseren en interpoleren sensor */

    /* pid regeling */

    /* aansturen motoren */

    //measure & normalize
   // SerialPort.print("normalised values: ");
    for (int i = 0; i<6; i++) 
    {
      normalised[i] = map(analogRead(sensor[i]), params.black[i], params.white[i], 0, 1000); //zwartwaarden ≈ 0, witwaarden ≈ 1000
     // SerialPort.print(normalised[i]);
      //SerialPort.print(" ");
    }
    //SerialPort.println(" ");
    
    //interpolatie
    float position = 0;
    int index = 0;
    for (int i = 0; i<6; i++) if(normalised[i] < normalised[index])index = i; //zwartste sensor van de 6 bepalen
    
    if (normalised[index] > 500) 
    {
    run = false;//als de waarde van de "zwartste sensor" kleiner is dan 400 kunnen we veronderstellen dat er geen zwarte lijn aanwezig is, dus moeten we de motoren niet aansturen
    }
    if (index == 0) position = -30;     //dit wil zeggen dat bvb. de uiterst linkse sensor de "zwartste sensor" is dus kunnen we de positie gelijk stellen aan -30 
    else if (index == 5) position = 30; //dit wil zeggen dat bvb. de uiterst rechtse sensor de "zwartste sensor" is dus kunnen we de positie gelijk stellen aan 30
    else
    {
      //We nemen 3 meetwaarden (sNul, sminEen, sPlusEen) van de genormaliseerde sensor en tekenen er een parabool mee
      int sNul = normalised[index];         
      int sMinEen = normalised[index-1];   //deze stap zou niet lukken met een index = 0 daardoor de "if(index == 0)" hierboven
      int sPlusEen = normalised [index+1]; //deze stap zou niet lukken met een index = 5 daardoor de "if(index == 1)" hierboven

      //formule parabool (y = ax² + bx + c)
      float b = sPlusEen - sMinEen;  //formule om b te berekenen
      b = b/2;

      float a = sPlusEen - b - sNul; //formule om a te berekenen

      position = -b / (2 * a);      //position kun je bepalen met de afgeleide van de parabool formule en y gelijk te stellen aan 0 en daaruit x(position) te halen
      position = position + index;  //zwartste punt verschuiven zodat de y-as op de sensorwaarde van sensor 0 terecht komt 
      position = position - 2.5;    //y-as verschuiven met 2,5 zodat de y-as in het midden van de sensorwaarden licht

      position = position * 15;     //verschalen van de positiewaarde
    }
    debugPosition = position;

    /* berekenen error = setpoint - positie */
    float error = -position;          //error = setpoint - input (de setpoint is 0 want dit is de ideale positie van de auto op de zwarte lijn)

    /* proportioneel regelen */
    float output = error * params.kp; //error is een input voor de regelaar

    /* intigrerend regelen */
    float iTerm = iTerm + params.ki * error;
    iTerm = constrain(iTerm, -510, 510);

    /*differeniërend regelen*/
    output = output + params.kd * (error - lastErr);
    lastErr = error;

    /* output begrenzen tot wat fysiek mogelijk is */
    output = constrain(output, -510, 510); //(constrain => waarde beperken tussen 2 grenzen) PWM waarde min = -255, max = +255 => 2 motoren min = -510, max = +510

    int powerLeft = 0;
    int powerRight = 0;
      
    if(run) if(output >= 0)
    {
      powerLeft = constrain (params.power + params.diff * output, -255, 255); //(constrain => waarde beperken tussen 2 grenzen) PWM waarde min = -255, max = +255
      powerRight = constrain (powerLeft - output, -255, 255);                 //(constrain => waarde beperken tussen 2 grenzen) PWM waarde min = -255, max = +255
      powerLeft = powerRight + output;
    }
    else
    {
      powerRight = constrain (params.power - params.diff * output, -255, 255);
      powerLeft = constrain (powerRight + output, -255, 255);
      powerRight = powerLeft - output;
    }
    analogWrite(MotorLeftForward, powerRight >0 ? powerRight : 0);      //MotorLeftForward wordt aangestuurd met analoge waarde powerLeft
    analogWrite(MotorLeftBackward, powerRight <0 ? -powerRight : 0);    //MotorLeftBackward wordt aangestuurd met analoge waarde powerLeft
    analogWrite(MotorRightForward, powerLeft >0 ? powerLeft : 0);       //MotorRightForward wordt aangestuurd met analoge waarde powerRight
    analogWrite(MotorRightBackward, powerLeft <0 ? -powerLeft : 0);     //MotorRightBackward wordt aangestuurd met analoge waarde powerRight
    /*indien de robot net wegrijdt van de zwarte lijn in plaats van er naar toe moet je de waardes powerLeft en powerRight omwisselen.*/
  }

  unsigned long difference = micros() - current;
  if (difference > calculationTime) calculationTime = difference;
}

void onUnknownCommand(char *command)
{
  SerialPort.print("unknown command: \"");
  SerialPort.print(command);
  SerialPort.println("\"");
}

void onSet()
{
  char* param = sCmd.next();
  char* value = sCmd.next();  
  
  if (strcmp(param, "cycle") == 0) // met het commando "set cycle 2000000" zet je de cyclustijd op 2 seconden
  {
    long newCycleTime = atol(value);  
    float ratio = ((float) newCycleTime)/((float) params.cycleTime);
    
    params.ki = params.ki * ratio;
    params.kd = params.kd / ratio;

    params.cycleTime = newCycleTime;
  }
  else if (strcmp(param, "power") == 0) params.power = atol(value);   //snelheid (met het commando "set power 50" zet je de power op 50 (waarde meegeven tussen 0 en 255))
  else if (strcmp(param, "diff") == 0) params.diff = atof(value);     //geeft aan of er rapper of trager moet gereden worden adhv de grote van de error (met het commando "set diff 0,5" zet je diff op 0,5(waarde meegeven tussen 0 en 255)) (waarde meegeven tussen 0 en 1)
  else if (strcmp(param, "kp") == 0) params.kp = atof(value);         //geeft aan hoe sterk er proportioneel moet bijgeregeld worden (met het commando "set kp 10" zet je kp op 10 (waarde meegeven groter dan 0))
  else if (strcmp(param, "ki") == 0)
  {
    float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
    params.ki = atof(value) * cycleTimeInSec;
  }
  else if (strcmp(param, "kd") == 0)
  {
    float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
    params.kd = atof(value) / cycleTimeInSec;
  }
  /* parameters een nieuwe waarde geven via het set commando doe je hier ... */
  /* rijdt de robot te traag / valt de robot stil => verhoog de parameter power */
  /* stuurt de robot niet snel genoeg bij => verhoog Kp */
  /* waggelt de robot over de zwarte lijn => verlaag Kp */
  /* versnelt de robot in de bochten => verlaag diff */
  /* vertraagt de robot in de bochten / valt de robot stil in de bochten => verhoog diff1*/
  EEPROM_writeAnything(0, params);
}

void onDebug() /* parameters weergeven met behulp van het debug commando doe je hier ... */
{
  SerialPort.print("cycle time: ");
  SerialPort.println(params.cycleTime);

  SerialPort.print("black: ");
  for (int i = 0; i < 6; i++) 
  {
    SerialPort.print(params.black[i]);  
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("white: ");
  for (int i = 0; i < 6; i++) 
  {
    SerialPort.print(params.white[i]);  
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("normalised values: ");
  for (int i = 0; i<6; i++)
  { 
    SerialPort.print(normalised[i]);    
    SerialPort.print(" ");
  }
  SerialPort.println(" ");

  SerialPort.print("positie: ");
  SerialPort.println(debugPosition);

  SerialPort.print("Power: ");
  SerialPort.println(params.power);
  SerialPort.print("diff: ");
  SerialPort.println(params.diff);
  SerialPort.print("kp: ");
  SerialPort.println(params.kp);
  
  float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
  float ki = params.ki / cycleTimeInSec;
  SerialPort.print("Ki: ");
  SerialPort.println(ki);

  float kd = params.kd * cycleTimeInSec;
  SerialPort.print("Kd: ");
  SerialPort.println(kd);
  
  SerialPort.print("calculation time: ");
  SerialPort.println(calculationTime);
  calculationTime = 0;
}
void onCalibrate()
{
  char* param = sCmd.next();

  if (strcmp(param, "black") == 0) //wordt uitgevoerd als het commando "calibrate black" wordt ingegeven
  {
    SerialPort.print("start calibrating black... ");
    for (int i = 0; i < 6; i++) params.black[i]=analogRead(sensor[i]);
    SerialPort.println("done");
  }
  else if (strcmp(param, "white") == 0) //wordt uitgevoerd als het commando "calibrate white" wordt ingegeven
  {
    SerialPort.print("start calibrating white... ");    
    for (int i = 0; i < 6; i++) params.white[i]=analogRead(sensor[i]);  
    SerialPort.println("done");      
  }

  EEPROM_writeAnything(0, params);
}
void onRun()
{
  char* param = sCmd.next();
  if (strcmp(param, "start") == 0)     //met het commando run start zet je run op true en worden de motoren aangestuurd
  {
    run = true;
    digitalWrite(ledPin, HIGH);
  }
  else if (strcmp(param, "stop") == 0) //met het commando run stop zet je run op false en stoppen de motoren
  {
    run = false; 
    digitalWrite(ledPin, LOW); 
  }
}
void toggleState() 
{
  if (millis() - lastInterrupt > 200)
  {
    state = !state;
    lastInterrupt = millis();
  }
}
