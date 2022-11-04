#include "SerialCommand.h"
#include "EEPROMAnything.h"

#define SerialPort Serial
#define Baudrate 115200

SerialCommand sCmd(SerialPort);
bool debug;
unsigned long previous, calculationTime;

const int sensor[] = {A3, A2, A1, A0, A6, A10};

struct param_t
{
  unsigned long cycleTime;
  int black[6];
  int white[6];
  /* andere parameters die in het eeprom geheugen moeten opgeslagen worden voeg je hier toe ... */
} params;

void setup()
{
  SerialPort.begin(Baudrate);

  EEPROM_readAnything(0, params);

  pinMode(13, OUTPUT);
  SerialPort.println("ready");
}

void loop()
{
  sCmd.readSerial();
 
  unsigned long current = micros();
  if (current - previous >= 1000000)
  {
    previous = current;

    SerialPort.print("Sensor waarden: ");
    for (int i = 0; i <6; i++)
  {
    SerialPort.print(analogRead(sensor[i]));
    SerialPort.print(" ");
  }
   SerialPort.println();

  }

  unsigned long difference = micros() - current;
  if (difference > calculationTime) calculationTime = difference;
}
