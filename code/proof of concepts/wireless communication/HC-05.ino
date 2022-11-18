
//#define bluetoothRX 1
//#define bluetoothTX 0

//SoftwareSerial mySerial(bluetoothTX, bluetoothRX);

#define myBlue Serial1

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  myBlue.begin(9600);

}
void loop() {
  //Serial.write(myBlue.available());
 // If any data is available at the Bluetooth Serial Port
//myBlue.available
 if (myBlue.available()>0) {
   // Write this data to the Serial Monitor (Arduino)
    Serial.write(myBlue.read());
 }
  
  // If any data is sent via the Serial Monitor (Arduino)
  if (Serial.available()>0) {
    // Send this data via the Bluetooth Serial Port
    myBlue.write(Serial.read());
  }
}
