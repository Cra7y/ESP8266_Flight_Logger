#include <Arduino.h> // Include main arduino lib

// Include other libs
#include <servo.h> //Servo
#include <Wire.h> // I2C

// Global functions
//Combines two 8-bit bytes to one 16-bit integer
//Inspired by this blogpost: http://projectsfromtech.blogspot.dk/2013/09/combine-2-bytes-into-int-on-arduino.html
int BitShiftCombine(byte MSB, byte LSB) {
  int combined; 
  combined = MSB;           //send MSB to rightmost 8 bits
  combined = combined<<8;         //shift MSB over to leftmost 8 bits
  combined |= LSB;                 //logical OR keeps MSB intact in combined and fills in rightmost 8 bits
  return combined;
}


// Servo global variables
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
void activateServo() {
  if (pos == 0) {
      pos = 180;
    } else {
      pos = 0;
    }
    myservo.write(pos);
}


// ADXL 345 global variables
float x, y, z, sum;
// Sets POWER_CTL and DATA_FORMAT on accelerometer
void adxlSetup() {
  //Write 00001000 to register 0x2D (POWER_CTL) to activate accelerometer
  Wire.beginTransmission(0x53); //Send start condition
  Wire.write(byte(0x2D)); //Choose register
  Wire.write(0b00001000); //Write data
  Wire.endTransmission(); //Send stop condition

  //Write 00001011 to register 0x31 (DATA_FORMAT) to set scale to +-16g
  Wire.beginTransmission(0x53); //Send start condition
  Wire.write(byte(0x31)); //Choose register
  Wire.write(0b00001011); //Write data
  Wire.endTransmission(); //Send stop condition
}
// Gets readings from accelerometer
void readAdxl() {
  //Read register 0x32 to 0x37 wich contains x, y and z acceleration
  Wire.beginTransmission(0x53); //Send start condition
  Wire.write(byte(0x32)); //Choose register
  Wire.endTransmission(); //Send stop condition
  Wire.requestFrom(0x53, 6); //Request 6 bytes
  byte xyz[6]; // create array af bytes
  int i = 0; // create iteretor
  //Run until all data is received
  while (Wire.available()) {
    xyz[i] = Wire.read();// Save data to variable
    i++; //Iterate
  }
  Wire.endTransmission(); //Send stop condition
  //Convert values to g
  x = BitShiftCombine(xyz[1], xyz[0])*0.00390625;
  y = BitShiftCombine(xyz[3], xyz[2])*0.00390625;
  z = BitShiftCombine(xyz[5], xyz[4])*0.00390625;
  sum = sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2)); //Calculate sumvector

}

void checkWeightless() {

}

unsigned long nextChange = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // Start libs
  Wire.begin(); //I2C lib
  myservo.attach(16);  // attaches servo on pin 2 (D4) to the servo object

  // Setup sensors
  adxlSetup();
  
  // Setup pins
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
  
  // Start serial and introduce project
  Serial.begin(115200);
  Serial.println("Started");
}

// the loop function runs over and over again forever
void loop() {
  
  if (nextChange <= millis()) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
 
    // Get acelerometer data
    readAdxl();
    // Display the results (acceleration is measured in g)
    Serial.print("X: "); Serial.print(x); Serial.print(" ");
    Serial.print("Y: "); Serial.print(y); Serial.print(" ");
    Serial.print("Z: "); Serial.print(z); Serial.print(" ");
    Serial.print("SUM: "); Serial.print(sum); Serial.println("g");

    nextChange = nextChange + 1000;
  }
  
}