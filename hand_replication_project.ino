#include <ESP32Servo.h>   // servo motors for fingers on the ESP32
#include <AccelStepper.h> // library for easy interfacing of the NEMA-17 stepper motor
#include <SPI.h>          // spi protocol for 4 fingers
#include <Wire.h>         // i2c protocol for thumb

#include "mainhtml.h"     // html file that constantly runs via handler function
#include "secrets.h"      // wifi name and password

#include <WiFi.h>         // for connecting to local wifi and allowing people to connect to its wifi
#include <WebServer.h>    // for displaying hand regions on webserver

#define thumbmotorpin 25
#define indexmotorpin 26
#define middlemotorpin 27
#define ringmotorpin 33
#define pinkymotorpin 32
// finger definitions 

#define indexCS 5
#define middleCS 17
#define ringCS 15
#define pinkyCS 16

#define vspiCLK 18
#define vspiMISO 19
#define vspiMOSI 23

#define SDA 21
#define SCL 22 
// communication definitions

#define xAxis 34
#define yAxis 35
#define zAxis 36
// wrist sensor definitions

#define dirPin 12
#define stepPin 14
int x = 0;
int y = 0;
int z = 0;
const int stepsPerRevolution = 2048;
AccelStepper wriststepper(AccelStepper::DRIVER, stepPin, dirPin); 
// stepper motor definition

Servo thumbservo;
Servo indexservo;
Servo middleservo;
Servo ringservo;
Servo pinkyservo;       // create servo instances

WebServer myServer(80); // create webserver on port 80

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
bool uprightState = true;
bool turnRight = false;
bool turnLeft = false;
bool turnFull = false;

String currentState = "";
String wristState = "";

void keepServerOn() {
  myServer.send_P(200, "text/html", htmlfile);
}

void selector() {
  myServer.send(200, "text/plain", currentState);
}

void wristSelect() {
  myServer.send(200, "text/plain", wristState);
}

class i2cmotor { // i2c comms class for thumb

  public:

    i2cmotor(uint8_t input) : address(input) {}

    void startReading() {  

      writeReg(0x2D, 0x08);

    }

    void readData(int16_t &x, int16_t &y, int16_t &z) {        // data transmission

      Wire.beginTransmission(address); // initialize I2C Bus
      Wire.write(0x32);                // point to x0 data bit to start reading
      Wire.endTransmission(false); // end transaction

      Wire.requestFrom(address, 6); // grab the 6 reads to send to sensor

      uint8_t x0 = Wire.read();
      uint8_t x1 = Wire.read();
      uint8_t y0 = Wire.read();
      uint8_t y1 = Wire.read();
      uint8_t z0 = Wire.read();
      uint8_t z1 = Wire.read();

      x = (int16_t)((x1 << 8) | x0);
      y = (int16_t)((y1 << 8) | y0);
      z = (int16_t)((z1 << 8) | z0);

    }

  private:
  
    uint8_t address;

    void writeReg(uint8_t reg, uint8_t val) {

      Wire.beginTransmission(address);

      Wire.write(reg); 
      Wire.write(val);

      Wire.endTransmission();

    }

};

class spimotor { // spi communication class for index-pinky finger

  public:

    spimotor(uint8_t input) : csPin(input) {}

    void startReading() {         // chooses cs pin
      pinMode(csPin, OUTPUT);
      digitalWrite(csPin, HIGH);

      writeReg(0x2D, 0x08);
    }

    void readData(int16_t &x, int16_t &y, int16_t &z) {        // data transmission

      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // initialize SPI Bus
      digitalWrite(csPin, LOW); // select chip for data transfer

       SPI.transfer(0x80 | 0x40 | 0x32);

      uint8_t x0 = SPI.transfer(0x00);
      uint8_t x1 = SPI.transfer(0x00);
      uint8_t y0 = SPI.transfer(0x00);
      uint8_t y1 = SPI.transfer(0x00);
      uint8_t z0 = SPI.transfer(0x00);
      uint8_t z1 = SPI.transfer(0x00);

      digitalWrite(csPin, HIGH); // deselect the chip

      SPI.endTransaction(); // end transaction

      x = (int16_t)((x1 << 8) | x0);
      y = (int16_t)((y1 << 8) | y0);
      z = (int16_t)((z1 << 8) | z0);

    }

  private:
  
    uint8_t csPin;

    void writeReg(uint8_t reg, uint8_t val) {

      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
      digitalWrite(csPin, LOW);
      SPI.transfer(reg & 0x3F); 
      SPI.transfer(val);
      digitalWrite(csPin, HIGH);
      SPI.endTransaction();

    }

};

i2cmotor thumbfinger(0x53);
spimotor indexfinger(indexCS);
spimotor middlefinger(middleCS);
spimotor ringfinger(ringCS);
spimotor pinkyfinger(pinkyCS);        // create objects for the five motors (spi & i2c)

void setup() {

  pinMode(xAxis, INPUT);
  pinMode(yAxis, INPUT);
  pinMode(zAxis, INPUT); // analog reads for wrist accelerometer

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT); // for the NEMA-17 stepper motor

  wriststepper.setMaxSpeed(600);
  wriststepper.setAcceleration(1200);
  SPI.begin(vspiCLK, vspiMISO, vspiMOSI, -1);

  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid);
  WiFi.softAP(userSSID, userPassword);

  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    delay(500);
  }
  
  Serial.print("Wifi IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("ESP32 Hotspot IP: ");
  Serial.println(WiFi.softAPIP());

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  thumbservo.setPeriodHertz(50);
  indexservo.setPeriodHertz(50);
  middleservo.setPeriodHertz(50);
  ringservo.setPeriodHertz(50);
  pinkyservo.setPeriodHertz(50);

  thumbservo.attach(thumbmotorpin, SERVO_MIN_US, SERVO_MAX_US);
  indexservo.attach(indexmotorpin, SERVO_MIN_US, SERVO_MAX_US);
  middleservo.attach(middlemotorpin, SERVO_MIN_US, SERVO_MAX_US);
  ringservo.attach(ringmotorpin, SERVO_MIN_US, SERVO_MAX_US);
  pinkyservo.attach(pinkymotorpin, SERVO_MIN_US, SERVO_MAX_US);

  thumbservo.write(180);
  indexservo.write(10);
  middleservo.write(90);
  ringservo.write(0);
  pinkyservo.write(45);

  Wire.begin();
  thumbfinger.startReading();
  indexfinger.startReading();
  middlefinger.startReading();
  ringfinger.startReading();
  pinkyfinger.startReading();


  myServer.on("/", keepServerOn);
  myServer.on("/select", selector);
  myServer.on("/wrists", wristSelect);
  myServer.begin(); 
}

void loop() {

  myServer.handleClient();                     // checks for any incoming http requests and acts correspondingly

  /*
    read in wrist accelerometer values ( x = 1330 & y = 1900 (default position), x = 1950 & y = 2450 (90 degrees to the right),
                                   x = 2450 & y = 1900 (hand flipped over 180), x = 1900 & y = 1330 (90 degrees to the left) )
  */ 

  int direction = 0;
  x = analogRead(xAxis);
  y = analogRead(yAxis);

  x = constrain(x, 1330, 2450);
  y = constrain(y, 1330, 2450);

  wristState = "";
  direction = 0;

  int yNormal = 1850;
  int rightYRange = 350;
  int leftYRange = 200;
  
  if ( (y - yNormal) >= rightYRange) {
    wristState = "rightwrist";
    direction = 1;
  }
  else if ( (yNormal - y) >= leftYRange) {
    wristState = "leftwrist";
    direction = -1;
  }
  else {
    wristState = "";
    direction = 0;
  }

  // set stepper motor speed/dir
  if (direction == 1) { wriststepper.setSpeed(-150); }
  else if (direction == -1) { wriststepper.setSpeed(150); }
  else { wriststepper.setSpeed(0); }

  // read in finger accelerometer values
  static int16_t tx, ty, tz;
  static int16_t ix, iy, iz;
  static int16_t mx, my, mz;
  static int16_t rx, ry, rz;
  static int16_t px, py, pz;

  static unsigned long nowTwo = 0;
  if ((millis() - nowTwo) >= 50) {

    thumbfinger.readData(tx, ty, tz);
    indexfinger.readData(ix, iy, iz);
    middlefinger.readData(mx, my, mz);
    ringfinger.readData(rx, ry, rz);
    pinkyfinger.readData(px, py, pz);

    nowTwo = millis();  
  }

  // map values to sensor movement
  tx = constrain(tx, 0, 110);
  int thumbval = map(tx, 0, 110, 180, 0);
  thumbservo.write(thumbval);  

  ix = constrain(ix, 0, 245);
  int indexval = map(ix, 0, 245, 180, 10);
  indexservo.write(indexval);

  mx = constrain(mx, 0, 245);
  int middleval = map(mx, 0, 245, 180, 0);
  middleservo.write(middleval);

  rx = constrain(rx, 0, 245);
  int ringval = map(rx, 0, 245, 180, 0);
  ringservo.write(ringval);

  px = constrain(px, 0, 245);
  int pinkyval = map(px, 0, 245, 90, 180);
  pinkyservo.write(pinkyval);

  if (thumbval < 170) { currentState = "thumb"; }
  else if (indexval < 170) { currentState = "indexfingerhtml"; }
  else if (middleval < 170) { currentState = "middle"; }
  else if (ringval < 170) { currentState = "ring"; }
  else if (pinkyval > 135) { currentState = "pinky"; }
  else { currentState = ""; }

  //Serial.print("(wrist) X: "); Serial.print(x);
  //Serial.print(" Y: "); Serial.println(y);

  //Serial.print("Thumb X: "); Serial.println(tx);
  //Serial.print("Index X: "); Serial.println(ix);
  //Serial.print("Middle X: "); Serial.println(ix);
  //Serial.print("Ring X: "); Serial.println(ix);
  //Serial.print("pinky angle: "); Serial.println(pinkyval);

  wriststepper.runSpeed(); // constatly check for stepper motor speed
}