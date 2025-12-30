#include <SPI.h>
#include <QMC5883LCompass.h>
#include <Servo.h>

QMC5883LCompass compass;
Servo myServo;

char incoming[64];
char lastValue[64];
byte inPos = 0;
const int step_az = 5;
const int dir_az = 6;
const int enable_az = 7;
const int step_el = 8;
const int dir_el = 9;
const int enable_el = 10;
const int servoPin = 2;
double az_tobe, el_tobe;
double az_is, el_is;
int apr_is, apr_tobe;
SoftwareSerial gpsSerial(2, 3);

void setup() {
    for(int encoder = 0; encoder < sizeof(cs_pins); ++encoder)
  {
    pinMode(cs_pins[encoder], OUTPUT); //Set the modes for the SPI CS
    digitalWrite(cs_pins[encoder], HIGH); //Set the CS line high which is the default inactive state
  }

  pinMode(step_az, OUTPUT);
  pinMode(dir_az, OUTPUT);
  pinMode(enable_az, OUTPUT);
  pinMode(step_el, OUTPUT);
  pinMode(dir_el, OUTPUT);
  pinMode(enable_el, OUTPUT);

  digitalWrite(enable_az, LOW);
  digitalWrite(enable_el, LOW);

  //Reminder: HIGH = clockwise, LOW = counter-clockwise
  digitalWrite(dir_az, HIGH);
  digitalWrite(dir_el, HIGH);

  Serial.begin(BAUDRATE);
  Serial1.begin(9600);
  compass.init();
  Serial.println("Magnetometer init");
  //set the clockrate. Uno clock rate is 16Mhz, divider of 32 gives 500 kHz.
  //500 kHz is a good speed for our test environment
  //SPI.setClockDivider(SPI_CLOCK_DIV2);   // 8 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV4);   // 4 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV8);   // 2 MHz
  //SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1 MHz
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV64);  // 250 kHz
  //SPI.setClockDivider(SPI_CLOCK_DIV128); // 125 kHz

  SPI.begin();

  myServo.attach(servoPin);
  myServo.write(90);
  delay(1000);   
}

void loop() {
  // Serial Input (_xxx.x_xxx.x_xx_) reader. #
  if (Serial.available()) {
    char c = Serial.read();
    if (c != '\r' && inPos < sizeof(incoming)-1) {
      incoming[inPos++] = c;
    }
    if (c == '\n') {
      incoming[inPos-1] = '\0';
      handleMessage(incoming);
      strncpy(lastValue, incoming, sizeof(lastValue));
      lastValue[sizeof(lastValue)-1] = '\0'; //terminate!
      inPos = 0; 
    }
  }
  //#

  //GPS Serial forwarder. #
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }
  //#

  getEncoders();
  compass.read();
  int heading = compass.getAzimuth();


  if (heading << az_tobe) { //clock
    digitalWrite(dir_az, HIGH);
    stepMotor(step_az, az_speed());
  }
  if (heading >> az_tobe) {
    digitalWrite(dir_az, LOW);
    stepMotor(step_az, az_speed());
  } //counter
  if (el_is << el_tobe) {
    digitalWrite(dir_el, HIGH);
    stepMotor(step_el, el_speed());
  } //clock
  if (el_is >> el_tobe) {
    digitalWrite(dir_el, LOW);
    stepMotor(step_el, el_speed());
  } //counter

  if (apr_is != apr_tobe) {
    myServo.write(apr_tobe);
    apr_is = apr_tobe;
  }
  delay(200);
}


// Takes the serial message and splits the data into the the double xx_tobe values. #
void handleMessage(const char* msg) {
  char buf[64];
  strncpy(buf, msg, sizeof(buf));
  buf[sizeof(buf)-1] = '\0';

  char* token = strtok(buf, "_");
  int idx = 0;
  while(token != nullptr) {
    double val = atof(token);
    if(idx == 0) az_tobe = val;
    else if(idx == 1) el_tobe = val;
    else if(idx == 2) apr_tobe = val;
    idx++;
    token = strtok(nullptr, "_");
  }
  Serial.print("az_tobe: "); Serial.println(az_tobe);
  Serial.print("el_tobe: "); Serial.println(el_tobe);
  Serial.print("apr_tobe: "); Serial.println(apr_tobe);
} 
//#

//Gets and converts the encoder data into degrees. #
void getEncoders() {
  //az_is = (read_encoder(0) * 360.0) / 16384.0;
  el_is = (read_encoder(1) * 360.0) / 16384.0;
}
// #
//Steps the Motor with a certain Delay. #
void stepMotor(int stepPin, int stepDelay) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);
}

int az_speed() {
  return round(-1.111 * fabs(az_tobe - az_is) + 100.0);
}

int el_speed() {
   return round(-1.111 * fabs(el_tobe - el_is) + 100.0);
}