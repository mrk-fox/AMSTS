/*
 * AMT22_Multiurn_SPI_Sample_Code_Uno.ino
 * Company: CUI Devices
 * Author: Jason Kelly, Damon Tarry
 * Version: 2.0.2.0
 * Date: July 18, 2023
 *
 * This sample code can be used with the Arduino Uno to control the AMT22 encoder.
 * It uses SPI to control the encoder and the the Arduino UART to report back to the PC
 * via the Arduino Serial Monitor.
 * For more information or assistance contact CUI Devices for support.
 *
 * After uploading code to Arduino Uno open the open the Serial Monitor under the Tools
 * menu and set the baud rate to 115200 to view the serial stream the position from the AMT22.
 *
 * Arduino Pin Connections
 * SPI Chip Select Enc 0:   Pin  2
 * SPI Chip Select Enc 1:   Pin  3
 * SPI MOSI                 Pin 11
 * SPI MISO                 Pin 12
 * SPI SCLK:                Pin 13
 *
 *
 * AMT22 Pin Connections
 * Vdd (5V):                Pin  1
 * SPI SCLK:                Pin  2
 * SPI MOSI:                Pin  3
 * GND:                     Pin  4
 * SPI MISO:                Pin  5
 * SPI Chip Select:         Pin  6
 */


/* Include the SPI library for the arduino boards */
#include <SPI.h>

/* Serial rates for UART */
#define BAUDRATE        115200

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_ZERO      0x70
#define AMT22_TURNS     0xA0

/* We will use this define macro so we can write code once compatible with 12 or 14 bit encoders */
#define RESOLUTION      14

//create an array containing CS pin numbers for all connected encoders
uint8_t cs_pins[] = {3}; //only one encoder connected, using pin 2 on arduino for CS. Two encoders connected, using pins 2 & 3 on arduino for CS

uint8_t read_encoder(int enc_num)
{
  uint8_t cs_pin = cs_pins[enc_num];

  //set the CS signal to low
  digitalWrite(cs_pin, LOW);
  delayMicroseconds(3);

  //read the two bytes for position from the encoder, starting with the high byte
  uint16_t encoderPosition = SPI.transfer(AMT22_NOP) << 8; //shift up 8 bits because this is the high byte
  delayMicroseconds(6);

  //set the CS signal to high
  digitalWrite(cs_pin, HIGH);

  if (verifyChecksumSPI(encoderPosition)) //position was good, print to serial stream
  {
    encoderPosition &= 0x3FFF; //discard upper two checksum bits
    if (RESOLUTION == 12) encoderPosition = encoderPosition >> 2; //on a 12-bit encoder, the lower two bits will always be zero

    Serial.print("Encoder #");
    Serial.print(enc_num, DEC);
    Serial.print(" position: ");
    Serial.print(encoderPosition, DEC); //print the position in decimal format
    Serial.write('\n');
    return encoderPosition;
  }
  else //position is bad, let the user know how many times we tried
  {
    Serial.print("Encoder #");
    Serial.print(enc_num, DEC);
    Serial.print(" position error.\n");
  }
}

/*
 * Using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent.
 */
bool verifyChecksumSPI(uint16_t message)
{
  //checksum is invert of XOR of bits, so start with 0b11, so things end up inverted
  uint16_t checksum = 0x3;
  for(int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }
  return checksum == (message >> 14);
}