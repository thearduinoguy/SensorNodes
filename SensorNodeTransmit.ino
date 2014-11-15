//===============================================
// Wireless sensor node transmission code
// ATMega328 with RFM69CW + Sensor
// M.McRoberts 2014
//===============================================

#include <RFM69.h>
#include <SPI.h>
#include <JeeLib.h>
#include <Wire.h>
#include "HTU21D.h"

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

HTU21D myHumidity;

#define NODEID      1
#define NETWORKID   100
#define GATEWAYID   1   // ID of Base Unit
#define FREQUENCY   RF69_433MHZ     // RF69_868MHZ
#define KEY         "CatfishHotdog123"    //has to be same 16 characters/bytes on all nodes, not more not less!
#define LED         9    // LED Pin
#define SERIAL_BAUD 115200   // For serial monitor window only
#define ACK_RETRIES 5       // Number of times to retry if no ack
#define ACK_TIME    30      // # of ms to wait for an ack
#define nodeURN     0xAAAA0002    // 4 byte reference 2 bytes Customer ID 2 bytes Node ID
#define sensorPin   7    // Sensor pin

RFM69 radio;

//===============================================
// Struct for the data packet
//===============================================
typedef struct { 
  long URN;
  int battery;
  float temperature, humidity; 
} PacketTX;      
PacketTX dataPacket; 

//===============================================
//  DEVICE CONFIGURATION
//===============================================
void setup() {
  pinMode(sensorPin, OUTPUT); delay(100);
  digitalWrite(sensorPin, HIGH); delay(100); // Turn the sensor on
  
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  
  // Set the Baud Rate  (9600) 
  radio.writeReg(0x03, 0x0D);
  radio.writeReg(0x04, 0x05);
  
  radio.encrypt(KEY);
  myHumidity.begin();
  Serial.println("SensorNodeTransmit_vB5");
}

//===============================================
// Read current supply voltage
//===============================================
 long readVcc() {
   bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
   long result;
   // Read 1.1V reference against Vcc
   #if defined(__AVR_ATtiny84__) 
    ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
   #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
   #endif 
   delay(2); // Wait for Vref to settle
   ADCSRA |= _BV(ADSC); // Convert
   while (bit_is_set(ADCSRA,ADSC));
   result = ADCL;
   result |= ADCH<<8;
   result = 1126400L / result; // Back-calculate Vcc in mV
   ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
   return result;
} 

//===============================================
// Blink the LED
//===============================================
void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

//===============================================
// Main program loop function
//===============================================
void loop() {  
    digitalWrite(sensorPin, HIGH); delay(5); // Turn the sensor on
    dataPacket.URN = nodeURN;
    dataPacket.humidity = myHumidity.readHumidity();
    dataPacket.temperature =  myHumidity.readTemperature();
    digitalWrite(sensorPin, LOW); // Turn the sensor off
    dataPacket.battery = readVcc(); // Get supply voltage
    
    Serial.print("Sending struct (");
    Serial.print(sizeof(dataPacket));
    Serial.print(" bytes) ... ");
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&dataPacket), sizeof(dataPacket), ACK_RETRIES, ACK_TIME))
      {Serial.print(" ok!");
    }
    else {Serial.print(" nothing...");
  }
    Serial.println();
    Blink(LED,2);
    sleep();
}

//===============================================
// Sleep function
//===============================================
void sleep() {
   Serial.flush(); // Clear the serial buffer
    radio.sleep();  // Put the RFM69CW to sleep
  // go to sleep for approx 60 seconds
  Sleepy::loseSomeTime(60000);
}
