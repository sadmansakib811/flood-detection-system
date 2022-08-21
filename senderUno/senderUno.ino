/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com

  Module SX1278 --> Arduino UNO/NANO
    GND         ->   GND    
    Vcc         ->   3.3V
    MISO        ->   D12
    MOSI        ->   D11     
    SLCK        ->   D13
    NSS         ->   D10
    DIO0        ->   D2
    RESET       ->   D9

  Module SX1278 --> ESP32
    ANA: Antenna
    GND: GND
    VCC/3.3V: 3.3V
    MISO: GPIO 19
    MOSI: GPIO 23
    SLCK: GPIO 18
    NSS: GPIO 5
    DIO0: GPIO 2
    RESET: GPIO 14
*********/

#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 10
#define rst 9
#define dio0 2
#define POWER_PIN  4 // UNO pin GIOP4 connected to sensor's VCC pin
#define SIGNAL_PIN A0 // ESP32 pin GIOPA0 (ADC0) connected to sensor's signal pin
int value = 0; // variable to store the sensor value

int counter = 0;
int LED = 7;

String loRaData;

void setup() {
  //initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender");

  pinMode(LED, OUTPUT);

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  //water sensor
  //pinMode(POWER_PIN, OUTPUT);   // configure pin as an OUTPUT
  //digitalWrite(POWER_PIN, LOW); // turn the sensor OFF
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {

  //sensor value
  //digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  //delay(10);                      // wait 10 milliseconds
  value = analogRead(SIGNAL_PIN); // read the analog value from sensor
  //digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);

  Serial.print("The water sensor value: ");
  Serial.println(value);

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
      loRaData = LoRa.readString();
    }
  }
  
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  String alertMessage="Flood at Dortmund!";
  //String sosMessage="I need help at Point B.";
  //String loRaMessage = String(counter) + "/" + String(value) + "&" + String(alertMessage) + "#" + String(sosMessage);
  String loRaMessage = String(counter) + "/" + String(value) + "&" + String(alertMessage);
  LoRa.beginPacket();
  LoRa.print(loRaMessage);
  if(loRaData.length()>0)
    LoRa.print(loRaData);
  LoRa.endPacket();

  

  counter++;

  delay(10000);
}
