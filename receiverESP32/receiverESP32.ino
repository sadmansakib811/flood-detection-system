/*********
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
#include <EasyBuzzer.h>
#include <U8g2lib.h>
#include <Wire.h> 
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "time.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

#define WIFI_SSID "ME!"
#define WIFI_PASSWORD "11223344"

char buff[10];
#define FIREBASE_HOST "https://floods-6836f-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "1VQX63bpUJC6diwlSeUccxr5aqYlzInr4d6Ximmh"

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPLhErdcna7"
#define BLYNK_DEVICE_NAME "House A"

#define BLYNK_FIRMWARE_VERSION  "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
  #define USE_ESP32_DEV_MODULE
//#define USE_ESP32S2_DEV_KIT

#include "BlynkEdgent.h"

U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 4, /* data=*/ 22, /* CS=*/ 17, /* reset=*/ 21); // ESP32

//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

//buzzer 
#define BUZZER 25 //buzzer to arduino pin 25
int LED =16;
int BUTTON = 15;

int buttonState = LOW;
int lastButtonState = LOW;


String identifier;
int sensorValue = 0;
String alertMessage;
String sosMessage;

//Define FirebaseESP32 data object
FirebaseData firebaseData;
FirebaseJson json;

void setup() {
  //initialize Serial Monitor
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println(F("LoRa Initializing OK!"));

  //buzzer
  EasyBuzzer.setPin(BUZZER);
  
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  
  u8g2.begin();

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontMode(0);    // enable transparent mode, which is faster

  //BlynkEdgent.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
 
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
 
  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
 
  /*
  This option allows get and delete functions (PUT and DELETE HTTP requests) works for device connected behind the
  Firewall that allows only GET and POST requests.
  
  Firebase.enableClassicRequest(firebaseData, true);
  */
 
  //String path = "/data";
  Serial.println("------------------------------------");
  Serial.println("Connected...");

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(3600);
}

void loop() {
  /* Always call this function in the loop for EasyBuzzer to work. */
  EasyBuzzer.update();
  //BlynkEdgent.run();

  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      //int value = LoRaData.toInt();
      Serial.println(LoRaData); 

      digitalWrite ( LED , HIGH );
      delay(500);
      digitalWrite ( LED , LOW );

      // Get readingID
      int pos1 = LoRaData.indexOf('/');
      int pos2 = LoRaData.indexOf('&');
      int posSOS = LoRaData.indexOf("SOS:");

      identifier = LoRaData.substring(0, pos1);
      sensorValue= LoRaData.substring(pos1 +1, pos2).toInt();
      alertMessage = LoRaData.substring(pos2+1, LoRaData.length());
      if(posSOS>=0)
      //sosMessage = LoRaData.substring(posSOS+4, LoRaData.length());
      sosMessage = LoRaData.substring(0, LoRaData.length());
      
      Serial.println("id: "+identifier);
      Serial.println("sensor value: "+String(sensorValue));

      u8g2.firstPage();
    
      if(sensorValue>600){     
        Serial.println("flood alert: "+alertMessage);

         String line1= alertMessage.substring(0, 9);
         String line2= alertMessage.substring(9);
         
          do {
            u8g2.setCursor(0, 10);
            u8g2.println("Alert: "+line1);
            u8g2.setCursor(0, 20);
            u8g2.println(line2);
          } while ( u8g2.nextPage() );

         //Blynk.virtualWrite(V0, HIGH); //flood status
         //Blynk.virtualWrite(V1, alertMessage); //flood alert

         String formattedDate = timeClient.getFormattedDate();
         json.clear();
         json.set("/floodStatus", true);
         json.set("/floodAlert", alertMessage);
         Firebase.updateNode(firebaseData,"/data/House B/Alert/"+formattedDate,json);
         
        /* Single beep. */
        EasyBuzzer.singleBeep(
          1000,  // Frequency in hertz(HZ).
          500  // Duration of the beep in milliseconds(ms).
        );

      }else{
        EasyBuzzer.stopBeep();
        
        do {
          u8g2.setCursor(0, 10);
          u8g2.println(F("No Flood :)"));
        } while ( u8g2.nextPage() );

         //Blynk.virtualWrite(V0, LOW); //flood status
         
          String formattedDate = timeClient.getFormattedDate();

          json.clear();
         json.set("/floodStatus", false);
         Firebase.updateNode(firebaseData,"/data/House B/Alert/"+formattedDate,json);
      }

      if(sosMessage.length()>0){
         Serial.println("sos message: "+sosMessage);
         
         String line1= sosMessage.substring(0,19);
         String line2= sosMessage.substring(19,40);
         String line3= sosMessage.substring(40,60);
          do {
            u8g2.setCursor(0, 30);
            u8g2.println(line1);
            u8g2.setCursor(0, 40);
            u8g2.println(line2);
            u8g2.setCursor(0, 50);
            u8g2.println(line3);
            //printText(sosMessage);
          } while ( u8g2.nextPage() );

          //Blynk.virtualWrite(V2, sosMessage); //flood alert
          String formattedDate = timeClient.getFormattedDate();
          json.clear();
          json.set("/received", sosMessage);
          Firebase.updateNode(firebaseData,"/data/House B/SOS/"+formattedDate,json);
      }
      // print RSSI of packet
      Serial.print("' with RSSI ");
      Serial.println(LoRa.packetRssi());
      delay(1000);
    }
  }

  buttonState = digitalRead(BUTTON); // read new state
  if(buttonState==HIGH)
    lastButtonState = buttonState;    // save the last state
  //Serial.println("state: "+String(buttonState));
  //Serial.println("last: "+String(lastButtonState));

  if(lastButtonState==HIGH && buttonState == LOW){
    lastButtonState = buttonState;
    
    Serial.println("The button is pressed");
    String sosMessage="SOS:I need help at Y str. House B.";
    
      LoRa.beginPacket();
      LoRa.print(sosMessage);
      LoRa.endPacket();
      Serial.println("sos sent.");

    String formattedDate = timeClient.getFormattedDate();
    json.clear();
    json.set("/sent", sosMessage);
    Firebase.updateNode(firebaseData,"/data/House B/SOS/"+formattedDate,json);
  }
}
