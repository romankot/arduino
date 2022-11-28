#include <SoftwareSerial.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFiMulti.h>
//#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 0 on the Arduino
#define ONE_WIRE_BUS 0 //D3
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

ESP8266WiFiMulti wifiMulti;

const char* ssid = "netis_2.4G_52C760";
const char* password = "password";
const char* ssid2 = "kremor";
const char* password2 = "12345678";

WiFiClient client; // All fucntions https://www.arduino.cc/en/Reference/WiFi & http://esp8266.github.io/Arduino/versions/2.0.0/doc/libraries.html#wifi-esp8266wifi-library
#define ledPin 2 //D4         // the onboard LED
#define yellowPin 4 //D2
#define greenPin 5 //D1
#define redPin 14 //D5
#define RGBbluePin 2   //D4 not connected as it messedup with 
#define RGBgreenPin 16 //D0
#define RGBredPin 12   //D6

unsigned long CO2Channel = 373401;
const char * CO2WriteAPIKey = "TFXJ7CNV566GSD1N";
unsigned long CO2Millis = 31 010;
unsigned long MillisLast = 0;

SoftwareSerial mySerial(13, 15); // RX,TX
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char response[9];
char temperatureCString[7];

void setup() {
  pinMode(ledPin, OUTPUT);  
  pinMode(RGBbluePin, OUTPUT);
  pinMode(RGBgreenPin, OUTPUT);
  pinMode(RGBredPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  Serial.begin(115200);
  Serial.println("");
  mySerial.begin(9600);
  Serial.setDebugOutput(true);
  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP(ssid2, password2);
  //wifiMulti.addAP("SS-MOBILE");
  if (int stat = wifiMulti.run() != WL_CONNECTED) {
    Serial.print("WiFi not connected! run returned: ");
    Serial.println(stat);
  }

//  sensors.begin();
  Serial.println("WiFi is ready...");  int lo = WiFi.RSSI();
  Serial.print("WiFi signal: ");  Serial.println(lo);
  Serial.println(WiFi.localIP());

  digitalWrite(ledPin, HIGH);
  Serial.println("Let us go....");
//  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  //Serial.print("Device 0 Address: ");
  //printAddress(insideThermometer);
  //Serial.println();
  
  delay(2000);
}
void checkTemp(DeviceAddress deviceAddress) {
  float tempC;
  float tempF;
  do {
//    sensors.requestTemperatures(); 
//    tempC = sensors.getTempCByIndex(0);
//    dtostrf(tempC, 2, 2, temperatureCString);    
//    delay(100);
  } while (tempC == 85.0 || tempC == (-127.0));
  
}
int getCO2Data() {
  mySerial.write(cmd, 9);
  memset(response, 0, 9);
  mySerial.readBytes(response, 9);

  // CRC check
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc += response[i];
  crc = 255 - crc;
  crc++;
  // End of CRC check
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    //Serial.println("CRC error: " + String(crc) + " / " + String(response[8]));
    char raw[32];
    sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X", response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7], response[8]);
    //Serial.println(raw);
    MillisLast = CO2Millis-20000;
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    int ppm = (256 * responseHigh) + responseLow;
    int temp = response[4] - 40;
    checkTemp(insideThermometer);
    Serial.println(ppm);
    char raw[32];
    sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X", response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7], response[8]);
    if (ppm <= 400 || ppm > 4900) {
      Serial.println("CO2: no valid data");
      MillisLast = CO2Millis;
    } else {
      Serial.println("CO2 PPM:" + String(ppm) + "; Temp:" + String(temp));
      digitalWrite(ledPin, LOW);
      String sURL = "http://api.thingspeak.com/update?api_key=";
      sURL += CO2WriteAPIKey;
      sURL += "&field1=" + String(ppm) + "&field2=" + String(temp) + "&field3=" + String(temperatureCString);
      Serial.println(sURL);
      HTTPClient http;
      http.begin(sURL);
      int httpCode = http.GET();
      if (httpCode == HTTP_CODE_OK) {
        Serial.printf("ThingSpeak responce code: %d\n", httpCode);
      } else {
        Serial.println("ThingSpeak failed, error: " + String(http.errorToString(httpCode).c_str()));
        MillisLast = CO2Millis-10000;
      }
      digitalWrite(ledPin, HIGH);
      return ppm;
    }
  }
}

void makeRed() {
  digitalWrite(RGBredPin, LOW);
  digitalWrite(RGBgreenPin, HIGH);
  digitalWrite(RGBbluePin, HIGH);    

  digitalWrite(redPin, HIGH);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);
}
void makeGreen() {
  digitalWrite(RGBredPin, HIGH);
  digitalWrite(RGBgreenPin, LOW);
  digitalWrite(RGBbluePin, HIGH); 

   digitalWrite(greenPin, HIGH);
   digitalWrite(redPin, LOW);
   digitalWrite(yellowPin, LOW);
}
void makeYellow() {
  digitalWrite(RGBredPin, LOW);
  digitalWrite(RGBgreenPin, LOW);
  digitalWrite(RGBbluePin, HIGH);  

  digitalWrite(yellowPin, HIGH);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
}
void makeDark() {
  digitalWrite(RGBredPin, HIGH);
  digitalWrite(RGBgreenPin, HIGH);
  digitalWrite(RGBbluePin, HIGH); 
  
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW); 
}
void makeWhite() {
  digitalWrite(RGBredPin, LOW);
  digitalWrite(RGBgreenPin, LOW);
  digitalWrite(RGBbluePin, LOW);  
}
void makeRGBBlink() {

  makeDark();
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW); 
  delay(300);
  digitalWrite(redPin, HIGH);
  digitalWrite(yellowPin, HIGH);
  digitalWrite(greenPin, HIGH);   
  
}
void loop() {
  unsigned long CurrentMillis = millis();
  if (CurrentMillis - MillisLast > CO2Millis || MillisLast == 0) {
    MillisLast = CurrentMillis;
    int pp = getCO2Data();    
    if (pp >= 2500 && pp < 5000) {
      makeRed();
    } else if (pp > 1200 && pp < 2500) {
      makeYellow();
    } else if (pp < 1200 && pp > 410){
      makeGreen();
    } else {
     makeRGBBlink();
    }

  }

  delay(200);
  wifiMulti.run();
}

