#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPS++.h> 
#include <SoftwareSerial.h>
#include <TimeLib.h> 
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <ThingsBoard.h>
#include <PubSubClient.h> 
#include <ArduinoJson.h>
#include "base64.hpp"

#define time_offset -5*3600
#define BUTTON_PIN_BITMASK 0x200000000

const uint8_t uplInterruptPin = 0;
const uint8_t lsInterruptPin = 21;
const uint8_t rpmInterruptPin = 22;
const uint8_t currentPin = 36;
const uint8_t voltagePin = 2;
const uint8_t redPin = 25;
const uint8_t greenPin = 27  ;
volatile long lsArcs = 0;
volatile long rpmArcs = 0;
volatile unsigned long lastTime = 0;
unsigned long epochTime;
unsigned long wakeupTime = 3600000000;
float lo, la;
uint16_t alt;
byte last_second, Second, Minute, Hour, Day, Month;
int Year = 0;
String tbHost = "thingsboard.cloud";
String tbToken = "frVxgGALjXbuhbWsNqGP";
unsigned char sdBuffer[147];
unsigned char base64[196];
char * dataTB64;
uint8_t uplHour = 22;
uint16_t i = 0;
boolean upl = false;
boolean uplAux = false;
boolean sleepFlag = false;
union trama_tag
{
  struct
  {
    uint32_t t;
    uint32_t o;
    uint32_t a;
    uint16_t v;
    uint16_t c;
    uint16_t l;
    uint16_t h;
    uint8_t r;
  };
  unsigned char all[21];
}trama;

const float radio = 0.4, pi = 3.141592;
volatile uint16_t delta_arcs = 0;
volatile long last_arcs = 0;
float lineal_speed = 0;

volatile uint16_t delta_rpmArcs = 0;
volatile long last_rpmArcs = 0;
float rpm = 0;

float voltageRef = 3.3; // Voltaje de referencia del ADC
float sensibility = 0.06; // Sensibilidad del sensor de corriente (en mV/A)
uint16_t samples = 0; // Contador de muestras tomadas
uint16_t n_samples = 100;
const float currentError = 1.38; // Valor del error de corriente (ajustable para calibrar el sensor)
volatile float currentSensor = 0; // Valor de la lectura del sensor
float current = 0; // Valor de la corriente
float current2 = 0; // Variable de almacenamiento

volatile float voltage_sensor = 0;
float voltage = 0;
float voltage2 = 0;

char btBuffer[15];
 
TaskHandle_t saveTaskHandle = NULL;
TaskHandle_t uploadTaskHandle = NULL;

TinyGPSPlus gpsDate;
SoftwareSerial SerialGPS(16, 17);
BluetoothSerial SerialBT;
WiFiClient espClient;
ThingsBoardSized<256> tb(espClient);

void IRAM_ATTR uplISR(){
  upl = true;
}

void IRAM_ATTR lsISR(){
  if(millis() - lastTime > 25){
    lsArcs = lsArcs+1;
    lastTime = millis();
  } 
}

void IRAM_ATTR rpmISR(){
  if(millis() - lastTime > 25){
    rpmArcs = rpmArcs+1;
    lastTime = millis();
  } 
}

void error(){
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
}

void ok(){
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
}

void waiting(uint16_t t){
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);
  delay(t/2);
  digitalWrite(greenPin, LOW);
  delay(t/2);
}

void connectSD(){
  if(!SD.begin(5)){
    error();
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    error();
    return;
  }
}

void connectWiFi(){
  const char* ssid = "MICRORRED";
  const char* pswd = "Microrred20181213";
//  const char* ssid = "Jh";
//  const char* pswd = "joel1662";
  WiFi.begin(ssid, pswd);
  uint8_t i_wifi = 0;
  while(WiFi.status() != WL_CONNECTED && i_wifi <= 120){
    Serial.print(".");
    waiting(500);
    i_wifi = i_wifi + 1;
  }
  ok();
}

void gps(){
  while(SerialGPS.available()){ 
    if(gpsDate.encode(SerialGPS.read())){
      la = gpsDate.location.lat();
      lo = gpsDate.location.lng();
      alt = gpsDate.altitude.meters();
      if(gpsDate.time.isValid()){
        Minute = gpsDate.time.minute();
        Second = gpsDate.time.second();
        Hour   = gpsDate.time.hour();
      }
      if(gpsDate.date.isValid()){
        Day   = gpsDate.date.day();
        Month = gpsDate.date.month();
        Year  = gpsDate.date.year();
      }
      if(last_second != gpsDate.time.second()){
        last_second = gpsDate.time.second();
        setTime(Hour, Minute, Second, Day, Month, Year);
        adjustTime(time_offset);
        epochTime = now();
        Serial.println(epochTime);
        signalRead();
      }
    }
  }
}

void adc(){
  adcAttachPin(currentPin);
  adcAttachPin(voltagePin);
  samples = samples + 1;
  currentSensor = analogRead(currentPin) * (3.3 / 4096.0); 
  voltage_sensor = analogRead(voltagePin) * (0.00123);
  current += ((currentSensor - currentError) / sensibility);
  voltage += voltage_sensor; 
  if (samples == n_samples) {
    current /= samples;
    voltage /= samples;
    current2 = current;
    voltage2 = voltage;
    
    Serial.println(voltage2);
    samples = 0;
    current = 0;
    voltage = 0;
  }
}

void appendFile(){
  File file = SD.open("/data.txt", FILE_APPEND);
  if(!file){
    error();
    return;
  }
  if(file.write(trama.all, sizeof(trama.all))){
     ok();
  }else{
     error();
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char * path){
  if(fs.remove(path)){
    error;
  }else{
    error;
  }
}
void readFile(fs::FS &fs, const char * path){
  File file = fs.open(path);
  if(!file){
    error;
    return;
  }
  while(file.available()){
    sleepFlag = false;
    if(!tb.connected()){
        tbReconnect();
    } 
    memset(sdBuffer, 0 , sizeof(sdBuffer));
    file.read(sdBuffer, 147);
    unsigned int base64_length = encode_base64(sdBuffer, 147, base64);
    dataTB64 = (char *)base64;
    Serial.println(dataTB64);
    tb.sendTelemetryString("array", dataTB64);
    waiting(250);
  }
  file.close();
}

void tbReconnect(){
  uint8_t i_tb = 0;
  while(!tb.connected() && i_tb <= 3){
    if(WiFi.status() != WL_CONNECTED){
      connectWiFi();
    }
    if (tb.connect(tbHost.c_str(),tbToken.c_str())){
      ok(); 
    }else{
      error();
    }
  i_tb = i_tb + 1;
  }
}

void signalRead(){
  delta_arcs = lsArcs - last_arcs;
  last_arcs = lsArcs;
  lineal_speed = ((7.2/7)*pi*radio*delta_arcs);
  if(lineal_speed < 1){
    i++;
  }else{
    i = 0;
    sleepFlag = false;
  }
  if(i >= 180){
    sleepFlag = true;
  }
  ("l" + String(lineal_speed, 2)).toCharArray(btBuffer, 8);
  SerialBT.println(btBuffer);
  
  delta_rpmArcs = rpmArcs - last_rpmArcs;
  last_rpmArcs = rpmArcs;
  rpm = ((60/14)*delta_rpmArcs);
  ("r" + String(int(rpm))).toCharArray(btBuffer, 8);
  SerialBT.println(btBuffer);

  ("c" + String(current2, 4)).toCharArray(btBuffer, 8);
  SerialBT.println(btBuffer);

  ("v" + String(voltage2, 2)).toCharArray(btBuffer, 8);
  SerialBT.println(btBuffer);
  Serial.println(voltage);
  
  trama.l = uint16_t(lineal_speed * 100);
  trama.r = uint16_t(rpm);
  trama.c = uint16_t(current2 * 1000);
  trama.v = uint16_t(voltage2 * 100);
  trama.h = uint16_t(alt);
  trama.a = uint32_t(la * 10000000);
  trama.o = uint32_t(lo * 10000000);
  trama.t = epochTime;

  appendFile();
}
void saveTask(void *pvParameters)  
{
  (void) pvParameters;
  
  for (;;)
  {
    adc();
    gps();
    vTaskDelay( 10 / portTICK_PERIOD_MS ); 
  }
}

void uploadTask(void *pvParameters)  
{
  (void) pvParameters;
  
  
  for (;;)
  {
  if(uplHour == hour() && uplAux == false){
    upl = true;
    uplAux = true;
  }
  if(upl){
    vTaskSuspend(saveTaskHandle);
    connectWiFi();
    if(WiFi.status() == WL_CONNECTED){
      readFile(SD, "/data.txt");
      deleteFile(SD, "/data.txt");
    }
    upl = false;
    WiFi.disconnect();
    vTaskResume(saveTaskHandle);
    ESP.restart();
  }
  if(sleepFlag){
    i = 0;
    uplAux = false;
    esp_deep_sleep_start();
  } 

  vTaskDelay( 1000 / portTICK_PERIOD_MS ); 
  }
}

void setup() {
  
  Serial.begin(115200);
  SerialGPS.begin(9600);
  SerialBT.begin("ebikeUC04");

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(uplInterruptPin, INPUT_PULLUP);
  pinMode(lsInterruptPin, INPUT_PULLUP);
  pinMode(rpmInterruptPin, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(uplInterruptPin), uplISR, RISING);
  attachInterrupt(digitalPinToInterrupt(lsInterruptPin), lsISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rpmInterruptPin), rpmISR, RISING);

  xTaskCreatePinnedToCore(saveTask, "save", 4096, NULL, 1, &saveTaskHandle, 1);
  xTaskCreatePinnedToCore(uploadTask, "upload", 4096, NULL, 1, &uploadTaskHandle, 0);
  
  esp_sleep_enable_timer_wakeup(wakeupTime);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);

  connectSD();
}

void loop() {
 
}
