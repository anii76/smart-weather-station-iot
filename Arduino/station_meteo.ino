#include <Tomoto_HM330X.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "192.168.43.169";
int        port     = 1883;

#define uint  unsigned int
#define ulong unsigned long

//Définir les pin
#define PIN_VANE       A6  //Girouette
#define PIN_ANEMOMETER 1 //Anémomètre
#define PIN_RAINGAUGE  0     // Pluviomètre
#define PIN_DHT 3
#define PIN_RAINSENSOR 2
#define PIN_PUSHBUTTON 4

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BMP280 bmp;
Tomoto_HM330X dust_sensor;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
DHT dht(PIN_DHT, DHT11); // digital pin 0 , type DHT11

volatile int numRevsAnemometer = 0; // Incremented in the interrupt
volatile int numDropsRainGauge = 0;
int valAAfficher = 0;
String firstLine;
String secondLine;
//Définir les temps de mesure
#define MSECS_CALC_WIND_SPEED 5000
#define MSECS_CALC_WIND_DIR   2500
#define MSECS_CALC_RAIN_FALL  5000

//Prochains temps de calcul
ulong nextCalcDir;
ulong nextCalcSpeed;
ulong nextCalcRain;
ulong time;


#define NUMDIRS 8
ulong   adc[NUMDIRS] = {45, 65, 85, 130, 165, 210, 230, 260};
String strVals[NUMDIRS] = {"East","South-East","South","North-East","South-West","North","North-West","West"};
int dirVal[NUMDIRS] = {90,135,180,45,225,0,315,270};

void setup() {
  Serial.begin(9600);
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
  Wire.begin();
  start_light_sensor();
  start_dust_sensor();
  start_pressure_sensor();
  dht.begin();
  lcd.init();
  lcd.backlight();
 
  //Kit météo
  pinMode(PIN_VANE, INPUT);
  pinMode(PIN_ANEMOMETER, INPUT_PULLUP);
  pinMode(PIN_RAINGAUGE, INPUT_PULLUP);
  pinMode(PIN_PUSHBUTTON, INPUT_PULLUP);
  pinMode(PIN_RAINSENSOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETER) , countAnemometer, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_RAINGAUGE) , countAnemometer, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_PUSHBUTTON) , lcdChange, FALLING);
  nextCalcSpeed = millis() + MSECS_CALC_WIND_SPEED;
  nextCalcDir   = millis() + MSECS_CALC_WIND_DIR;
  nextCalcRain = millis() + MSECS_CALC_RAIN_FALL;
  
}

float wind_speed, rain_fall, altitude, pressure, dust, light, temperature, humidity;
String wind_dir, isRaining;
int rain, wind_dir_mqtt,global_reading;
void loop() {
  time = millis();
  light = lightValue();
  dust = dustValue();
  temperature = temp_Value();
  humidity = Humidity_Value();
  altitude = alt_Value();
  pressure = pressure_Value();
  rain = digitalRead(PIN_RAINSENSOR);
  if (rain) {
    isRaining = "No";
  }else{
    isRaining = "Yes";
  }
  if (time >= nextCalcSpeed) {
      wind_speed = calcWindSpeed();
      nextCalcSpeed = time + MSECS_CALC_WIND_SPEED;
   }
   if (time >= nextCalcRain) {
      rain_fall = calcRainFall();
      nextCalcRain = time + MSECS_CALC_RAIN_FALL;
   }
  if (time >= nextCalcDir) {
    int indice = calcWindDir();
      wind_dir = strVals[indice];
       wind_dir_mqtt = dirVal[indice];
      nextCalcDir = time + MSECS_CALC_WIND_DIR;
   }
   printValues();
   sendToMQTT();
   afficherLCD();
  delay(500);
}

void printValues(){
   Serial.print("\nLight : ");
   Serial.print(light);
   Serial.print("\ndust : ");
   Serial.print(dust);
   Serial.print("\nTemp : ");
   Serial.print(temperature);
   Serial.print("\nHumidity : ");
   Serial.print(humidity);
   Serial.print("\nPressure : ");
   Serial.print(pressure);
   Serial.print("\nalt : ");
   Serial.print(altitude);
   Serial.print("\nwind speed : ");
   Serial.print(wind_speed);
   Serial.print("\nwind dir : ");
   Serial.print(wind_dir);
   Serial.print("\nrain sensor : ");
   Serial.print(isRaining);
   Serial.print("\nrain gauge: ");
   Serial.print(rain_fall);
}

void sendToMQTT(){
   mqttClient.beginMessage("light");
   mqttClient.print(light);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("dust");
   mqttClient.print(dust);
   mqttClient.endMessage();

   mqttClient.beginMessage("temperature");
   mqttClient.print(temperature);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("humidity");
   mqttClient.print(humidity);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("pressure");
   mqttClient.print(pressure);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("altitude");
   mqttClient.print(altitude);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("windSpeed");
   mqttClient.print(wind_speed);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("windDir");
   mqttClient.print(wind_dir);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("windDirection");
   mqttClient.print(wind_dir_mqtt);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("raining");
   mqttClient.print(isRaining);
   mqttClient.endMessage();
   
   mqttClient.beginMessage("rain");
   mqttClient.print(rain_fall);
   mqttClient.endMessage();
}

void afficherLCD(){
  switch (valAAfficher) {
    case 0:
      firstLine = "Temp:";
      firstLine += temperature;
      firstLine += " C         ";
      lcd.setCursor(0, 0);
      lcd.print(firstLine);
      secondLine = "Humid:";
      secondLine += humidity;
      secondLine += " %         ";
      lcd.setCursor(0, 1);
      lcd.print(secondLine);
      break;
    case 1:
      firstLine = "Light:";
      firstLine += light;
      firstLine += " Lux           ";
      lcd.setCursor(0, 0);
      lcd.print(firstLine);
      secondLine = "Dust:";
      secondLine += dust;
      secondLine += " mg/m3          ";
      lcd.setCursor(0, 1);
      lcd.print(secondLine);
      break;
    case 2:
      firstLine = "Alt:";
      firstLine += altitude;
      firstLine += " m          ";
      lcd.setCursor(0, 0);
      lcd.print(firstLine);
      secondLine = "Press:";
      secondLine += pressure;
      secondLine += "pa          ";
      lcd.setCursor(0, 1);
      lcd.print(secondLine);
      break;
    case 3:
      firstLine = "Speed:";
      firstLine += wind_speed;
      firstLine += " km/h           ";
      lcd.setCursor(0, 0);
      lcd.print(firstLine);
      secondLine = "Dir:";
      secondLine += wind_dir;
      secondLine += "           ";
      lcd.setCursor(0, 1);
      lcd.print(secondLine);
      break;
    case 4:
      firstLine = "rain: ";
      firstLine += isRaining;
      firstLine += "          ";
      lcd.setCursor(0, 0);
      lcd.print(firstLine);
      secondLine = "gauge:";
      secondLine += rain_fall;
      secondLine += " mm         ";
      lcd.setCursor(0, 1);
      lcd.print(secondLine);
      break;
  }
}

void start_dust_sensor(){
  if (!dust_sensor.begin()) {
    Serial.println("Failed to initialize HM330X");
    while (1);
  }
  Serial.println("HM330X initialized");
}

void start_light_sensor(){
  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
}

void start_pressure_sensor(){
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring !"));
    while (1) delay(10);
  }
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

  /*
   *
   * Interruptions
   *
   */
void countAnemometer() {
   numRevsAnemometer++;
}
void countRainGauge() {
   numDropsRainGauge++;
}
void lcdChange(){
  valAAfficher = (valAAfficher + 1) % 5;
}
/*
 *
 * Traitements
 *
 */
float dustValue(){
  if (!dust_sensor.readSensor()) {
    Serial.println("Failed to read HM330X");
  } /*else {
    Serial.print("Sensor number"); Serial.println(dust_sensor.getSensorNumber());
    Serial.println("Concentration of PM2.5 particles based on atmospheric environment (ug/m^3) :");
    Serial.print("PM2.5 "); Serial.println(dust_sensor.atm.getPM2_5());
  }*/
  return dust_sensor.atm.getPM2_5();
}


float lightValue()
{
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  //Serial.print("Light is: "); Serial.print(tsl.calculateLux(full, ir));Serial.println("  lux. ");
  return tsl.calculateLux(full, ir);
}

float temp_Value()
{
  float t = dht.readTemperature();

  if (isnan(t)){
    return 0.0;
  }
  //Serial.print("Temperature: ");Serial.print(t);Serial.println("°C");
  return t;
}

float Humidity_Value()
{
  float h = dht.readHumidity();


  if (isnan(h)){

    return 0.0;
  }
  //Serial.print("Humidity: ");Serial.print(h);Serial.println("%");
  return h;
}

float pressure_Value()
{
  /*
  Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
  */
  return bmp.readPressure();
}

float alt_Value()
{
  /*
  Serial.print(F("Approx altitude = "));
    
    Serial.print(bmp.readAltitude(1013.25)); 
    Serial.println(" m");
    */
    return bmp.readAltitude(1020.25);
}


/*
 *
 * Kit station météo
 *
 */


int calcWindDir() {
   int val;
   byte x, reading;

   val = analogRead(PIN_VANE);
   val >>=2;                        // Shift to 255 range
   reading = val;
   global_reading = reading;
   // Look the reading up in directions table. Find the first value
   // that's >= to what we got.
   for (x=0; x<NUMDIRS; x++) {
      if (adc[x] >= reading)
         break;
   }
   /*Serial.print("  Dir: ");
   Serial.println(strVals[x]);*/
   return x;
}

float calcWindSpeed() {
   int x, iSpeed;
   long speed = 14920;
   speed *= numRevsAnemometer++;
   speed /= MSECS_CALC_WIND_SPEED;
   iSpeed = speed * 1.62;         // Convertir en km/h

  /* Serial.print("Wind speed: ");
   x = iSpeed / 10;
   Serial.print(x);
   Serial.print('.');
   x = iSpeed % 10;
   Serial.print(x);*/

   numRevsAnemometer = 0;        // Reset counter
   return iSpeed / 10.0;
}

float calcRainFall() {
   int x, iVol;
   long vol = 2794; // 0.2794 mm
   vol *= numDropsRainGauge;
   vol /= MSECS_CALC_RAIN_FALL;
   iVol = vol;         // Need this for formatting below
   /*
   Serial.print("Rain fall: ");
   x = iVol / 10000;
   Serial.print(x);
   Serial.print('.');
   x = iVol % 10000;
   Serial.print(x);
   Serial.println();*/
   
   numDropsRainGauge = 0;        // Reset counter
   return iVol/10000.0;
}
