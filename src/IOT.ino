#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <IoAbstraction.h>
#include <IoAbstractionWire.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

void dustPwmCallback();
void coPwmCallback();
void mqttServiceCallback();
void sensorCallback();
void blinkled();

/************************* Tasks *********************************/

Scheduler runner;

Task t1(10, TASK_FOREVER, &dustPwmCallback, &runner, true);
//Task t2(60000, TASK_FOREVER, &coPwmCallback, &runner, true);
//Task t3(90000, TASK_FOREVER, &coPwmCallback, &runner, true);
Task t4(1000, TASK_FOREVER, &sensorCallback, &runner, true);
Task t5(5000, TASK_FOREVER, &mqttServiceCallback, &runner, true);
Task t6(500, 4, &blinkled, &runner, true);

/************************* I2C Devices *********************************/

#define ADDR_PEX  0x20
#define ADDR_ADC  0x48
#define ADDR_BME  0x76


Adafruit_ADS1115 adc;
Adafruit_BME280 bme;
IoAbstractionRef ioExpander = ioFrom8574(ADDR_PEX);


/************************* Global Sensor Variables *********************************/

unsigned int data_chache[16][5];
float data_bme[3];
bool  control_reg[1] = {0}; //{bme enable bit, 
bool ready = 0;

/************************* WiFi Access Point *********************************/

#define WLAN_SSID "***REMOVED***"
#define WLAN_PASS "***REMOVED***"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "***REMOVED***"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
#define BOARDNAME "senbrd1"
Adafruit_MQTT_Publish MQ3 =   Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/MQ3");
Adafruit_MQTT_Publish MQ4 =   Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/MQ4");
Adafruit_MQTT_Publish MQ7 =   Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/MQ7");
Adafruit_MQTT_Publish MQ8 =   Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/MQ8");
Adafruit_MQTT_Publish MQ135 = Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/MQ135");
Adafruit_MQTT_Publish LIGHT = Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/LIGHT");
Adafruit_MQTT_Publish UV =    Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/UV");
Adafruit_MQTT_Publish DUST =  Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/DUST");
Adafruit_MQTT_Publish SOUND = Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/SOUND");
Adafruit_MQTT_Publish TEMP =  Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/TEMP");
Adafruit_MQTT_Publish HUM =   Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/HUM");
Adafruit_MQTT_Publish PRES =  Adafruit_MQTT_Publish(&mqtt, "/" BOARDNAME "/PRES");


void setup(void) 
{
  
  Serial.begin(115200);
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  
  Wire.begin();

  ioDevicePinMode(ioExpander, 0, OUTPUT);
  ioDevicePinMode(ioExpander, 1, OUTPUT);
  ioDevicePinMode(ioExpander, 2, OUTPUT);
  ioDevicePinMode(ioExpander, 3, OUTPUT);
  ioDevicePinMode(ioExpander, 4, OUTPUT);
  ioDevicePinMode(ioExpander, 5, INPUT);
  ioDevicePinMode(ioExpander, 6, OUTPUT);
  ioDevicePinMode(ioExpander, 7, OUTPUT);
  pinMode(D5, OUTPUT);

  ioDeviceDigitalWrite(ioExpander, 0, 0); //A
  ioDeviceDigitalWrite(ioExpander, 1, 0); //B
  ioDeviceDigitalWrite(ioExpander, 2, 0); //C
  ioDeviceDigitalWrite(ioExpander, 3, 0); //EN, ACTIVE LOW
  ioDeviceDigitalWrite(ioExpander, 4, 1); //EN, ACTIVE LOW
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
   adc.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // adc.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // adc.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // adc.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // adc.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // adc.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  pinMode(2, OUTPUT);
  digitalWrite(2, 1);

  Wire.beginTransmission(ADDR_PEX);
  if (Wire.endTransmission()!=0) {
        Serial.println("Could not find PCF8574 Port Expander!");
        while (1);
  }
  ioDeviceSync(ioExpander);
  Serial.println("Port Expander connected.");

  Wire.beginTransmission(ADDR_ADC);
  if (Wire.endTransmission()!=0) {
        Serial.println("Could not find ADS1115 ADC!");
        while (1);
  }
  adc.begin();
  Serial.println("ADC connected.");

  Wire.beginTransmission(ADDR_BME);
  if (Wire.endTransmission()!=0) {
      Serial.println("Could not find a valid BME280 sensor, ignoring!");
      control_reg[0] = 0;
  }
  else{
    bme.begin(ADDR_BME);
    control_reg[0] = 1;
    Serial.println("BME280 connected.");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X16,  // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X16,  // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5 );
  }
  
  
  t5.delay(15001);
  runner.startNow(); 
}


void loop(){

  MQTT_connect();
  ioDeviceSync(ioExpander);
  runner.execute();
  
}

void blinkled(){
  static bool state = 1;
  state = !state;
  digitalWrite(2, state);
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

void dustPwmCallback(){

  static int dim = -1;
  dim++;
  if(dim==5)
    dim = 0;

  ioDeviceDigitalWrite(ioExpander, 0, 1); //A
  ioDeviceDigitalWrite(ioExpander, 1, 1); //B
  ioDeviceDigitalWrite(ioExpander, 2, 1); //C
  ioDeviceDigitalWrite(ioExpander, 3, 0); //EN, ACTIVE LOW
  //ioDeviceDigitalWrite(ioExpander, 6, 1);
  digitalWrite(D5, 1);
  ioDeviceSync(ioExpander);
  
  delayMicroseconds(280);
  data_chache[7][dim] = adc.readADC_SingleEnded(0);
  delayMicroseconds(40);
  
  //ioDeviceDigitalWrite(ioExpander, 6, 0);
  digitalWrite(D5, 0);
  ioDeviceSync(ioExpander);
}


void sensorCallback(){

  #define LAST_SENSOR 14
  
  static int dim = -1;
  dim++;
  if(dim==5){
    ready = 1;
    dim = 0;
  }

  int i;
  for(i=0; i<=16;i++){

    ioDeviceDigitalWrite(ioExpander, 0, (i >> 0) & 1); //A
    ioDeviceDigitalWrite(ioExpander, 1, (i >> 1) & 1); //B
    ioDeviceDigitalWrite(ioExpander, 2, (i >> 2) & 1); //C
    ioDeviceDigitalWrite(ioExpander, 3, ((i >> 3) & 1)); //EN, ACTIVE LOW
    ioDeviceSync(ioExpander);

    if(i!=16){
      delayMicroseconds(500);
    }

    if(i==LAST_SENSOR)
      i = 16;
    if(i==7)
      continue; //skip
    else if(i<8)
      data_chache[i][dim] = adc.readADC_SingleEnded(0);
    else if(i <16)
      data_chache[i][dim] = adc.readADC_SingleEnded(2);
  }
  if(control_reg[0]){
    data_bme[0] = bme.readTemperature();
    data_bme[1] = bme.readPressure()/100;
    data_bme[2] = bme.readHumidity();  
  }

  
}

void mqttServiceCallback(){
  bool state = 0;
  float data_chache_sum[16];

  if(!ready) return;

  for(int i=0; i<16; i++){
    data_chache_sum[i]=0;
    for(int j=0; j<5; j++){
      if(data_chache[i][j] <= 0x7FFF)
        data_chache_sum[i] += data_chache[i][j] * 0.0001875;
      else
        data_chache_sum[i] += ((float)data_chache[i][j] - (float)0xFFFF) * 0.0001875;
    }
    data_chache_sum[i] = data_chache_sum[i]/5;
  }

  MQ3.publish(data_chache_sum[0], 5);
  MQ4.publish(data_chache_sum[1], 5);
  MQ7.publish(data_chache_sum[2], 5);
  MQ8.publish(data_chache_sum[3], 5);
  MQ135.publish(data_chache_sum[4], 5);
  LIGHT.publish(data_chache_sum[5], 5);
  UV.publish(data_chache_sum[6], 5);
  DUST.publish(data_chache_sum[7], 5);
  SOUND.publish(data_chache_sum[8], 5);
  if(control_reg[0]){
    TEMP.publish(data_bme[0]);
    PRES.publish(data_bme[1]);
    HUM.publish(data_bme[2]);
  }
    
   
}
