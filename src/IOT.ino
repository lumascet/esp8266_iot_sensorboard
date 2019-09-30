#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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

typedef enum sensors_e{
  MQ3,
  MQ4,
  MQ7,
  MQ8,
  MQ135,
  LIGHT,
  UV,
  DUST,
  SOUND,
  TEMP,
  HUM,
  PRES
}sensors_t;


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

const char* ssid = "***REMOVED***";
const char* password = "***REMOVED***";
const char* mqtt_server = "***REMOVED***";
const char* clientUsr = "***REMOVED***";
const char* clientPw = "***REMOVED***";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

String BOARDNAME = "senbrd1";
String MTYPE = "sensor";
String HOMEPREFIX = "homeassistant";
String UUID = (String)ESP.getChipId();
String VERSION = "V2.0";

String MQ3_topic = "MQ3";
String MQ4_topic = "MQ4";
String MQ7_topic = "MQ7";
String MQ8_topic = "MQ8";
String MQ135_topic = "MQ135";
String LIGHT_topic = "LIGHT";
String UV_topic = "UV";
String DUST_topic = "DUST";
String SOUND_topic = "SOUND";
String TEMP_topic = "TEMP";
String HUM_topic = "HUM";
String PRES_topic = "PRES";
String sensor_suffix[] ={
  MQ3_topic,
  MQ4_topic,
  MQ7_topic,
  MQ8_topic,
  MQ135_topic,
  LIGHT_topic,
  UV_topic,
  DUST_topic,
  SOUND_topic,
  TEMP_topic,
  HUM_topic,
  PRES_topic
};
String sensor_unit[] ={
  "V",
  "V",
  "V",
  "V",
  "V",
  "V",
  "V",
  "V",
  "V",
  "°C",
  "%",
  "hPa"
};
String sensor_icon[]{
  "mdi:gas-cylinder",
  "mdi:gas-cylinder",
  "mdi:gas-cylinder",
  "mdi:gas-cylinder",
  "mdi:gas-cylinder",
  "mdi:brightness-6",
  "mdi:weather-sunny",
  "mdi:chart-bubble",
  "mdi:volume-high",
  "mdi:thermometer",
  "mdi:water-percent",
  "mdi:gauge"
};


//https://arduinojson.org/v6/assistant/

void switchDiscovery(){
  char topic_buffer[50], payload_buffer[300];
  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(15);
  DynamicJsonDocument doc(capacity);

  doc["~"] = BOARDNAME + "/";
  doc["name"] = BOARDNAME;
  doc["cmd_t"] = "~cmnd/POWER";
  doc["stat_t"] = "~tele/STATE";
  doc["val_tpl"] = "{{value}}";
  doc["pl_off"] = "OFF";
  doc["pl_on"] = "ON";
  doc["avty_t"] = "~tele/LWT";
  doc["pl_avail"] = "Online";
  doc["pl_not_avail"] = "Offline";
  doc["uniq_id"] = UUID + "_switch";

  JsonObject device = doc.createNestedObject("device");
  JsonArray device_identifiers = device.createNestedArray("identifiers");
  device_identifiers.add(UUID);

  serializeJson(doc, payload_buffer);
  sprintf(topic_buffer, "%s/%s/%s/%s", HOMEPREFIX.c_str(), "switch",(UUID + "_switch").c_str(), "config");
  client.publish(topic_buffer, payload_buffer, strlen(payload_buffer));
}

void statusDiscovery(){
  char topic_buffer[50], payload_buffer[1000];
  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(5) + JSON_OBJECT_SIZE(20);
  DynamicJsonDocument doc(capacity);

  doc["name"] = BOARDNAME + " status";
  doc["stat_t"] = "~HASS_STATE";
  doc["avty_t"] = "~LWT";
  doc["pl_avail"] = "Online";
  doc["pl_not_avail"] = "Offline";
  doc["json_attributes_topic"] = "~HASS_STATE";
  doc["unit_of_meas"] = " ";
  doc["val_tpl"] = "{{value_json['RSSI']}}";
  doc["uniq_id"] = UUID + "_status";

  JsonObject device = doc.createNestedObject("device");
  JsonArray device_identifiers = device.createNestedArray("identifiers");
  device_identifiers.add(UUID);
  device["name"] = BOARDNAME;
  device["model"] = "DIY Sensorboard";
  device["sw_version"] = VERSION;
  device["manufacturer"] = "Lukas Schröer";
  doc["~"] = BOARDNAME + "/tele/";

  serializeJson(doc, payload_buffer);
  //strcpy(payload_buffer,"{\"name\":\"senbrd1 status\",\"stat_t\":\"~HASS_STATE\",\"avty_t\":\"~LWT\",\"pl_avail\":\"Online\",\"pl_not_avail\":\"Offline\",\"json_attributes_topic\":\"~HASS_STATE\",\"unit_of_meas\":\" \",\"val_tpl\":\"{{value_json['RSSI']}}\",\"uniq_id\":\"3998661_status\",\"device\":{\"identifiers\":[\"3998661\"],\"name\":\"senbrd1\",\"model\":\"DIY Sensorboard\"}}");
  //strcpy(payload_buffer,"{\"name\":\"senbrd1 status\",\"pl_avail\":\"Online\",\"pl_not_avail\":\"Offline\",\"json_attributes_topic\":\"~HASS_STATE\",\"unit_of_meas\":\" \",\"val_tpl\":\"{{value_json['RSSI']}}\",\"uniq_id\":\"3998661_status\",\"device\":{\"identifiers\":[\"3998661\"],\"name\":\"senbrd1\",\"model\":\"DIY Sensorboard\"}}");
  sprintf(topic_buffer, "%s/%s/%s/%s", HOMEPREFIX.c_str(), "sensor", (UUID + "_status").c_str(), "config");
  Serial.println(topic_buffer);
  Serial.println(payload_buffer);
  client.publish(topic_buffer, payload_buffer, strlen(payload_buffer));
}

void hassStatePublish(){
  char topic_buffer[50], payload_buffer[1000];
  const size_t capacity = JSON_OBJECT_SIZE(15);
  DynamicJsonDocument doc(capacity);

  doc["Version"] = VERSION;
  doc["BuildDateTime"] = __TIME__;
  doc["Module"] = BOARDNAME;
  doc["Uptime"] = millis();
  doc["IPAddress"] = WiFi.localIP().toString();
  doc["RSSI"] = WiFi.RSSI();

  serializeJson(doc, payload_buffer);
  sprintf(topic_buffer, "%s/%s/%s", BOARDNAME.c_str(), "tele", "HASS_STATE");
  client.publish(topic_buffer, payload_buffer, strlen(payload_buffer));
}

void sensorConfigDiscovery(String name, String unit, String icon){
  char topic_buffer[50], payload_buffer[1000];
  String uniqid = UUID + "_" + name;
  uniqid.replace(' ', '_');
  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(16);
  DynamicJsonDocument doc(capacity);

  doc["name"] = BOARDNAME + " " + name;
  doc["stat_t"] = "~" + name;
  doc["avty_t"] = "~LWT";
  doc["val_tpl"] = "{{value}}";
  doc["pl_avail"] = "Online";
  doc["pl_not_avail"] = "Offline";
  doc["uniq_id"] = uniqid;

  JsonObject device = doc.createNestedObject("device");
  JsonArray device_identifiers = device.createNestedArray("identifiers");
  device_identifiers.add(UUID);
  doc["~"] = BOARDNAME + "/tele" + "/";
  doc["unit_of_meas"] = unit;
  doc["icon"] = icon;

  serializeJson(doc, payload_buffer);
  sprintf(topic_buffer, "%s/%s/%s/%s", HOMEPREFIX.c_str(), "sensor", uniqid.c_str(), "config");
  client.publish(topic_buffer, payload_buffer, strlen(payload_buffer));
}

void setupDiscovery(){
  switchDiscovery();
  statusDiscovery();
  for(int i=0; i<sizeof(sensor_suffix)/sizeof(sensor_suffix[0]); i++){
    sensorConfigDiscovery(sensor_suffix[i], sensor_unit[i], sensor_icon[i]);
  }
}


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), clientUsr, clientPw, (BOARDNAME + "/tele/LWT").c_str(), 0, 0, "Offline")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish((BOARDNAME + "/tele/LWT").c_str(), "Online");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup(void)
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  reconnect();
  setupDiscovery();

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

  digitalWrite(LED_BUILTIN, 1);

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
  if (!client.connected()) {
     reconnect();
   }
  client.loop();
  ioDeviceSync(ioExpander);
  runner.execute();

}

void blinkled(){
  static bool state = 1;
  state = !state;
  digitalWrite(LED_BUILTIN, state);
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
  float data_cache_sum[16];

  if(!ready) return;

  for(int i=0; i<16; i++){
    data_cache_sum[i]=0;
    for(int j=0; j<5; j++){
      if(data_chache[i][j] <= 0x7FFF)
        data_cache_sum[i] += data_chache[i][j] * 0.0001875;
      else
        data_cache_sum[i] += ((float)data_chache[i][j] - (float)0xFFFF) * 0.0001875;
    }
    data_cache_sum[i] = data_cache_sum[i]/5;
  }

  char topic[50], payload[10];
  for(int i=0; i<(sizeof(sensor_suffix)/sizeof(sensor_suffix[0])-3); i++){
    snprintf(payload, sizeof(payload), "%.5f", data_cache_sum[i]);
    strcpy(topic,(BOARDNAME + "/tele/" + sensor_suffix[i]).c_str());
    //snprintf(topic, sizeof(topic), "%s", tmp);
    client.publish(topic, payload);
  }

  if(control_reg[0]){
    snprintf(payload, sizeof(payload), "%.2f", data_bme[0]);
    strcpy(topic,(BOARDNAME + "/tele/" + sensor_suffix[TEMP]).c_str());
    client.publish(topic, payload);
    snprintf(payload, sizeof(payload), "%.2f", data_bme[1]);
    strcpy(topic,(BOARDNAME + "/tele/" + sensor_suffix[PRES]).c_str());
    client.publish(topic, payload);
    snprintf(payload, sizeof(payload), "%.2f", data_bme[2]);
    strcpy(topic,(BOARDNAME + "/tele/" + sensor_suffix[HUM]).c_str());
    client.publish(topic, payload);
  }
  hassStatePublish();
  /*
  MQ3.publish(data_cache_sum[0], 5);
  MQ4.publish(data_cache_sum[1], 5);
  MQ7.publish(data_cache_sum[2], 5);
  MQ8.publish(data_cache_sum[3], 5);
  MQ135.publish(data_cache_sum[4], 5);
  LIGHT.publish(data_cache_sum[5], 5);
  UV.publish(data_cache_sum[6], 5);
  DUST.publish(data_cache_sum[7], 5);
  SOUND.publish(data_cache_sum[8], 5);
  if(control_reg[0]){
    TEMP.publish(data_bme[0]);
    PRES.publish(data_bme[1]);
    HUM.publish(data_bme[2]);
  }
  */

}
