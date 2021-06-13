
#define TINY_GSM_MODEM_SIM800

#include <Arduino.h>
#include <AsyncTCP.h>
#include "painlessMesh.h"
#include "FS.h"
#include <SPIFFS.h>
#include <ModbusRTU.h>
#include <Arduino_JSON.h>
#include "PubSubClient.h"
#include <TinyGsmClient.h>
#include <SPI.h> 
//#include "esp_wifi.h"
//#include "esp_eth.h"
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <AsyncElegantOTA.h>


#define relayPin 5
#define MAX485_DE_RE 5
#define sendLed 26
#define connLed 26
#define BUTTON_PIN 0
#define ROLE "LMH_1"
#define VERSION "SIM v1.0.2"
//#define MESH_PORT 5555 // Mesh Port should be same for all nodes in Mesh Network

//volatile bool need_restart = false;
//volatile int interrupts;

//hw_timer_t * timer = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


#define SerialMon Serial
#ifndef __AVR_ATmega328P__
#define SerialAT Serial

// or Software Serial on Uno, Nano
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(2, 3); // RX, TX
#endif

#define TINY_GSM_DEBUG SerialMon
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
//#define GSM_PIN ""
const char apn[] = "www";
const char gprsUser[] = "";
const char gprsPass[] = "";


const char *topicLed = "test/rofl";
const char *topicInit = "test/lol";
const char *topicLedStatus = "test/lol";

#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(Serial);
#endif
TinyGsmClient espClient(modem);

const char *mqtt_server = "50.62.22.142";

//objects declaraation
ModbusRTU mb;
PubSubClient client(espClient);
Modbus::ResultCode err;
Scheduler userScheduler; // to control your personal task
//AsyncWebServer server(80);
//WiFiClient espClient;

//variables
uint8_t mfd_read_pos = 0, sdQueue;
uint16_t hregs2[96];
float mfdValues[25];
uint32_t mfdVals[25];
int device_count;
uint16_t mfd_dev_id[10];
String id;
unsigned long ts_epoch;
int rebootTime;
int pos;
uint32_t root;
long previousMillis = 0;
int mfd = 0;
int first_Reg, second_Reg;
int time_to_print;
bool MCP_Sent = false;
String msgMfd_payload[5];
uint32_t lastReconnectAttempt = 0;
//String payload;
//xSemaphoreHandle xMutex;
//int wdt = 0;


// User stub
void mfdConfig();
void updateTime();
void sendMFD();
void vApplicationIdleHook(void);
void writeTimeToCard();
String readMfd(uint16_t devId, uint16_t address, uint8_t iteration);
bool dataStream(uint16_t address, uint16_t deviceId);
boolean read_Mfd_Task();
void updateTime();
void sendPayload(String &payload);
void saveToCard(String &payload);
void multi_mfd_read();
void convertMfdFloats();
void sendLog();
//void IRAM_ATTR onTime();
//void checkAlive(void *bwehh);
//void meshUpdate(void *random);
//void schedulerUpdate(void *random);


void updateTime()
{
  //will update time && also watchdog
  //wdt++;
  ts_epoch++;
  rebootTime++;

}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String bweh;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    bweh += (char)payload[i];
  }
  Serial.println(bweh);
  ts_epoch = bweh.toInt();
  Serial.println(ts_epoch);
}
/*
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += "hhh";//" String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("test/lol", "hello world");
      // ... and resubscribe
      client.subscribe("test/rofl");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}*/

Task taskSendLog(TASK_SECOND * 8, TASK_FOREVER, &sendLog); // Set task second to send msg in a time interval
void sendLog()
{
  String logs;
  String topic = "test/lol";

  SPIFFS.begin();
  File file = SPIFFS.open("/offlinelog.txt", "r"); // FILE_READ is default so not realy needed but if you like to use this technique for e.g. write you need FILE_WRITE
  
  if(!file)
  {
    Serial.println("No File Found ");
    taskSendLog.disable();
    pos = 0;
    return;
  }
  String buffer;
  uint8_t i = 0;
  for (i = 0; i < 1; i++)
  {
    file.seek(pos);
    buffer = file.readStringUntil('\n');
    Serial.println(buffer); //Printing for debugging purpose
    logs = buffer;
    if (buffer != "")
    {
      client.publish(topic.c_str(), logs.c_str());
      Serial.println(logs);
      pos = file.position();
    }

    file.close();
    Serial.println(F("DONE Reading"));
  }

  if (buffer == "")
  {
    Serial.print("done dumping");
    SPIFFS.remove("/offlinelog.txt");
  }
  SPIFFS.end();
}

//Declarations for tasks scheduling
Task taskUpdateTime(TASK_SECOND * 1, TASK_FOREVER, &updateTime);        // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskReadMfd(TASK_MINUTE * 2, TASK_FOREVER, &read_Mfd_Task);        // Set task second to send msg in a time interval (Here interval is 4 second)
Task taskMultiMfdRead(TASK_SECOND * 10, TASK_FOREVER, &multi_mfd_read); // Set task second to send msg in a time interval (Here interval is 4 second)

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(mqtt_server);

  // Connect to MQTT Broker
  //boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
   boolean status = client.connect("GsmClientName", "username", "password");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  client.publish(topicInit, " MQTT Connected");
  client.subscribe(topicLed,ts_epoch);
  return client.connected();
}


void setup()
{
  SerialMon.begin(115200);
  delay(10);
  SerialMon.println("Wait...");
  // TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);

  delay(6000);
  SerialMon.println("Initializing modem...");
  //modem.restart();
  modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    SerialMon.println(" fail");
    delay(10);
    modem.restart();
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected())
  {
    SerialMon.println("Network connected");
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    SerialMon.println(" fail");
    delay(10);
    modem.restart();
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected())
  {
    SerialMon.println("GPRS connected");
  }
#endif
/*
// Configure Prescaler to 80, as our timer runs @ 80Mhz
	// Giving an output of 80,000,000 / 80 = 1,000,000 ticks / second
	timer = timerBegin(0, 80, true);                
	timerAttachInterrupt(timer, &onTime, true);    
	// Fire Interrupt every 1m ticks, so 1s
	timerAlarmWrite(timer, 20 * 1000000, true);			
	timerAlarmEnable(timer);*/

  //setup_MQTT;
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

 
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8E1);
  mb.begin(&Serial2);
  mb.master();

  SPIFFS.begin();

  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile)
  {
    Serial.println("Failed to open config file");
    //lcd.println("Failed to open config file");
  }
  size_t size = configFile.size();
  if (size > 1024)
  {
    Serial.println("Config file size is too large");
  }
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<1024> doc;
  auto error = deserializeJson(doc, buf.get());
  if (error)
  {
    Serial.println("Failed to parse config file");
  }
  const char *ID = doc["id"];

  const char *ROOT = doc["root"];
  const char *MFD = doc["mfd"];
  const char *DELAY = doc["delay"];

  Serial.print("Loaded id: ");

  id = ID;
  
  Serial.println(ID);
  // Serial.print("Loaded root id: ");
  root = 2137584946;
  Serial.println(ROOT);

   mfd = atoi(MFD);
  Serial.println(mfd);
  configFile.close();

  Serial.print("Loaded MCP pins: ");
  Serial.print(" loaded Send Delay: ");
  // maintain time in case of wdt reset
  File timeFile = SPIFFS.open("/time.txt", "r");
  if (timeFile)
  {
    String timeTemp = timeFile.readStringUntil('\n');
    ts_epoch = timeTemp.toInt();
    timeFile.close();
  }
  if (SPIFFS.exists("/pos.txt"))
  {
    File posFile = SPIFFS.open("/pos.txt", "r");
    String timeTemp = timeFile.readStringUntil('\n');
    pos = timeTemp.toInt();

    posFile.close();
  }
  Serial.println("position");
  Serial.print(pos);
  mfdConfig();
  //start the mesh
  //declarations for scheduler UwU
  userScheduler.addTask(taskMultiMfdRead);
  
  userScheduler.addTask(taskUpdateTime);
  taskUpdateTime.enable();
  userScheduler.addTask(taskReadMfd);
  taskReadMfd.enable();
  userScheduler.addTask(taskSendLog);
  taskSendLog.enable();

  SPIFFS.end();
  pinMode(connLed, OUTPUT);
  //xTaskCreatePinnedToCore(schedulerUpdate, "schedulerUpdate", 32000, NULL, 2, NULL, 1);
  //xTaskCreatePinnedToCore(checkAlive, "check to see if root is alive", 4096 * 2, NULL, 2, NULL, 1);
  
 /* server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");*/
 
}

/*
void schedulerUpdate(void *random)
{

  for (;;)
  {
    userScheduler.execute();

    if (!client.connected())
    {
      reconnect();
    }
    client.loop();

    vTaskDelay(10 / portTICK_RATE_MS);  delay(10);
  }
}
void checkAlive(void *bwehh)
{

  for (;;)
  {
    client.publish("test/lol", "ready");
    vTaskDelay(120 / portTICK_RATE_MS);
  }
}*/

void loop()
{
  // it will run the scheduler
  userScheduler.execute();

  if (!modem.isNetworkConnected())
  {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(30000L, true))
    {
      SerialMon.println(" fail");
      ESP.restart();
      return;
    }
    if (modem.isNetworkConnected())
    {
      SerialMon.println("Network re-connected");
    }

#if TINY_GSM_USE_GPRS
    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected())
    {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass))
      {
        SerialMon.println(" fail");
        ESP.restart();
        return;
      }
      if (modem.isGprsConnected())
      {
        SerialMon.println("GPRS reconnected");
      }
    }
#endif
  }

  if (!client.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
  }

if(rebootTime > 84600){
  writeTimeToCard();
  ESP.restart();
}
 //AsyncElegantOTA.loop();
 client.loop();
}

void writeTimeToCard()
{
  SPIFFS.begin();
  SPIFFS.remove("/time.txt");
  File dataFile = SPIFFS.open("/time.txt", "w");
  if (dataFile)
  {
    dataFile.println(ts_epoch);
    dataFile.close();
  }
  else
  {
    Serial.println("error opening time.txt");
  }
  SPIFFS.end();
}

boolean read_Mfd_Task()
{
  MCP_Sent = false;
  // userScheduler.disableAll();
  taskUpdateTime.enableIfNot();
  // vTaskSuspend(meshTaskHandle_t);
  time_to_print = ts_epoch;
  taskMultiMfdRead.enable();
  // xTaskCreate(multi_mfd_read,"readsMFD",20000,NULL,2,&mfdTaskHandle_t);
  return true;
}

bool resCallback(Modbus::ResultCode event, uint16_t, void *)
{
  err = event;
}

Modbus::ResultCode readSync(uint16_t Address, uint16_t start, uint16_t num, uint16_t *buf)
{
  // xSemaphoreTake(xMutex, portMAX_DELAY);
  if (mb.slave())
  {
    // xSemaphoreGive(xMutex);
    return Modbus::EX_GENERAL_FAILURE;
  }
  Serial.printf("SlaveID: %d Hreg %d\n", Address, start);
  mb.readHreg(Address, start, buf, num, resCallback);
  while (mb.slave())
  {
    mb.task();
  }
  Modbus::ResultCode res = err;
  // xSemaphoreGive(xMutex);
  return res;
}
bool dataStream(uint16_t address, uint16_t deviceId)
{

  if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
  {
    Serial.println("OK 2");
    convertMfdFloats();
    return true;
  }
  else
  {
    Serial.print("Error trying again ! ");

    // dataStream(address,deviceId);

    if (readSync(deviceId, address, 48, hregs2) == Modbus::EX_SUCCESS)
    {
      Serial.println("OK 2");
      convertMfdFloats();
      return true;
    }
    else
    {
      uint8_t j = 0;
      // String msgMfd;
      for (uint8_t i = 0; i <= 48; i++)
      {
        mfdValues[i] = 0;
      }

      return false;
    }
  }
}

void convertMfdFloats()
{

  uint8_t j = 0;
  // String msgMfd;
  for (uint8_t i = 0; i <= 48; i++)
  {

    uint16_t temp1[2] = {hregs2[i], hregs2[i + 1]};
    memcpy(&mfdValues[j], temp1, 32); i++;j++;
  }
}
String readMfd(uint16_t devId, uint16_t address, uint8_t iteration)
{
  String msgMfd;
  time_to_print = ts_epoch;

  if (dataStream(address, devId))
  {
    
    msgMfd += time_to_print;
    msgMfd.concat(",");
    msgMfd.concat(id);
    msgMfd.concat(",");
    msgMfd.concat(devId);
    msgMfd.concat(",");
    msgMfd.concat("13");
    msgMfd.concat(",");
    msgMfd.concat(iteration);
    msgMfd.concat(",");

    for (uint8_t i = 0; i <= 23; i++)
    {
      msgMfd.concat(mfdValues[i]);
      if (i <= 23)
      {
        msgMfd.concat(",");
      }
    }
    delay(500);
    String msgToSend = msgMfd;

    return msgToSend;
  }
  else{
    delay(500);
    // mfd_read_pos++;
    // taskMultiMfdRead.restart();
    msgMfd += time_to_print;
    msgMfd.concat(",");
    msgMfd.concat(id);
    msgMfd.concat(",");
    msgMfd.concat(devId);
    msgMfd.concat(",");
    msgMfd.concat("13");
    msgMfd.concat(",");
    msgMfd.concat(iteration);
    msgMfd.concat(",");

    //msgMfd.concat(",");

    for (uint8_t i = 0; i <= 23; i++)
    {
      msgMfd.concat(mfdValues[i]);
      if (i <= 23)
      {
        msgMfd.concat(",");
      }
    }
    return msgMfd;
  }
}
void multi_mfd_read()
{

  time_to_print++; //set the time for mfd data to be in sync
  Serial2.end();
  Serial2.begin(9600, SERIAL_8E1);

  msgMfd_payload[0] = readMfd(mfd_dev_id[mfd_read_pos], 100, 1);
  vTaskDelay(100 / portTICK_RATE_MS);
  msgMfd_payload[1] = readMfd(mfd_dev_id[mfd_read_pos], 148, 2);
  //msgMfd_payload[2] = readMfd(mfd_dev_id[mfd_read_pos], 148, 3);
  //msgMfd_payload[3] = readMfd(mfd_dev_id[mfd_read_pos],172,4);
  //msgMfd_payload[4] = readMfd(mfd_dev_id[mfd_read_pos],196,5);

  delay(10);
  sendMFD();
  Serial2.flush();
  mfd_read_pos++;
  if (mfd_read_pos >= device_count)
  {
    Serial2.end();
    mfd_read_pos = 0;
    taskMultiMfdRead.disable();
  }
}

void sendMFD()
{
  for (uint8_t i = 0; i < 5; i++)
  {
    sendPayload(msgMfd_payload[i]);
    vTaskDelay(300 / portTICK_RATE_MS);
    taskSendLog.enable();
  }
}

//writing data to card
void saveToCard(String &payload)
{
SPIFFS.begin();
if(!client.connected()){
  File dataFile = SPIFFS.open("/offlinelog.txt", "a");
  Serial.println("current data size : ");
  Serial.println(dataFile.size());
  dataFile.println(payload);
  dataFile.close();
  }
  SPIFFS.end();
}
//sending data to server
void sendPayload(String &payload)
{

  client.publish("test/lol", payload.c_str());

  Serial.println(payload);

}

void mfdConfig()
{

  SPIFFS.begin();

  File configFile = SPIFFS.open("/mfdConf.json", "r");
  if (!configFile)
  {
    Serial.println("Failed to open config file");
  }
  size_t size = configFile.size();
  if (size > 1024)
  {
    Serial.println("Config file size is too large");
  }
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<1024> doc;
  auto error = deserializeJson(doc, buf.get());
  if (error)
  {
    Serial.println("Failed to parse config file");
  }
  const char *DEV_COUNT = doc["device_count"];
  device_count = atoi(DEV_COUNT);
  Serial.println(device_count);

  const char *MFD_SERIAL_ID_1 = doc["mfd_dev_id_1"];
  mfd_dev_id[0] = atoi(MFD_SERIAL_ID_1);
  Serial.println(mfd_dev_id[0]);

  const char *MFD_SERIAL_ID_2 = doc["mfd_dev_id_2"];
  mfd_dev_id[1] = atoi(MFD_SERIAL_ID_2);
  Serial.println(mfd_dev_id[1]);

  const char *MFD_SERIAL_ID_3 = doc["mfd_dev_id_3"];
  mfd_dev_id[2] = atoi(MFD_SERIAL_ID_3);
  Serial.println(mfd_dev_id[2]);

  const char *MFD_SERIAL_ID_4 = doc["mfd_dev_id_4"];
  mfd_dev_id[3] = atoi(MFD_SERIAL_ID_4);
  Serial.println(mfd_dev_id[3]);

  const char *MFD_SERIAL_ID_5 = doc["mfd_dev_id_5"];
  mfd_dev_id[4] = atoi(MFD_SERIAL_ID_5);
  Serial.println(mfd_dev_id[4]);

  const char *MFD_SERIAL_ID_6 = doc["mfd_dev_id_6"];
  mfd_dev_id[5] = atoi(MFD_SERIAL_ID_6);
  Serial.println(mfd_dev_id[5]);

  const char *MFD_SERIAL_ID_7 = doc["mfd_dev_id_7"];
  mfd_dev_id[6] = atoi(MFD_SERIAL_ID_7);
  Serial.println(mfd_dev_id[6]);

  const char *MFD_SERIAL_ID_8 = doc["mfd_dev_id_8"];
  mfd_dev_id[7] = atoi(MFD_SERIAL_ID_8);
  Serial.println(mfd_dev_id[7]);
}

/*
void IRAM_ATTR onTime() {
  portENTER_CRITICAL_ISR(&timerMux);
  interrupts++;
	portEXIT_CRITICAL_ISR(&timerMux);

}*/