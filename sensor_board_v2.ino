#include <ETH.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <DHTesp.h>
#include <Wire.h>
#include <SocketIOClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"

#define WDT_TIMEOUT 3
#define ESP32
#define DHTpin 23

const char *ssid = "Lau 6";        // Lau 6
const char *password = "Sixfloor"; // Sixfloor
char host[] = "192.168.0.108"; //nam-cu.herokuapp.com
char namespace_32[] = "sensor_board";
int port = 3000; //80
extern String RID;
extern String Rname;
extern String Rcontent;
extern String Rfull;

SocketIOClient client;
DHTesp dht1;

// Timer variable
unsigned long pingTimeOut = 0;
unsigned long timeGetValue = 0;
unsigned long upTime = 0;
unsigned long timeCounter = 0;

// Variable
char esp32WorkingTime[100];
bool isPing = false;

String ip2Str(IPAddress ip)
{
  String s = "";
  for (int i = 0; i < 4; i++)
  {
    s += i ? "." + String(ip[i]) : String(ip[i]);
  }
  return s;
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, password);
}

void initWiFi()
{
  WiFi.disconnect(true);

  delay(1000);

  WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);

  /* Remove WiFi event
  Serial.print("WiFi Event ID: ");
  Serial.println(eventID);
  WiFi.removeEvent(eventID);*/

  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println();
  Serial.println("Wait for WiFi... ");
}

void connectServer()
{
  if (!client.connect(host, port, namespace_32))
  {
    Serial.println("connection failed");
    return;
  }
  if (client.connected())
  {
    client.send("connection", "message", "Connected !!!!");
  }
}

void moduleConfig()
{
  dht1.setup(DHTpin, DHTesp::DHT11);
}

void handleEventDisconnectServer()
{
  if (isPing == true)
  {
    pingTimeOut = millis();
  }
  else
  {
    if (millis() - pingTimeOut > 10000)
    {
      pingTimeOut = millis();
      client.connect(host, port, namespace_32);
    }
  }
  isPing = false;
}

void sendData(int temp_value, int humi_value)
{
  // Read value from EEPROM
  int temp_value_EEPROM = EEPROM.read(0);
  int humi_value_EEPROM = EEPROM.read(1);

  // Is check value change?
  if (temp_value != temp_value_EEPROM)
  {
    client.sendNumber("temp_value", temp_value);
    EEPROM.write(0, temp_value);
    Serial.println("temp value is change");
  }

  if (humi_value != humi_value_EEPROM)
  {
    client.sendNumber("humi_value", humi_value);
    EEPROM.write(1, humi_value);
    Serial.println("humi value is change");
  }

  Serial.print("Temp: ");
  Serial.print(temp_value);
  Serial.print("Temp EEPROM: ");
  Serial.print(temp_value_EEPROM);
  Serial.print("Humi: ");
  Serial.print(humi_value);
  Serial.print("Humi EEPROM: ");
  Serial.println(humi_value_EEPROM);

  // EEPROM commit
  EEPROM.commit();
}

void checkStatusOfSensor(int temp_value, int humi_value)
{
  // check error
  if (isnan(temp_value) || temp_value >= 200 || humi_value >= 200 || isnan(humi_value))
  {
    client.sendNumber("StatusOfDHT", 0);
  }
  else
  {
    client.sendNumber("StatusOfDHT", 1);
    sendData(temp_value, humi_value);
  }
}

void readDataSensor()
{
  // Read Value and status DH11
  int temp_value = dht1.getTemperature();
  int humi_value = dht1.getHumidity();

  checkStatusOfSensor(temp_value, humi_value);
}

void sendSensorBoardInformation()
{
  String JSON;
  long rssi = WiFi.RSSI();
  long ramLeft = esp_get_free_heap_size();
  long clockCPU = getCpuFrequencyMhz();

  Serial.print("rssi: ");
  Serial.print(rssi);
  Serial.print("ramLeft: ");
  Serial.print(ramLeft);
  Serial.print("esp32WorkingTime: ");
  Serial.print(esp32WorkingTime);
  Serial.print("clockCPU: ");
  Serial.println(clockCPU);

  StaticJsonBuffer<400> jsonBuffer;
  JsonObject &root1 = jsonBuffer.createObject();
  root1["signal"] = rssi;
  root1["ip"] = ip2Str(WiFi.localIP());
  root1["ramLeft"] = ramLeft;
  root1["clockCPU"] = clockCPU;
  root1["uptime"] = esp32WorkingTime;
  root1.printTo(JSON);
  client.sendJSON("sensor_board_information", JSON);
  JSON = "";
  jsonBuffer.clear();
}

void initWDT()
{
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
}

void handleWorkingTime()
{
  if (millis() - upTime > 60000)
  {
    timeCounter++;
    upTime = millis();
  }

  int hour = timeCounter / 60;
  int minute = timeCounter - ( hour * 60 );
  sprintf(esp32WorkingTime, "%d hour %d minute", hour, minute);
}

void initModule()
{
  Serial.begin(115200);
  Wire.begin();
  EEPROM.begin(12);
}

void setup()
{
  initModule();
  delay(100);
  initWiFi();
  delay(100);
  connectServer();
  delay(100);
  moduleConfig();
  delay(100);
  initWDT();
}

void loop()
{
  esp_task_wdt_reset();
  if (client.monitor())
  {
    Serial.println(Rfull);
    Serial.println(RID);

    if (RID == "ping")
    {
      client.sendNumber("pong", 0);
      isPing = true;
    }

    if (RID == "restart") {
      Serial.println("Restart");
      ESP.restart();
    }
    esp_task_wdt_reset();
    RID = "";
  }
  handleEventDisconnectServer();
  if (millis() - timeGetValue > 5000)
  {
    timeGetValue = millis();
    readDataSensor();
    sendSensorBoardInformation();
  }
  handleWorkingTime();
}
