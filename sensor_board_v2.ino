#include <ETH.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <iostream>
#include <string>
#include <DHTesp.h>
#include <BH1750.h>
#include <Wire.h>
#include <SocketIOClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"

using namespace std;

#define WDT_TIMEOUT 15
#define ESP32
#define DHTpin 23

const char *ssid = "Lau 6";        // Lau 6
const char *password = "Sixfloor"; // Sixfloor
char host[] = "nam-cu.herokuapp.com";         // nam-cu.herokuapp.com
char namespace_32[] = "sensor_board_1";
int port = 80; // 80
extern String RID;
extern String Rname;
extern String Rcontent;
extern String Rfull;

SocketIOClient client;
DHTesp dht1;
BH1750 lightMeter;

// Timer variable
unsigned long pingTimeOut = 0;
unsigned long timeGetValue = 0;
unsigned long upTime = 0;
unsigned long timeCounter = 0;

// Variable
char esp32WorkingTime[100];
bool isPing = false;

// Array
String system_information[5] = {"ram", "ip", "rssi", "cpu clock", "working time"};
bool sensor_status[] = {true};

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

void sendData(String sensor_name, int sensor_index, String value_sensor_name, int EPPROM_index, int value)
{
  if (sensor_status[sensor_index] == true)
  {
    // Read value from EEPROM
    int EPPROM_value = EEPROM.read(EPPROM_index);

    // Is check value change?
    if (value != EPPROM_value)
    {
      String JSON;
      StaticJsonBuffer<200> jsonBuffer;
      JsonArray &array = jsonBuffer.createArray();
      array.add(sensor_name);
      array.add(value_sensor_name);
      array.add(value);
      array.printTo(JSON);
      client.sendJSON("sensor_data", JSON);
      EEPROM.write(EPPROM_index, value);
      Serial.println("temp value is change");
    }

    Serial.print(sensor_name);
    Serial.print(value);
    Serial.print("EPPROM_value: ");
    Serial.println(EPPROM_value);

    // EEPROM commit
    EEPROM.commit();
  }
}

template <typename val>
void send_system_information_value(String name, int index, val value)
{
  if (String(value) != system_information[index])
  {
    client.send("system_information", name, String(value));
    system_information[index] = value;
  }
}

void checkStatusOfSensor(String sensor_name, int index, int sensor_value, int dk)
{
  // check error
  if (isnan(sensor_value) || sensor_value >= dk)
  {
    if (sensor_status[index] != false)
    {
      client.send("sensor_status", sensor_name, "false");
      sensor_status[index] = false;
    }
  }
  else
  {
    if (sensor_status[index] != true)
    {
      client.send("sensor_status", sensor_name, "true");
      sensor_status[index] = true;
    }
  }

  Serial.print("sensor_name: ");
  Serial.println(sensor_status[index]);
}

void get_system_information()
{
  send_system_information_value("ram", 0, esp_get_free_heap_size());
  send_system_information_value("ip", 1, ip2Str(WiFi.localIP()));
  send_system_information_value("rssi", 2, WiFi.RSSI());
  send_system_information_value("cpu_clock", 3, getCpuFrequencyMhz());
  send_system_information_value("working_time", 4, esp32WorkingTime);
}

void get_data_sensor()
{
  sendData("DHT 11", 0, "temperature", 0, dht1.getTemperature());
  sendData("DHT 11", 0, "humidity", 1, dht1.getHumidity());
  sendData("BH1750", 1, "humidity", 2, lightMeter.readLightLevel());
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
  int minute = timeCounter - (hour * 60);
  sprintf(esp32WorkingTime, "%d hour %d minute", hour, minute);
}

void initModule()
{
  Serial.begin(115200);
  Wire.begin();
  lightMeter.begin();
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
  checkStatusOfSensor("DHT 11", 0, dht1.getTemperature(), 100);
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

    if (RID == "restart")
    {
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
    get_system_information();
    checkStatusOfSensor("DHT 11", 0, dht1.getTemperature(), 100);
    checkStatusOfSensor("BH1750", 1, lightMeter.readLightLevel(), 100000);
    get_data_sensor();
  }
  handleWorkingTime();
}
