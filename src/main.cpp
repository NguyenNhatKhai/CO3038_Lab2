////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Arduino_MQTT_Client.h>
#include <DHT20.h>
#include <HardwareSerial.h>
#include <ThingsBoard.h>
#include <WiFi.h>

////////////////////////////////////////////////////////////////////////////////////////////////////

constexpr char WIFI_SSID[] = "Harw";
constexpr char WIFI_PASSWORD[] = "baohan1107";
constexpr char TOKEN[] = "ynf3uNJE1tlUgkiz9tFQ";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600UL;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard thingsboard(mqttClient, MAX_MESSAGE_SIZE);
DHT20 dht20;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

void InitWiFi() {
  Serial.print("Connecting to WiFi ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(); Serial.println("Connected to WiFi!");
}

void InitThingsBoard() {
  Serial.print("Connecting to ThingsBoard ...");
  thingsboard.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT);
  while (!thingsboard.connected()) {
    delay(500);
    Serial.print(".");
    thingsboard.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT);
  }
  Serial.println(); Serial.println("Connected to ThingsBoard!");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

void TaskWiFi(void *pvParameters) {
  while(1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting to reconnect ...");
      InitWiFi();
    }
    vTaskDelay(5000);
  }
}

void TaskThingsBoard(void *pvParameters) {
  while(1) {
    if (!thingsboard.connected()) {
      Serial.println("ThingsBoard disconnected, attempting to reconnect ...");
      InitThingsBoard();
    }
    vTaskDelay(5000);
  }
}

void TaskDHT20(void *pvParameters) {
  while(1) {
    dht20.read();
    double temperature = dht20.getTemperature();
    double humidity = dht20.getHumidity();
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" *C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
    thingsboard.sendTelemetryData("temperature", temperature);
    thingsboard.sendTelemetryData("humidity", humidity);
    vTaskDelay(5000);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println();
  dht20.begin();
  Wire.begin(GPIO_NUM_21, GPIO_NUM_22);
  pinMode(GPIO_NUM_2, OUTPUT);
  InitWiFi();
  InitThingsBoard();
  xTaskCreate(TaskWiFi, "WiFi", 2048, NULL, 2, NULL);
  xTaskCreate(TaskThingsBoard, "ThingsBoard", 2048, NULL, 2, NULL);
  xTaskCreate(TaskDHT20, "DHT20", 2048, NULL, 2, NULL);
}

void loop() {}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////