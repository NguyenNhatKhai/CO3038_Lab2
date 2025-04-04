#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Arduino_MQTT_Client.h>
#include <DHT20.h>
#include <HardwareSerial.h>
#include <ThingsBoard.h>
#include <WiFi.h>

constexpr char WIFI_SSID[] = "Harw";
constexpr char WIFI_PASSWORD[] = "baohan1107";
constexpr char TOKEN[] = "5Ap61lYda1GENmE54hla";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600UL;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
DHT20 dht20;

void InitWiFi() {
  Serial.print("Connecting to WiFi ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(); Serial.println("Connected to WiFi!");
}

void TaskWiFi(void *pvParameters) {
  while(1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting to reconnect ...");
      InitWiFi();
    }
    vTaskDelay(5000);
  }
}

void TaskDHT20(void *pvParameters) {
  while(1) {
    dht20.read();
    vTaskDelay(1000);
  }
}

void TaskTemperature(void *pvParameters) {
  while(1) {
    double temperature = dht20.getTemperature();
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" *C");
    tb.sendTelemetryData("temperature", temperature);
    vTaskDelay(3000);
  }
}

void TaskHumidity(void *pvParameters) {
  while(1) {
    double humidity = dht20.getHumidity();
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
    tb.sendTelemetryData("humidity", humidity);
    vTaskDelay(2000);
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println();
  InitWiFi();
  Wire.begin(GPIO_NUM_21, GPIO_NUM_22);
  dht20.begin();
  tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT);
  xTaskCreate(TaskWiFi, "WiFi", 2048, NULL, 2, NULL);
  xTaskCreate(TaskDHT20, "DHT20", 2048, NULL, 2, NULL);
  xTaskCreate(TaskTemperature, "Temperature", 2048, NULL, 2, NULL);
  xTaskCreate(TaskHumidity, "Humidity", 2048, NULL, 2, NULL);
}

void loop() {}