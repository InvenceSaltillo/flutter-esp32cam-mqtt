#include "esp_camera.h"
#include <WiFi.h>
#include <MQTT.h>

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22


#define ESP32CAM_PUBLISH_TOPIC "esp32/cam_0"
const int bufferSize = 1024 * 23; // 23552 bytes

const char ssid[] = "RiojasdelaCruz";
const char pass[] = "VictoriaDLC";

WiFiClient net;
MQTTClient client = MQTTClient(bufferSize);

void connect() {

  WiFi.begin(ssid, pass);

  Serial.println("\n\n=====================");
  Serial.println("Connecting to Wi-Fi");
  Serial.println("=====================\n\n");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println("\n\n=====================");
  Serial.println("WIFI Connected!!");
  Serial.println("=====================\n\n");

  client.begin("192.168.0.12", net);

  Serial.println("\n\n=====================");
  Serial.println("Connecting to Raspberry PI");
  Serial.println("=====================\n\n");

  while (!client.connect("ESP32CAM")) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("Raspberry PI Timeout!");
    ESP.restart();
    return;
  }

  Serial.println("\n\n=====================");
  Serial.println("Raspberry PI Connected!");
  Serial.println("=====================\n\n");



}

void setup() {

  Serial.begin(115200);
  connect();

}

void loop() {
  client.publish(ESP32CAM_PUBLISH_TOPIC, "Hola mundo desde ESP32Cam");
  delay(5000);
}
