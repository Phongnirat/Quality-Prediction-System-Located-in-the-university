#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

// ---------- WiFi & MQTT ----------
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_TOPIC "sensor/aqi"

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- Sensor Pins ----------
#define MQ7_PIN 34
#define MQ131_PIN 32
#define MQ136_PIN 35
#define RXD2 16 // PMS5003 TX
#define TXD2 17 // PMS5003 RX

// ---- MQ Constants (ค่าคงที่) -------
#define RL 10.0
#define VCC 5.0
float R0_MQ7 = 10.0;
float R0_MQ131 = 15.0;
float R0_MQ136 = 12.0;

#define A_MQ7 99.042 
#define B_MQ7 -1.518

#define A_MQ131_O3 9.4783
#define B_MQ131_O3 -1.179
// NO2 Constants (MQ131)
#define A_NO2 3.027
#define B_NO2 1.69

#define A_MQ136 5.944
#define B_MQ136 -2.222

// ---------- PMS5003 Data Structure ---------
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um;
  uint16_t particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

// ---------- WiFi ----------
void connectWiFi() {
  Serial.print("📶 Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n❌ WiFi Connection Failed!");
  }
}

// ---------- MQTT ----------
void connectMQTT() {
  Serial.print("🔌 Connecting to MQTT Broker...");
  int retry = 0;
  while (!client.connected() && retry < 5) {
    if (client.connect("ESP32_Client")) {
      Serial.println("✅ MQTT Connected");
      return;
    } else {
      Serial.print("❌ MQTT Failed (Code ");
      Serial.print(client.state());
      Serial.println("), retrying...");
      retry++;
      delay(2000);
    }
  }
  if (!client.connected()) {
    Serial.println("🚨 MQTT Connection Failed after retries!");
  }
}

// ---------- Calculate MQ ppm ----------
float readMQppm(int analogPin, float R0, float A, float B) {
  int adcValue = analogRead(analogPin);
  float voltage = adcValue * (VCC / 4095.0);
  float Rs = (VCC * RL / voltage) - RL;
  float ratio = Rs / R0;
  float ppm = A * pow(ratio, B);
  return ppm;
}

// ---------- PMS5003 Read ----------
boolean readPMSdata(Stream *s) {
  if (!s->available()) return false;
  if (s->peek() != 0x42) {
    s->read(); return false;
  }
  if (s->available() < 32) return false;

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  for (uint8_t i = 0; i < 30; i++) sum += buffer[i];

  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("❌ Checksum failure");
    return false;
  }
  return true;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); // PMS5003
  connectWiFi();
  client.setServer(MQTT_BROKER, MQTT_PORT);
  connectMQTT();
}

// ---------- LOOP ----------
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Lost WiFi. Reconnecting...");
    connectWiFi();
  }
  if (!client.connected()) {
    Serial.println("⚠️ Lost MQTT. Reconnecting...");
    connectMQTT();
  }

  client.loop();

  if (readPMSdata(&Serial1)) {
    float mq7_ppm = readMQppm(MQ7_PIN, R0_MQ7, A_MQ7, B_MQ7);
    
    // คำนวณค่า O3 และ NO2 จาก MQ131
    float mq131_o3_ppm = readMQppm(MQ131_PIN, R0_MQ131, A_MQ131_O3, B_MQ131_O3);
    int adc_MQ131 = analogRead(MQ131_PIN);
    float voltage_MQ131 = adc_MQ131 * (VCC / 4095.0);
    float Rs_MQ131 = (VCC * RL / voltage_MQ131) - RL;
    float ratio_MQ131 = Rs_MQ131 / R0_MQ131;
    float mq131_no2_ppm = A_NO2 * pow(ratio_MQ131, B_NO2);

    float mq136_ppm = readMQppm(MQ136_PIN, R0_MQ136, A_MQ136, B_MQ136);

    // แสดงค่าบน Serial
    Serial.println("📊 Sensor Readings:");
    Serial.printf("PM1.0: %d\tPM2.5: %d\tPM10: %d\n", data.pm10_standard, data.pm25_standard, data.pm100_standard);
    Serial.printf("MQ7(CO): %.2f ppm\tMQ131(O3): %.2f ppm\tMQ131(NO2): %.2f ppm\tMQ136(SO2): %.2f ppm\n",
                  mq7_ppm, mq131_o3_ppm, mq131_no2_ppm, mq136_ppm);
    Serial.println("------------------------------------------------");

    // ส่งข้อมูล MQTT
    String payload = "{";
    payload += "\"pm1_0\":" + String(data.pm10_standard) + ",";
    payload += "\"pm2_5\":" + String(data.pm25_standard) + ",";
    payload += "\"pm10\":" + String(data.pm100_standard) + ",";
    payload += "\"CO\":" + String(mq7_ppm, 2) + ",";
    payload += "\"O3\":" + String(mq131_o3_ppm, 2) + ",";
    payload += "\"NO2\":" + String(mq131_no2_ppm, 2) + ",";
    payload += "\"SO2\":" + String(mq136_ppm, 2);
    payload += "}";

    client.publish(MQTT_TOPIC, payload.c_str());
  }

  delay(2000);
}