#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>

const char* ssid     = "ASUS_68";
const char* password = "kitchen_9544";

Adafruit_FXOS8700 magOnly = Adafruit_FXOS8700(0x1F);
WebServer server(80);

const uint16_t STREAM_HZ = 50;                
const uint32_t STREAM_DT_MS = 1000 / STREAM_HZ;

void handleMagOnce() {
  sensors_event_t a, m;
  magOnly.getEvent(&a, &m);
  // Formato JSON 
  String json = "{\"x\":" + String(m.magnetic.x, 2) +
                ",\"y\":" + String(m.magnetic.y, 2) +
                ",\"z\":" + String(m.magnetic.z, 2) + "}";
  server.send(200, "application/json", json);
}

void handleMagStream() {
  WiFiClient client = server.client();
  client.setNoDelay(true);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/x-ndjson");
  client.println("Cache-Control: no-cache");
  client.println("Connection: close");
  client.println();

  uint32_t last = millis();
  while (client.connected()) {
    uint32_t now = millis();
    if (now - last < STREAM_DT_MS) { delay(1); continue; }
    last = now;

    sensors_event_t a, m;
    magOnly.getEvent(&a, &m);

    char line[96];
    int n = snprintf(line, sizeof(line),
                     "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}\n",
                     m.magnetic.x, m.magnetic.y, m.magnetic.z);
    client.write((const uint8_t*)line, n);

    // keep WiFi/WD alive
    delay(0);
    if (!client.connected()) break;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.setSleep(false); 
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  Wire.begin(21, 22);
  if (!magOnly.begin(0x1F, &Wire)) {
    Serial.println("FXOS8700 not found"); while (1) delay(1000);
  }
  Serial.println("FXOS8700 ready");

  server.on("/magnetometer", handleMagOnce);
  server.on("/magnetometer/stream", handleMagStream);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
