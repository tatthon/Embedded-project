#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>

#define BAUDRATE 115200

#define DATABASE_URL "https://embbedproject-bdd4f-default-rtdb.asia-southeast1.firebasedatabase.app"
#define API_KEY "AIzaSyCftk72k3Jfn9PO3wSVrPpfhzRwuWvq6Zo"

#define SSID "Del"
#define PASSWORD "12345678"

#define UART_TX 17
#define UART_RX 18

HardwareSerial STM32_UART(1);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool signupOK = false;

// store parsed data
float temp = 0;
float humid = 0;
int soil = 0;
int pump = 0;

void setup() {
  Serial.begin(115200);

  STM32_UART.begin(BAUDRATE, SERIAL_8N1, UART_RX, UART_TX);

  // --- Connect WiFi ---
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("WiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
  }
  Serial.println("sign up successful");

  Firebase.begin(&config, &auth);
  Serial.println("fire base begin");
}

void loop() {
  if (STM32_UART.available()) {
    String jsonString = STM32_UART.readStringUntil('\n');
    jsonString.trim();

    Serial.println("Received → " + jsonString);

    StaticJsonDocument<256> doc;

    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.println("JSON Error!");
      return;
    }

    temp = doc["data"]["temp"];
    humid = doc["data"]["humid"];
    soil = doc["data"]["soil"];
    pump = doc["data"]["pump"];

    Serial.printf("Parsed → Temp: %.2f Humid: %.2f Soil: %d Pump: %d\n",
                  temp, humid, soil, pump);
  }

  // ส่ง Firebase
  if (signupOK && Firebase.ready()) {
      Firebase.RTDB.setFloat(&fbdo, "/smart-farm/temperature", temp);
      Firebase.RTDB.setFloat(&fbdo, "/smart-farm/humidity", humid);
      Firebase.RTDB.setInt(&fbdo,   "/smart-farm/soil-moisture", soil);
      Firebase.RTDB.setInt(&fbdo,   "/smart-farm/pump-status", pump);
  }
}
