#include <WiFi.h>

const char* ssid = "Remi";
const char* password = "qwerty123";

WiFiServer server(80);

// Motor driver pins
#define IN1 25
#define IN2 26
#define ENA 27
#define IN3 32
#define IN4 33
#define ENB 14

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ ESP32 Connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    if (request.indexOf("/MOTOR=ON") != -1) {
      Serial.println("➡️ MOTOR ON command received");
      moveForwardFor5Seconds();
    } else if (request.indexOf("/MOTOR=OFF") != -1) {
      Serial.println("⛔ MOTOR OFF command received");
      stopMotors();
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>ESP32 Motor Control</h2></body></html>");
    client.stop();
  }
}

void moveForwardFor5Seconds() {
  moveForward();
  delay(5000);
  stopMotors();
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
