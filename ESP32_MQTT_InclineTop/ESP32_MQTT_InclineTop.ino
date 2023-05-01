// This example uses an ESP32 Development Board
// to connect to shiftr.io.
//
// You can check on your device after a successful
// connection here: https://www.shiftr.io/try.
//
// by Joël Gähwiler
// https://github.com/256dpi/arduino-mqtt

#include <WiFi.h>
#include <MQTT.h>
#include "Sensor.h"

const char ssid[] = "oops";
const char pass[] = "oops";

#define NumberOfSensors 1
Sensor Sensors[NumberOfSensors];

WiFiClient net;
MQTTClient client;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("ESP32_Top_Incline")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  //client.subscribe("track/turnout/#");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);

    int tryDelay = 500;
  int numberOfTries = 20;

  // Wait for the WiFi event
  bool isConnected = false;
  while (!isConnected) {

    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:
        Serial.println("[WiFi] SSID not found");
        break;
      case WL_CONNECT_FAILED:
        Serial.print("[WiFi] Failed - WiFi not connected! Reason: ");
        return;
        break;
      case WL_CONNECTION_LOST:
        Serial.println("[WiFi] Connection was lost");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("[WiFi] Scan is completed");
        break;
      case WL_DISCONNECTED:
        Serial.println("[WiFi] WiFi is disconnected");
        break;
      case WL_CONNECTED:
        Serial.println("[WiFi] WiFi is connected!");
        Serial.print("[WiFi] IP address: ");
        Serial.println(WiFi.localIP());
        isConnected = true;
        break;
      default:
        Serial.print("[WiFi] WiFi Status: ");
        Serial.println(WiFi.status());
        break;
    }
    delay(tryDelay);

    if (numberOfTries <= 0) {
      Serial.print("[WiFi] Failed to connect to WiFi!");
      // Use disconnect function to force stop trying to connect
      WiFi.disconnect();
      return;
    } else {
      numberOfTries--;
    }
  }

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin("192.168.100.29", net);
  client.onMessage(messageReceived);

  connect();

    InitialiseConfig();
}

void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  for (int i = 0; i < NumberOfSensors; i++) {
    UpdateSensor(i);
  }


    //client.publish("/hello", "world");

}

void UpdateSensor(int i) {
  bool hasChanged = Sensors[i].UpdateSensor();
  if (hasChanged) {
    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();

    client.publish(topic, publishMessage,true,0);

  }
}

void InitialiseConfig() {
  //Sensors - Name, Pin, JMRIId, IsInverted = false, Pinmode = INPUT, Lastknownvalue = 0
  Sensors[0] = Sensor("MS Incline Top 3", 23, "3013", true, INPUT_PULLUP);
  //Sensors[1] = Sensor("MS Incline Top 4", 14, "3015", true, INPUT_PULLUP);



  for (int i = 0; i < NumberOfSensors; i++) {
    Sensors[i].SetPinMode();
    UpdateSensor(i);

    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();
  }
  Serial.println("Setup complete");
}
