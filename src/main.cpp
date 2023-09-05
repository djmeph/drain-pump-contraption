#include <Arduino.h>
#include <Bounce2.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define FLOAT_SWITCH_PIN 25
#define LED_PIN 23
#define RELAY_1_PIN 12
// #define RELAY_2_PIN 27 
# define RELAY_2_PIN 13

static int keepAlive;
static long timer;
// static int keepAliveTimeout = 60 * 1000; // 1 Minute
static int keepAliveTimeout = 5000;
static int pumpOverride = 0;

// WiFi
const char *ssid = "uss-enterprise-iot"; // Enter your WiFi name
const char *password = "jLI%4z6iy88qVxmjvAeA";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "192.168.0.194";
const char *publishTopic = "drain_pump_tx";
const char *subscribeTopic = "drain_pump_rx";
const char *mqtt_username = "djmeph";
const char *mqtt_password = "Dt06241701";
const int mqtt_port = 1883;
unsigned long lastMsg = 0;
const int READ_CYCLE_TIME = 3000;

enum STATES {
  RELAY_OFF,
  RELAY_ON,
  SWITCH_OFF,
  SWITCH_ON
};
void sendMsg(STATES state);

Bounce floatSwitch = Bounce();
WiFiClient espClient;
Adafruit_MQTT_Client mqtt(&espClient, mqtt_broker, mqtt_port, mqtt_username, mqtt_password);

// Subscriber
Adafruit_MQTT_Subscribe subscriber = Adafruit_MQTT_Subscribe(&mqtt, subscribeTopic, MQTT_QOS_1);

// Publisher
Adafruit_MQTT_Publish publisher = Adafruit_MQTT_Publish(&mqtt, publishTopic, MQTT_QOS_1);

void callback(char *message, uint16_t len) {
  char messageBuffer[40];
  snprintf(messageBuffer, sizeof(messageBuffer), "Message :: %s, len :: %u", message, len);
  Serial.println(messageBuffer);
  Serial.print("OVERRIDE_ON ");
  Serial.println(strcmp(message, "OVERRIDE_ON"));
  Serial.print("OVERRIDE_OFF ");
  Serial.println(strcmp(message, "OVERRIDE_OFF"));

  if (strcmp(message, "OVERRIDE_ON") == 0) {
    pumpOverride = 1;
    digitalWrite(RELAY_1_PIN, HIGH);
    digitalWrite(RELAY_2_PIN, HIGH);
    Serial.println("Relay On");
    sendMsg(RELAY_ON);
  }

  if (strcmp(message, "OVERRIDE_OFF") == 0) {
    pumpOverride = 0;
  }
}

void setupWifi() {
  delay(10);

  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int wifiTimeout = millis() + (30 * 1000); // 30 Second Timeout
  while (WiFi.status() != WL_CONNECTED && millis() <= wifiTimeout) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void connectToMQTTServer()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 10 seconds...");
    mqtt.disconnect();
    delay(10000); // wait 10 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void setup() {
  Serial.begin(115200);

  floatSwitch.attach(FLOAT_SWITCH_PIN,  INPUT_PULLUP);
  floatSwitch.interval(10);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);

  // Connect to Wifi
  setupWifi();

  // Randomize seed
  randomSeed(micros());

  // Set MQTT callback function
  subscriber.setCallback(&callback);

  // Setup MQTT subscription for time feed.
  mqtt.subscribe(&subscriber);

  keepAlive = 0;
  floatSwitch.update();
  int floatSwitchInput = floatSwitch.read();
  digitalWrite(LED_PIN, floatSwitchInput);
  digitalWrite(RELAY_1_PIN, floatSwitchInput);
  digitalWrite(RELAY_2_PIN, floatSwitchInput);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) connectToMQTTServer();
  if (mqtt.connected()) mqtt.processPackets(10);

  if (pumpOverride == 1) return;

  if (keepAlive && millis() - timer >= keepAliveTimeout) {
    keepAlive = 0;    
    digitalWrite(RELAY_1_PIN, LOW);
    digitalWrite(RELAY_2_PIN, LOW);
    Serial.println("Relay off");
    sendMsg(RELAY_OFF);
  }

  floatSwitch.update();

  if (floatSwitch.changed()) {
    int floatSwitchInput = floatSwitch.read();

    if (floatSwitchInput == HIGH) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(RELAY_1_PIN, HIGH);
      digitalWrite(RELAY_2_PIN, HIGH);
      Serial.println("Switch On");
      Serial.println("Relay On");
      sendMsg(SWITCH_ON);
      sendMsg(RELAY_ON);
    } else {
      digitalWrite(LED_PIN, LOW);
      Serial.println("Switch Off");
      sendMsg(SWITCH_OFF);
      keepAlive = 1;
      timer = millis();
    }
  }
}

void sendMsg(STATES state) {
  if (!mqtt.connected()) {
    Serial.println("Not connected to broker");
    return;
  }
  switch(state) {
    case RELAY_ON:
      publisher.publish("RELAY_ON");
      break;
    case RELAY_OFF:
      publisher.publish("RELAY_OFF");
      break;
    case SWITCH_ON:
      publisher.publish("SWITCH_ON");
      break;
    case SWITCH_OFF:
      publisher.publish("SWITCH_OFF");
      break;
  }
}