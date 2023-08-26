#include <Arduino.h>
#include <Bounce2.h>

#define FLOAT_SWITCH_PIN 25
#define LED_PIN 23
#define RELAY_1_PIN 12
#define RELAY_2_PIN 27 

Bounce floatSwitch = Bounce();

static int keepAlive;
static long timer;
static int timeout = 60 * 1000; // 1 Minute

void setup() {
  Serial.begin(9600);
  floatSwitch.attach(FLOAT_SWITCH_PIN,  INPUT_PULLUP);
  floatSwitch.interval(10);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);

  keepAlive = 0;
  floatSwitch.update();
  int floatSwitchInput = floatSwitch.read();
  digitalWrite(LED_PIN, floatSwitchInput);
  digitalWrite(RELAY_1_PIN, floatSwitchInput);
  digitalWrite(RELAY_2_PIN, floatSwitchInput);
}

void loop() {
  if (keepAlive && millis() - timer >= timeout) {
    keepAlive = 0;    
    digitalWrite(RELAY_1_PIN, LOW);
    digitalWrite(RELAY_2_PIN, LOW);
    Serial.println("Relay off");
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
    } else {
      digitalWrite(LED_PIN, LOW);
      Serial.println("Switch Off");
      keepAlive = 1;
      timer = millis();
    }
  }
}
