#include <Arduino.h>
#include <Bounce2.h>

#define FLOAT_SWITCH_PIN 25
#define LED 23
#define RELAY_1 12
#define RELAY_2 27 

Bounce floatSwitch = Bounce();

static int keepAlive;
static long timer;
static int timeout = 60 * 1000; // 1 Minute

void setup() {
  Serial.begin(9600);
  floatSwitch.attach(FLOAT_SWITCH_PIN,  INPUT_PULLUP);
  floatSwitch.interval(10);
  pinMode(LED, OUTPUT);
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);

  
  floatSwitch.update();
  int floatSwitchInput = floatSwitch.read();
  keepAlive = floatSwitchInput;
  digitalWrite(LED, floatSwitchInput);
  digitalWrite(RELAY_1, floatSwitchInput);
  digitalWrite(RELAY_2, floatSwitchInput);
}

void loop() {
  if (keepAlive && millis() - timer >= timeout) {
    keepAlive = 0;    
    digitalWrite(RELAY_1, LOW);
    digitalWrite(RELAY_2, LOW);
    Serial.println("Relay off");
  }

  floatSwitch.update();

  if (floatSwitch.changed()) {
    int floatSwitchInput = floatSwitch.read();

    if (floatSwitchInput == HIGH) {
      digitalWrite(LED, HIGH);
      digitalWrite(RELAY_1, HIGH);
      digitalWrite(RELAY_2, HIGH);
      Serial.println("Switch On");
      Serial.println("Relay On");
    } else {
      digitalWrite(LED, LOW);
      Serial.println("Switch Off");
      keepAlive = 1;
      timer = millis();
    }
  }
}
