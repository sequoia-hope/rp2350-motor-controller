#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Hello World");
}

void loop() {
    if (Serial.available()) {
        Serial.print("echo: ");
        Serial.println(Serial.readStringUntil('\n'));
    }
}
