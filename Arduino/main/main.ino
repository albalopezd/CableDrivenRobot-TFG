#include <SoftwareSerial.h>
#include "LX16A.h"
#include "PCCkinematics.h"
#include "TendonRobot.h"

SoftwareSerial lx16aSerial(10, 11);
LX16A servos(lx16aSerial);

const uint8_t SERVO_IDS[3] = {10, 4, 7};
TendonRobot robot(
  servos,
  4,
  0.02f,
  0.004f,
  1.5708f,
  -1.5708f,
  0.0f,
  SERVO_IDS,
  1000.0f,
  240.0f
);

String inputBuffer = "";

void parseAndMove(const String& line) {
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);

    if (i1 < 0 || i2 < 0) {
        Serial.println("ERR:formato incorrecto, usar x,y,z");
        return;
    }

    float x = line.substring(0, i1).toFloat();
    float y = line.substring(i1 + 1, i2).toFloat();
    float z = line.substring(i2 + 1).toFloat();

    robot.moveTo(x, y, z, 1000);
    Serial.println("OK");
}

void setup() {
  Serial.begin(9600);
  lx16aSerial.begin(115200);
  delay(500);
  Serial.println("Set up done");
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            parseAndMove(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}