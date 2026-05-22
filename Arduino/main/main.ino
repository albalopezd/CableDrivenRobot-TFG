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

// Velocidades actuales de los servos
int16_t currentSpeeds[3] = {0, 0, 0};

// Voltajes leídos (mV), -1 si no disponible
int voltages[3] = {-1, -1, -1};

unsigned long lastVinRead = 0;
uint8_t vinIndex = 0;   // qué servo leer en este ciclo (0, 1, 2)

void parseVelocity(const String& line) {
    // Formato: "v:s0,s1,s2"
    int colon = line.indexOf(':');
    if (colon < 0) return;
    String data = line.substring(colon + 1);

    int i1 = data.indexOf(',');
    int i2 = data.indexOf(',', i1 + 1);
    if (i1 < 0 || i2 < 0) return;

    currentSpeeds[0] = (int16_t)data.substring(0, i1).toInt();
    currentSpeeds[1] = (int16_t)data.substring(i1 + 1, i2).toInt();
    currentSpeeds[2] = (int16_t)data.substring(i2 + 1).toInt();

    servos.setMotorSpeed(SERVO_IDS[0], currentSpeeds[0]);
    servos.setMotorSpeed(SERVO_IDS[1], currentSpeeds[1]);
    servos.setMotorSpeed(SERVO_IDS[2], currentSpeeds[2]);
}

void parseAndMove(const String& line) {
    // Formato legado: "x,y,z" para control IK standalone
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    if (i1 < 0 || i2 < 0) return;

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
    // Procesar comandos entrantes
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            if (inputBuffer.startsWith("v:")) {
                parseVelocity(inputBuffer);
            } else {
                parseAndMove(inputBuffer);
            }
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }

    // Leer voltaje de un servo cada 500ms (rotando entre los 3)
    unsigned long now = millis();
    if (now - lastVinRead >= 500) {
        lastVinRead = now;

        int vin = servos.readVin(SERVO_IDS[vinIndex]);
        voltages[vinIndex] = vin;
        vinIndex = (vinIndex + 1) % 3;

        // Cuando hemos leído los 3, enviar todos
        if (vinIndex == 0) {
            Serial.print("w:");
            Serial.print(voltages[0]);
            Serial.print(",");
            Serial.print(voltages[1]);
            Serial.print(",");
            Serial.println(voltages[2]);
        }
    }
}
