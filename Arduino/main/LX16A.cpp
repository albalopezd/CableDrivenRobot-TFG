#include "LX16A.h"

#define FRAME_HEADER     0x55
#define MOVE_TIME_WRITE  1
#define MOVE_STOP        12
#define POS_READ         28
#define ID_WRITE         13
#define LOAD_UNLOAD      31

LX16A::LX16A(Stream& serial) : serial_(serial) {}

byte LX16A::checksum(byte buf[]) {
    uint16_t temp = 0;
    for (byte i = 2; i < buf[3] + 2; i++) temp += buf[i];
    return (byte)(~temp);
}

void LX16A::move(uint8_t id, uint16_t position, uint16_t time_ms) {
    position = constrain(position, 0, 1000);
    byte buf[10];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 7;
    buf[4] = MOVE_TIME_WRITE;
    buf[5] = (byte)(position);
    buf[6] = (byte)(position >> 8);
    buf[7] = (byte)(time_ms);
    buf[8] = (byte)(time_ms >> 8);
    buf[9] = checksum(buf);
    serial_.write(buf, 10);
}

void LX16A::stop(uint8_t id) {
    byte buf[6];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 3;
    buf[4] = MOVE_STOP;
    buf[5] = checksum(buf);
    serial_.write(buf, 6);
}

void LX16A::unload(uint8_t id) {
    byte buf[7];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = LOAD_UNLOAD;
    buf[5] = 0;
    buf[6] = checksum(buf);
    serial_.write(buf, 7);
}

void LX16A::load(uint8_t id) {
    byte buf[7];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = LOAD_UNLOAD;
    buf[5] = 1;
    buf[6] = checksum(buf);
    serial_.write(buf, 7);
}

int LX16A::readPosition(uint8_t id) {
    byte req[6];
    req[0] = req[1] = FRAME_HEADER;
    req[2] = id;
    req[3] = 3;
    req[4] = POS_READ;
    req[5] = checksum(req);

    while (serial_.available()) serial_.read();
    serial_.write(req, 6);

    int count = 10000;
    while (!serial_.available()) {
        if (--count < 0) return -1;
    }

    byte ret[32];
    if (receiveHandle(ret) > 0)
        return (int16_t)((((uint16_t)ret[2]) << 8) | ret[1]);

    return -1;
}

void LX16A::setID(uint8_t oldID, uint8_t newID) {
    byte buf[7];
    buf[0] = buf[1] = FRAME_HEADER;
    buf[2] = oldID;
    buf[3] = 4;
    buf[4] = ID_WRITE;
    buf[5] = newID;
    buf[6] = checksum(buf);
    serial_.write(buf, 7);
}

int LX16A::receiveHandle(byte* ret) {
    bool frameStarted = false;
    byte frameCount = 0, dataCount = 0, dataLength = 2;
    byte rxBuf;
    byte recvBuf[32];

    while (serial_.available()) {
        rxBuf = serial_.read();
        delayMicroseconds(100);
        if (!frameStarted) {
            if (rxBuf == FRAME_HEADER && ++frameCount == 2) {
                frameCount = 0;
                frameStarted = true;
                dataCount = 1;
            } else if (rxBuf != FRAME_HEADER) {
                frameCount = 0;
            }
        }
        if (frameStarted) {
            recvBuf[dataCount] = rxBuf;
            if (dataCount == 3) {
                dataLength = recvBuf[dataCount];
                if (dataLength < 3 || dataLength > 7) {
                    frameStarted = false;
                    continue;
                }
            }
            if (++dataCount == dataLength + 3) {
                if (checksum(recvBuf) == recvBuf[dataCount - 1]) {
                    memcpy(ret, recvBuf + 4, dataLength);
                    return 1;
                }
                return -1;
            }
        }
    }
    return 0;
}
