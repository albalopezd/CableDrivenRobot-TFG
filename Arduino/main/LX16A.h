#ifndef LX16A_h
#define LX16A_h

#include <Arduino.h>

class LX16A {
    public:
        LX16A(Stream& serial);
        void move(uint8_t id, uint16_t position, uint16_t time_ms);
        void stop(uint8_t id);
        void unload(uint8_t id);
        void load(uint8_t id);
        int  readPosition(uint8_t id);
        void setID(uint8_t oldID, uint8_t newID);
    private:
        Stream& serial_;
        int  receiveHandle(byte* ret);
        byte checksum(byte buf[]);
};

#endif
