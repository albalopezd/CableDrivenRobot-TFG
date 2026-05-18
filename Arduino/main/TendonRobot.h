#ifndef TendonRobot_h
#define TendonRobot_h

#include "Arduino.h"
#include "PCCkinematics.h"
#include "LX16A.h"
#include <math.h>

class TendonRobot {
    public:
      TendonRobot(LX16A& servos, int n_seg, float d_cable, float pulley_radius,
                   float phi_c1, float phi_c2, float phi_c3, uint8_t ids[3],
                   float max_position_servo, float max_angle_servo);
      void moveTo(float x, float y, float z, uint16_t time_ms);
      uint16_t cableToServoPos(float cable_length);
    private:
      LX16A& servos_;
      PCCKinematics kinematics_;
      int n_seg_;
      float d_cable_;
      float pulley_radius_;
      float phi_c1_;
      float phi_c2_;
      float phi_c3_;
      uint8_t ids_[3];
      float max_position_servo_;
      float max_angle_servo_;
};

#endif
