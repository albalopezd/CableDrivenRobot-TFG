#include "TendonRobot.h"

TendonRobot::TendonRobot(LX16A& servos, int n_seg, float d_cable,
                           float pulley_radius, float phi_c1, float phi_c2,
                           float phi_c3, uint8_t ids[3],
                           float max_position_servo, float max_angle_servo)
    : servos_(servos),         
      n_seg_(n_seg),
      d_cable_(d_cable),
      pulley_radius_(pulley_radius),
      phi_c1_(phi_c1),
      phi_c2_(phi_c2),
      phi_c3_(phi_c3),
      max_position_servo_(max_position_servo),
      max_angle_servo_(max_angle_servo)
{
    ids_[0] = ids[0];
    ids_[1] = ids[1];
    ids_[2] = ids[2];
}

void TendonRobot::moveTo(float x, float y, float z, uint16_t time_ms)
{
  float theta, phi;
  PCCKinematics::invkinematics(x, y, z, theta, phi);
  
  float c1, c2, c3;
  PCCKinematics::getCablePull(theta * n_seg_, phi, c1, c2, c3, d_cable_, 
                          phi_c1_, phi_c2_, phi_c3_);
  
  servos_.move(ids_[0], cableToServoPos(c1), time_ms);
  servos_.move(ids_[1], cableToServoPos(c2), time_ms);
  servos_.move(ids_[2], cableToServoPos(c3), time_ms);
}

uint16_t TendonRobot::cableToServoPos(float cable_length) {
    float angle_rad = cable_length / pulley_radius_;
    float angle_deg = angle_rad * (180.0f / M_PI);
    int delta = (int)(angle_deg * (max_position_servo_ / max_angle_servo_));
    return (uint16_t)constrain((max_position_servo_ / 2) + delta, 0, max_position_servo_);
}
