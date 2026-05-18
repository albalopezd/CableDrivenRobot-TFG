#ifndef PCCkinematics_h
#define PCCkinematics_h

#include "Arduino.h"
#include <math.h>

class PCCKinematics {
    public:
      PCCKinematics::PCCKinematics(){}
      
      static void invkinematics(float x, float y, float z, float& theta, float& phi){
        phi = atan2(y, x);

        float xy2 = x*x + y*y;
        float xy   = sqrt(xy2);
        float k    = 2.0f * xy / (xy2 + z*z);

        float acos_arg = constrain(1.0f - k * xy, -1.0f, 1.0f);

        theta = (z > 0) ? acos(acos_arg) : (2.0f * M_PI - acos(acos_arg));
      }
      
      static void fwdKinematics(float c1, float c2, float c3, float& x, float& y, 
        float& z, float robot_length, float d_cable, int n_seg) 
      {
        float d = d_cable * n_seg;
        float A  = c1 / d;
        float B  = (c2 - c3) / (d * sqrt(3.0f));

        float theta = sqrt(A*A + B*B);
        float phi   = atan2(A, B);

        if (theta < 1e-6f) { 
            x = 0.0f; y = 0.0f; z = robot_length;
            return;
        }
        float r = robot_length / theta;
        x = r * cos(phi) * (1.0f - cos(theta));
        y = r * sin(phi) * (1.0f - cos(theta));
        z = r * sin(theta);
      }
      
      static void getCablePull(float theta_total, float phi, float& c1, float& c2, 
        float& c3, float d_cable, float PHI_C1, float PHI_C2, float PHI_C3) {
        float base = d_cable * theta_total;
        c1 = base * sin(phi + PHI_C1);
        c2 = base * sin(phi + PHI_C2);
        c3 = base * sin(phi + PHI_C3);
      }
    private:
  };

#endif
