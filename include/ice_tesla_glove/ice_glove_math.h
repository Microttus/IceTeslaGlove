//
// Created by biomech on 15.03.24.
//

#ifndef ICE_TESLA_GLOVE_SRC_ICE_GLOVE_MATH_H_
#define ICE_TESLA_GLOVE_SRC_ICE_GLOVE_MATH_H_

#include "haptic_structs.h"

class IceGloveMath {
 public:
  IceGloveMath() = default;

  double map_output(double in_val, double out_min, double out_max);
  double to_norm_val_with_guard(float input_val, double upper_val, double lower_val, double last_val=0);
  double admittance_calc(FingerStorage finger, double force, double dt=0.01);

};

#endif //ICE_TESLA_GLOVE_SRC_ICE_GLOVE_MATH_H_
