//
// Created by biomech on 15.03.24.
//

#include "../include/ice_tesla_glove/ice_glove_math.h"

#include "../include/ice_tesla_glove/haptic_structs.h"

double IceGloveMath::map_output(double in_val, double out_min, double out_max){
  return (in_val - 0.0f) * (out_max - out_min) / (1.0f - 0.0f) + out_min;
}

// Function to normalize position values with guard for outliers
double IceGloveMath::to_norm_val_with_guard(float input_val, double upper_val, double lower_val, double last_val){
  auto d_input_val = static_cast<double>(input_val);
  if (d_input_val > upper_val or d_input_val < lower_val) {
    return last_val;
  } else {
    return (d_input_val - lower_val) / (upper_val - lower_val);
  }
}
double IceGloveMath::admittance_calc(FingerStorage finger, double force, double dt) {

  finger.cur_speed = (finger.last_pos - finger.cur_pos)/dt;
  finger.cur_acc = (finger.last_speed - finger.cur_speed)/dt;

  finger.cur_pos = (1/finger.spring.K)*(force - finger.spring.M*finger.cur_acc - finger.spring.D*finger.cur_speed);

  return finger.cur_pos;
}
