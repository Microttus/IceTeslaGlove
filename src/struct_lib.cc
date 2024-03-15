//
// Created by biomech on 14.03.24.
//

struct HandDouble{
  double thumb;
  double index;
  double middle;
  double ring;
  double little;
  double palm;
};

struct HandServoPos{
  int thumb;
  int index;
  int middle;
  int ring;
  int little;
  int palm;
};

struct SpringConst{
  double K;
  double D;
  double M;
};

struct HandSpring{
  SpringConst thumb;
  SpringConst index;
  SpringConst middle;
  SpringConst ring;
  SpringConst little;
  SpringConst palm;
};

struct FingerStorage{
  double last_pos;
  double last_speed;
  double cur_pos;
  double cur_speed;
  double cur_acc;
  SpringConst spring;
};

struct HandStorage{
  FingerStorage thumb;
  FingerStorage index;
  FingerStorage middle;
  FingerStorage ring;
  FingerStorage little;
  FingerStorage palm;
};