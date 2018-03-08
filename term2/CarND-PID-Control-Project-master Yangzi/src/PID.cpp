#include "PID.h"

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  cte_integral += cte;
  p_error = - Kp * cte;
  i_error = - Ki * cte_integral;
  d_error = - Kd * (cte - pre_cte);
  pre_cte = cte;
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

