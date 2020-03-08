#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
   p_error = 0.0;
   i_error = 0.0;
   d_error = 0.0;

   cte_prev = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
   d_error = cte - cte_prev;   // calculate differential term
   cte_prev = cte;
   p_error = cte;             // assign cte to proportional term
   i_error += cte;            // sum all ctes over time

}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return -Kp * p_error - Ki* i_error - Kd * d_error;
}
