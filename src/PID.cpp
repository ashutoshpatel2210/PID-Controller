#include "PID.h"
#include <cmath>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    // Params , Propotional, Integral and diifferential
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
  
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
  
    prev_cte = 0.0;
  
    cte_error = 0.0;
    max_error = -9999999.00;
    min_error = 9999999.00;
  
    counter = 0;
  
    /*  
    dp_p = 1.0;
    dp_i = 1.0;
    dp_d = 1.0; */
  
    
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   p_error = cte;
   i_error += cte;
   d_error = cte - prev_cte;
   
   prev_cte = cte;
  
   cte_error += fabs(cte);  
  
   if (fabs(cte) > max_error)
     max_error =  cte;
  
   if (fabs(cte) < min_error)
     min_error =  cte;
  
   counter++;
    
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return (p_error * -Kp) + (i_error * -Ki) + (d_error * -Kd);  // TODO: Add your total error calc here!
}

int PID::Counter() {

  return counter;
}

double PID::AvgError() {

  return cte_error/counter;
}

double PID::MinError() {

  return min_error;
}

double PID::MaxError() {

  return max_error;
}
