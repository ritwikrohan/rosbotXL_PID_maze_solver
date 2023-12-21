#include "pid.hpp"

#include <iostream>

#define WINDOWSIZE 25

using namespace std;

/*
 * : Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    rollingaccumulator.resize(WINDOWSIZE, 0.0);
    rollingindex = WINDOWSIZE - 1;
}

void PID::UpdateError(double cte) {
    // double dt = timestamp - last_timestamp;
    d_error = (cte - p_error);
    p_error = cte;

    if (Ki != 0.0 && rollingaccumulator.size() > 0) {
        rollingindex = (rollingindex + 1) % WINDOWSIZE;
        double head = rollingaccumulator[rollingindex];
        rollingaccumulator[rollingindex] = cte;
        i_error += cte - head ;
    } else {
        i_error += cte;
    }

    // last_timestamp = timestamp;
}



double PID::TotalError() {
    return 0.0;
}