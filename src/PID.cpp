#include "PID.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <math.h>

using namespace std;

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

	is_initialized = false;
	best_error = 0.0;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	prev_cte = 0.0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	p_error = cte;
	i_error += cte;
	d_error = cte - prev_cte;
	prev_cte = cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */

  double total_error = -1 * ((Kp * p_error) + (Kd * d_error) + (Ki * i_error));
  return total_error;  // TODO: Add your total error calc here!
}

void PID::Twiddle(double tolerance){

	vector<double> params = {Kp, Ki, Kd};
	vector<double> deltas = {1.0, .001, .1};

	//double best_error = PID::TotalError();
	double error;

	double sum = accumulate(deltas.begin(), deltas.end(), 0.0);

	while(sum > tolerance){
		for(unsigned int i = 0; i < params.size(); i++){
			params[i] += deltas[i];
			Kp = params[0];
			Ki = params[1];
			Kd = params[2];
			error = PID::TotalError();

			if(fabs(error) < fabs(best_error)){
				best_error = error;
				deltas[i] *= 1.1;
			}else {
				params[i] -= 2.0 * deltas[i];
				Kp = params[0];
				Ki = params[1];
				Kd = params[2];
				error = PID::TotalError();

				if(fabs(error) < fabs(best_error)){
					best_error = error;
					deltas[i] *=1.1;
				}else {
					params[i] += deltas[i];
					Kp = params[0];
					Ki = params[1];
					Kd = params[2];
					deltas[i] *= 0.9;
				}
			}

		}
		sum = accumulate(deltas.begin(), deltas.end(), 0.0);
	}
	Kp = params[0];
	Ki = params[1];
	Kd = params[2];
	cout << "Kp:      " << Kp << "; Ki:      " << Ki << "; Kd:      " << Kd << endl;
	cout << "p_error: " << p_error << "; i_error: " << i_error << "; d_error: " << d_error << endl;
	//is_initialized = true;
}
