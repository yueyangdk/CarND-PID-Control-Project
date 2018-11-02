#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = d_error = i_error = 0.0;

    // Twiddling parameters

    twiddle = true;
    dp = {0.1*Kp,0.1*Kd,0.1*Ki};
    step = 1;
    param_index = 2;  // param_index will change to 0 after the first twiddle loop to process start from Proportional Control
    settle_steps = 100;  
    eval_steps = 2000;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    increased_flag = false; 
    decreased_flag = false;
}

void PID::UpdateError(double cte) {
    if (step == 1) {
        // to get correct initial d_error
        p_error = cte;
    }
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    // update total_error only if the vehicle have past the settling process 
    if (step % (settle_steps + eval_steps) > settle_steps){
        total_error += pow(cte,2);
    }

    // Twiddle loop process:
    // 
    // 1. Compare current total error with best error, 
    //    if total error > best error, whether current parameter will be increased or decreased depend on the state of the flag;
    //    if parameter both flags are false, increase current parameter by dp and set increase flag to true; 
    //    if increase flag is true, decrease current parameter by 2*dp and set decrease flag to true; 
    //    if both flags are true, change shrink dp by 10% and set both flags to false.
    // 2. if total error < best error, refresh best error, switch to next parameter and set both flag to false.
    if (twiddle && step % (settle_steps + eval_steps) == 0){
        cout << "step: " << step << endl;
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;
        if (total_error < best_error) {
            cout << "Parameter improved, index:" << param_index << endl;
            best_error = total_error;
            if (step !=  settle_steps + eval_steps) {
                // don't do this if it's the first loop of twiddle
                dp[param_index] *= 1.1;            
            }
            // switch to next parameter
            param_index = (param_index + 1) % 3;
            increased_flag = decreased_flag = false;
        }
        if (!increased_flag && !decreased_flag) {
            // increase params[i] by dp[i]
            AddToParameterAtIndex(param_index, dp[param_index]);
            increased_flag = true;
        }
        else if (increased_flag && !decreased_flag) {
            // decrease params[i] by dp[i], multiply by 2 because we already add dp[i] in previous loop
            AddToParameterAtIndex(param_index, -2 * dp[param_index]);     
            decreased_flag = true;         
        }
        else {
            // shrink dp[i], move on to next parameter
            AddToParameterAtIndex(param_index, dp[param_index]);      
            dp[param_index] *= 0.9;
            // next parameter
            param_index = (param_index + 1) % 3;
            increased_flag = decreased_flag = false;
        }
        total_error = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        
    }
    step++;
    if (twiddle && step % (n_settle_steps + n_eval_steps) == 0){
        cout << "step: " << step << endl;
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;    	
    }
}

double PID::TotalError() {
    return 0.0;  
}

void PID::AddToParameterAtIndex(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Kd += amount;
    }
    else if (index == 2) {
        Ki += amount;
    }
    else {
        std::cout << "AddToParameterAtIndex: index out of bounds";
    }
}

