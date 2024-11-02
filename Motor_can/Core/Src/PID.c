/** 
* @file           :
* @brief          :
* @author         : Ho Quang Dung
* @date           :24/10/2024 
* @version        :V1.0
* @par Copyright (c):  
*               /
* @par History: 1:Create         
*               /
* @par Reference     : 
*/
#include "pid.h"


void PID_INIT(PID_PARA *pid, float target, float sample_time, float outmax, float outmin, float kp, float ki, float kd) {
    // Thiết lập các tham số PID
    pid->target = target;
    pid->T = sample_time;
    pid->MaxOutput = outmax;
    pid->MinOutput = outmin;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    // Khởi tạo các giá trị ban đầu
    pid->output = 0;
    pid->output_last = 0;
    pid->last_error1 = 0;
    pid->last_error2 = 0;
    pid->error = 0;
}

void PID_CONTROLLER(PID_PARA *pid, float measure) {
    // Caculation error
    pid->measure = measure;
    pid->error = pid->target - pid->measure;

    // Deadband check
    float deadband = 0.5;
    if (ABS(pid->error) < deadband) {
        return; 
    }

    // 
    float incKp = pid->kp * (pid->error - pid->last_error1);
    float incKi = pid->ki * pid->T / 2.0f * (pid->error + pid->last_error1);
    float incKd = (pid->T > 0) ? (pid->kd / pid->T * (pid->error - 2 * pid->last_error1 + pid->last_error2)) : 0;

    // Tổng hợp các thành phần để có đầu ra mới
    pid->output = pid->output_last + incKp + incKi + incKd;

    // Limit output
    if (pid->output > pid->MaxOutput) {
        pid->output = pid->MaxOutput;
    } else if (pid->output < pid->MinOutput) {
        pid->output = pid->MinOutput;
    }

    // Store history value 
    pid->last_error2 = pid->last_error1;
    pid->last_error1 = pid->error;
    pid->output_last = pid->output;
}
