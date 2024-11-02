#ifndef __PID_H_
#define __PID_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* ------------------------------------------------------------------ */

/* Check compiler */
#ifdef __CODEVISIONAVR__  
    /* Đoạn mã đặc biệt cho CodeVisionAVR (nếu cần) */
#endif
#define ABS(x)		((x>0)? x: -x)
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  PID_PARA ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

typedef struct {
    float target;         // Giá trị mục tiêu
    float measure;        // Giá trị đo hiện tại
    float error;          // Sai số hiện tại
    float kp, ki, kd;     // Các hệ số PID
    float T;              // Thời gian lấy mẫu
    float output;         // Giá trị đầu ra hiện tại
    float output_last;    // Giá trị đầu ra trước đó
    float MaxOutput;      // Giới hạn đầu ra trên
    float MinOutput;      // Giới hạn đầu ra dưới
    float last_error1;    // Sai số lần trước
    float last_error2;    // Sai số lần trước hai
} PID_PARA;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Khai báo hàm ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Hàm khởi tạo PID */
void PID_INIT(PID_PARA *pid, float target, float sample_time, float outmax, float outmin, float kp, float ki, float kd);

/* Hàm điều khiển PID */
void PID_CONTROLLER(PID_PARA *pid, float measure);

#endif /* __PID_H_ */

