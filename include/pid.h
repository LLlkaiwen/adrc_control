#ifndef PID_H_
#define PID_H_

#include <stdint.h>
typedef enum PID_MODE {
  /* Use PID_MODE_DERIVATIV_NONE for a PI controller (vs PID) */
  PID_MODE_DERIVATIV_NONE = 0,
  /* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error,
   * val_dot in pid_calculate() will be ignored */
  PID_MODE_DERIVATIV_CALC,
  /* PID_MODE_DERIVATIV_CALC_NO_SP calculates discrete derivative from previous value,
   * setpoint derivative will be ignored, val_dot in pid_calculate() will be ignored */
  PID_MODE_DERIVATIV_CALC_NO_SP,
  /* Use PID_MODE_DERIVATIV_SET if you have the derivative already (Gyros, Kalman) */
  PID_MODE_DERIVATIV_SET
} pid_mode_t;

typedef struct
{
  pid_mode_t mode;
  float dt_min;
  float kp;
  float ki;
  float kd;
  float integral;
  float integral_limit;
  float output_limit;
  float error_previous;
  float last_output;
} PID_t;

void pid_init(PID_t *pid, pid_mode_t mode, float dt_min);
int pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt);
void pid_reset_integral(PID_t *pid);

#endif /* PID_H_ */