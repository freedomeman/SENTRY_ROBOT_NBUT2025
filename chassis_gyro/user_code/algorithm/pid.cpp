#include "pid.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "user_lib.h"

#ifdef __cplusplus
}
#endif
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define LimitMax_Increase(input, max)\
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < 100) \
        {                      \
            input = 100;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_iout: pid最大积分输出
  * * @param[in]      max_out: pid最大输出
  * @retval         none
  */
void Pid::init(pid_mode_e mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_, fp32 erro_delta_)
{
    mode = mode_;
    data.Kp = pid_parm[0];
    data.Ki = pid_parm[1];
    data.Kd = pid_parm[2];
    data.max_iout = pid_parm[3];
    data.max_out = pid_parm[4];

    data.set = set_;
    data.ref = ref_;
    data.error = *set_ - *ref_;

    if (data.mode == PID_ANGLE)
        data.error_delta = erro_delta_;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
 fp32 Pid::pid_calc()
 {
     data.last_error = data.error;
     data.error = *data.set - *data.ref;
     if (mode == PID_SPEED)
         data.error_delta = data.error - data.last_error;

     if (mode == PID_ANGLE){
        data.error = rad_format(data.error);
        data.error_delta = data.error - data.last_error;       
         }

     data.Pout = data.Kp * data.error;
     data.Iout += data.Ki * data.error;
     data.Dout = data.Kd * (data.error_delta);

     LimitMax(data.Iout, data.max_iout);

     data.out = data.Pout + data.Iout + data.Dout;
     LimitMax(data.out, data.max_out);

     return data.out;
}

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void Pid::pid_clear()
{
    data.last_error = data.error = *data.set = *data.ref = 0;
    data.out = data.Pout = data.Iout = data.Dout = 0;
}

/**
 * @brief 增量式pid初始化
 * 
 * @param mode_ 
 * @param pid_parm 
 * @param ref_ 
 * @param set_ 
 * @param erro_delta_ 
 * @param Increase_date 
 */
void Pid::Increase_pid_init(pid_mode_e mode_, const fp32 *pid_parm, fp32 *ref_, fp32 *set_, fp32 erro_delta_,fp32 *Increase_date)
{
    mode = mode_;
    data.Kp = pid_parm[0];
    data.Ki = pid_parm[1];
    data.Kd = pid_parm[2];
    data.max_iout = pid_parm[3];
    data.max_out = pid_parm[4];

    data.set = set_;
    data.ref = ref_;
    data.error = *set_ - *ref_;

    data.In = Increase_date;

    if (data.mode == PID_ANGLE)
        data.error_delta = erro_delta_;
}

fp32 Pid::Increase_pid_calc()
{
    //计算差值
    data.last_error = data.error;
    data.error = *data.set - *data.ref;
    data.error_delta = data.error - data.last_error;
    //运算
    data.Pout = data.Kp * data.error;
    data.Iout += data.Ki * data.error;
    data.Dout = data.Kd * (data.error_delta);

    //LimitMax_Increase(data.Iout, data.max_iout);

    
    data.out = data.Pout + data.Iout + data.Dout + *data.In;
    LimitMax_Increase(data.out, data.max_out);

    return data.out;
}
