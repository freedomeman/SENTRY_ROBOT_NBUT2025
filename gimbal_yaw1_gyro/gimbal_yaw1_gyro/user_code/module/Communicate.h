#ifndef COMMUNICAT_H
#define COMMUNICAT_H

#include "cmsis_os.h"
#include "main.h"

#include "Remote_control.h"
#include "Can_receive.h"





class Communicate
{
public:

    RC_ctrl_t remote_control_yaw; //小yaw用于接收can传回来遥控器数据的内存
    RC_ctrl_t remote_control_yaw_last; 



    void init();

    void run();

    bool game_start();

    const RC_ctrl_t * get_rc_yaw_point(void);

    RC_ctrl_t * get_rc_yaw_point_last(void);


    uint32_t systime;
};

extern Remote_control remote_control;

extern Can_receive can_receive;

extern Communicate communicate;

#endif

