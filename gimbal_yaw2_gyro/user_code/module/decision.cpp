
#include "decision.h"

Decision decision;
extern Remote_control remote_control;

void Decision::decision_init(void)
{
    remote_control_robot = remote_control.get_remote_control_point();
    remote_control_robot_last = remote_control.get_last_remote_control_point();

    robot_mode = ROBOT_ZERO_FORCE;
}

void Decision::robot_set_mode(void)
{
    
}

uint8_t sw;
void Decision::remote_switch(void)
{
    if(remote_control_robot->rc.ch[4] < -600 && sw == 1)
    {
        //temp_v = 512;//对应键盘值"F"
        if (robot_mode == YAW2_CTRL)
        {
            robot_mode = YAW1_CTRL;
            /* code */
        }
        else if (robot_mode == YAW1_CTRL)
        {
            robot_mode = YAW2_CTRL;
            /* code */
        }
        sw = 0;
    }
    else if (remote_control_robot->rc.ch[4] == 0)
    {
        /* code */
        sw = 1;
    }

    if (robot_mode == YAW2_CTRL && remote_control_robot->rc.s[1] == 2)
    {
        robot_mode = ROBOT_ZERO_FORCE;
        /* code */
    }
    if (robot_mode == ROBOT_ZERO_FORCE && remote_control_robot->rc.s[1] == 3)
    {
        robot_mode = YAW2_CTRL;
        /* code */
    }
    
    

}

void Decision::robot_set_control(void)
{

    //保存遥控器值
    remote_ctrl_yaw2_last = remote_ctrl_yaw2;
    remote_ctrl_yaw1_last = remote_ctrl_yaw1;

    if (robot_mode == ROBOT_ZERO_FORCE) //无力模式
    {
        remote_ctrl_yaw2.rc.ch[0] = 0;
        remote_ctrl_yaw2.rc.ch[1] = 0;
        remote_ctrl_yaw2.rc.ch[2] = 0;
        remote_ctrl_yaw2.rc.ch[3] = 0;
        remote_ctrl_yaw2.rc.ch[4] = 0;
        remote_ctrl_yaw2.rc.s[0] = RC_SW_DOWN;
        remote_ctrl_yaw2.rc.s[1] = RC_SW_MID;

        remote_ctrl_yaw1.rc.ch[0] = 0;
        remote_ctrl_yaw1.rc.ch[1] = 0;
        remote_ctrl_yaw1.rc.ch[2] = 0;
        remote_ctrl_yaw1.rc.ch[3] = 0;
        remote_ctrl_yaw1.rc.ch[4] = 0;
        remote_ctrl_yaw1.rc.s[0] = RC_SW_DOWN;
        remote_ctrl_yaw1.rc.s[1] = RC_SW_MID;
        /* code */
    }
    else if (robot_mode == YAW1_CTRL)//控制小yaw
    {
        remote_ctrl_yaw1.rc.ch[0] = remote_control_robot->rc.ch[0];
        remote_ctrl_yaw1.rc.ch[1] = remote_control_robot->rc.ch[1];
        remote_ctrl_yaw1.rc.ch[2] = remote_control_robot->rc.ch[2];
        remote_ctrl_yaw1.rc.ch[3] = remote_control_robot->rc.ch[3];
        remote_ctrl_yaw1.rc.ch[4] = remote_control_robot->rc.ch[4];
        remote_ctrl_yaw1.rc.s[0] = remote_control_robot->rc.s[0];
        remote_ctrl_yaw1.rc.s[1] = remote_control_robot->rc.s[1];

        remote_ctrl_yaw2.rc.ch[0] = 0;
        remote_ctrl_yaw2.rc.ch[1] = 0;
        remote_ctrl_yaw2.rc.ch[2] = 0;
        remote_ctrl_yaw2.rc.ch[3] = 0;
        remote_ctrl_yaw2.rc.ch[4] = 0;
        /* code */
    }
    else if (robot_mode == YAW2_CTRL)//控制大yaw
    {
        remote_ctrl_yaw2.rc.ch[0] = remote_control_robot->rc.ch[0];
        remote_ctrl_yaw2.rc.ch[1] = remote_control_robot->rc.ch[1];
        remote_ctrl_yaw2.rc.ch[2] = remote_control_robot->rc.ch[2];
        remote_ctrl_yaw2.rc.ch[3] = remote_control_robot->rc.ch[3];
        remote_ctrl_yaw2.rc.ch[4] = remote_control_robot->rc.ch[4];
        remote_ctrl_yaw2.rc.s[0] = remote_control_robot->rc.s[0];
        remote_ctrl_yaw2.rc.s[1] = remote_control_robot->rc.s[1];

        remote_ctrl_yaw1.rc.ch[0] = 0;
        remote_ctrl_yaw1.rc.ch[1] = 0;
        remote_ctrl_yaw1.rc.ch[2] = 0;
        remote_ctrl_yaw1.rc.ch[3] = 0;
        remote_ctrl_yaw1.rc.ch[4] = 0;
        /* code */
    }
}

const RC_ctrl_t* Decision::get_yaw2_ctrl_point(void)
{
    return &remote_ctrl_yaw2;
}

RC_ctrl_t* Decision::get_yaw2_ctrl_point_last(void)
{
    return &remote_ctrl_yaw2_last;
}

const RC_ctrl_t* Decision::get_yaw1_ctrl_point(void)
{
    return &remote_ctrl_yaw1;
}

RC_ctrl_t* Decision::get_yaw1_ctrl_point_last(void)
{
    return &remote_ctrl_yaw1_last;
}
