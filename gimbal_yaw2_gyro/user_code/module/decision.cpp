
#include "decision.h"
#include "string.h"
#include "Can_receive.h"
#include "gimbal.h"



#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"

#ifdef __cplusplus
}
#endif


Decision decision;
extern Remote_control remote_control;
extern Can_receive can_receive;
extern Gimbal gimbal;

void Decision::decision_init(void)
{
    remote_control_robot = remote_control.get_remote_control_point();
    remote_control_robot_last = remote_control.get_last_remote_control_point();

    robot_mode = ROBOT_ZERO_FORCE;

    vset_to_remotset.x_set = 180;
    vset_to_remotset.y_set = -280;
    vset_to_remotset.z_set = -280;
}

void Decision::robot_set_mode(void)
{
    
}

uint8_t sw,goto_sentry_time;
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

    if (robot_mode != YAW1_CTRL && remote_control_robot->rc.s[1] == 2)
    {
        robot_mode = ROBOT_ZERO_FORCE;
        /* code */
    }
    if (robot_mode == ROBOT_ZERO_FORCE && remote_control_robot->rc.s[1] == 3)
    {
        robot_mode = YAW2_CTRL;
        /* code */
    }
    if (robot_mode == YAW2_CTRL && remote_control_robot->rc.s[1] == 1)
    {
        goto_sentry_time++;
        if (goto_sentry_time == 125)
        {
            goto_sentry_time = 0;
            robot_mode = SENTRY_CTRL;
            /* code */
        }
        
        /* code */
    }
    else
    {
        goto_sentry_time = 0;
    }
    if (robot_mode == SENTRY_CTRL)
    {
        if (remote_control_robot->rc.s[1] != 1)
        {
            robot_mode = YAW2_CTRL;
            /* code */
        }
        
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
    else if (robot_mode == SENTRY_CTRL)
    {
        //sentry_mode_set();
        navi_set();
        /* code */
    }
    
}

//uint8_t hurt_direction=0;
void Decision::sentry_mode_set (void)
{
    sentry_behavior = GOTO_FIGHT;
    hp_last = hp_current;
    hp_current = can_receive.robot_decision_receive.hp;
    
    if (yaw1_status.self_aiming_status == 1) //这时小yaw已经瞄准到敌人
    {
        sentry_behavior = IS_FIGHTING;
        if (yaw1_status.yaw_encoder_angle >= 0.8)//小yaw到最右边
        {
            add_yaw_to_follow =0.8;
            /* code */
        }
        else if(yaw1_status.yaw_encoder_angle <= -0.8)
        {
            add_yaw_to_follow =-0.8;
        }
        else
        {
            add_yaw_to_follow =0;
        }
        /* code */
    }
    if ((hp_current - hp_last) >= 2)//此时说明受到伤害
    {
        avoid_damage_time = AVOID_TIME;
        //hurt_direction = can_receive.robot_decision_receive.by_hurt & 0x01;
        // if ((can_receive.robot_decision_receive.by_hurt & 0x01) != 0)//说明第一快装甲受击
        // {
        //     add_yaw_to_counterattack = -gimbal.gimbal_pitch_motor.encode_angle + 0.785;//这里装甲板逆时针放置
        //     /* code */
        // }
        if (using_small_gyroscope == 4)//已经关闭小陀螺
        {
            using_small_gyroscope = 1; //请求开启小陀螺
            /* code */
        }
		can_receive.robot_decision_receive.by_hurt = 0;
        /* code */
    }
    else
    {
        if (avoid_damage_time > 0)
        {
            avoid_damage_time --;
            /* code */
        }
        if (avoid_damage_time == 0 && using_small_gyroscope == 2) //已经开启小陀螺，并且一段时间没有受到伤害
        {
            using_small_gyroscope = 3;//请求关闭小陀螺
            /* code */
        }
        
        
        
    }
    if (can_receive.robot_decision_receive.hp <= LOW_HP) //如果血量低回家补血
    {
        sentry_behavior = GOTO_HOME ;
        /* code */
    }
    
    
}

void Decision::navi_set (void)
{
    remote_ctrl_yaw2.rc.ch[0] = receivenavistate.data.speed_vector.rollz * vset_to_remotset.z_set;
    remote_ctrl_yaw2.rc.ch[1] = 0;
    remote_ctrl_yaw2.rc.ch[2] = receivenavistate.data.speed_vector.speedy * vset_to_remotset.y_set;
    remote_ctrl_yaw2.rc.ch[3] = receivenavistate.data.speed_vector.speedx * vset_to_remotset.x_set;
    remote_ctrl_yaw2.rc.ch[4] = 0;
    remote_ctrl_yaw2.rc.s[0] = 1;
    remote_ctrl_yaw2.rc.s[1] = 3;

    remote_ctrl_yaw1.rc.ch[0] = 0;
    remote_ctrl_yaw1.rc.ch[1] = 0;
    remote_ctrl_yaw1.rc.ch[2] = 0;
    remote_ctrl_yaw1.rc.ch[3] = 0;
    remote_ctrl_yaw1.rc.ch[4] = 0;
    remote_ctrl_yaw1.rc.s[0] = 2;
    remote_ctrl_yaw1.rc.s[1] = 3;
}

void Decision::reveive_navi_status (uint8_t *data)
{
     //判断帧头数据是否为0xA5
    memcpy(&receivenavistate, data, 43);
}

uint8_t Navi_send_data[40];

void Decision::Send_Joint_Status (void)
{
    uint8_t send_length;
    send_length = sizeof(SendJointState);
    SendJointState.HeaderFrame.sof = 0x5A;
    SendJointState.HeaderFrame.len = send_length-6; //sizeof(SendGameStatusData.HeaderFrame);
    SendJointState.HeaderFrame.id  = 0x0C;
    SendJointState.time_stamp = 0;

    //send_length = sizeof(SendGameStatusData);

    SendJointState.HeaderFrame.crc = get_CRC8_check_sum((uint8_t*)(&SendJointState.HeaderFrame),3,0xff);

    SendJointState.data.yaw = gimbal.gimbal_yaw_motor.gyro_angle;
    SendJointState.data.pitch = 0;

    SendJointState.crc = get_CRC16_check_sum((uint8_t*)&SendJointState,send_length-2,0xffff);

    memcpy(Navi_send_data, &SendJointState, send_length);

    HAL_UART_Transmit(&huart1, Navi_send_data, send_length, 0xFFF);
  
    memset(Navi_send_data, 0, send_length);


}

uint8_t game_mode_user=3;

void Decision::Send_Gmae_Status (void)
{
    uint8_t send_length;
    send_length = sizeof(SendGameStatusData);
    SendGameStatusData.HeaderFrame.sof = 0X5A;
    SendGameStatusData.HeaderFrame.len = send_length-6; //sizeof(SendGameStatusData.HeaderFrame);
    SendGameStatusData.HeaderFrame.id  = 0X07;
    SendGameStatusData.time_stamp = 0;

    SendGameStatusData.HeaderFrame.crc = get_CRC8_check_sum((uint8_t*)(&SendGameStatusData.HeaderFrame),3,0xff);

    SendGameStatusData.data.game_progress = game_mode_user;//can_receive.gimbal_receive.game_progress;
    SendGameStatusData.data.stage_remain_time = 300;//can_receive.gimbal_receive.game_time;

    SendGameStatusData.crc = get_CRC16_check_sum((uint8_t*)&SendGameStatusData,send_length-2,0xffff);

    memcpy(Navi_send_data, &SendGameStatusData, send_length);

    HAL_UART_Transmit(&huart1, Navi_send_data, send_length, 0xFFF);
  
    memset(Navi_send_data, 0, send_length);
}

void Decision::Send_Bace_Status (void)
{
    uint8_t send_length;
    send_length = sizeof(SendBaceStatusData);
   SendBaceStatusData.HeaderFrame.sof = 0X5A;
   SendBaceStatusData.HeaderFrame.len = send_length-6; //sizeof(SendGameStatusData.HeaderFrame);
   SendBaceStatusData.HeaderFrame.id  = 0X01;
   SendBaceStatusData.time_stamp = 0;

   SendBaceStatusData.HeaderFrame.crc = get_CRC8_check_sum((uint8_t*)(&SendBaceStatusData.HeaderFrame),3,0xff);

   SendBaceStatusData.data.flag = 1;//(can_receive.gimbal_receive.game_progress>>2)&1;//can_receive.gimbal_receive.game_progress;为1可以运动
   SendBaceStatusData.data.chassis_cmd = chassis_cmd;//can_receive.gimbal_receive.game_time;0到中心点，1回家，2补弹

   SendBaceStatusData.crc = get_CRC16_check_sum((uint8_t*)&SendBaceStatusData,send_length-2,0xffff);

    memcpy(Navi_send_data, &SendBaceStatusData, send_length);

    HAL_UART_Transmit(&huart1, Navi_send_data, send_length, 0xFFF);
  
    memset(Navi_send_data, 0, send_length);
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
