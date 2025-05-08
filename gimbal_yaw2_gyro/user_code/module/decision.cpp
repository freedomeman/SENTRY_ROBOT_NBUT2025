
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
float zhuangjiafangxiang[4] = {0 , 1.57 , 3.14 , -1.57};

void Decision::decision_init(void)
{
    remote_control_robot = remote_control.get_remote_control_point();
    remote_control_robot_last = remote_control.get_last_remote_control_point();

    robot_mode = ROBOT_ZERO_FORCE;

    vset_to_remotset.x_set = 180;
    vset_to_remotset.y_set = -280;
    vset_to_remotset.z_set = -280;

    using_small_gyroscope = 4;
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

        location =INIT;
        behavior =FREE;
        /* code */
    }
    else if (robot_mode == SENTRY_CTRL)
    {
        // sentry_mode_set();
         injury_detection(); //这里会检测是否受到伤害，并开启小陀螺
        // sentry_mode_set_control();
        // if (sentry_behavior != IS_FIGHTING) //这里是为了屏蔽在去中心点时遇到敌人可以做击打，但不会设置到回家补血
        // {
        //     navi_set();
        //     /* code */
        // }
        set_mode();
        set_contrl();
        navi_set();
        remote_ctrl_yaw1.rc.s[0] = 1;
        /* code */
    }
    
}

uint8_t Decision::find_armi(void)
{
   return yaw1_status.self_aiming_status;
}

uint8_t mode_sw=0;
void Decision::set_mode(void)
{
    if (can_receive.robot_decision_receive.hp>=150 && mode_sw == 0 )
	{
		location = MID;
		if (reach_target()==1)
		{
			if (find_armi() == 0)
			{
				behavior = PATROL;
			}
			
			if (find_armi() == 1)
			{
				behavior = FIGHT;
			}
		}
		else
		{
			behavior = FREE;
		}
	}
	else if (can_receive.robot_decision_receive.hp<=100)
	{
		location = HOME;
		behavior = FREE;
        mode_sw = 1;
		
	}
    if (mode_sw == 1)
    {
        if (can_receive.robot_decision_receive.hp>300)
        {
            mode_sw = 0;
            /* code */
        }
        /* code */
    }
    

    
    
}

void Decision::set_contrl(void)
{
    if (location == HOME)
    {
        chassis_cmd = 1;
        /* code */
    }
    if (location == MID)
    {
        chassis_cmd = 0;
        /* code */
    }
    if (behavior == FREE)
    {
        /* code */
    }
    if (behavior == FIGHT)
    {
        fight();
        /* code */
    }
    if (behavior == PATROL)
    {
        patrol();
        /* code */
    }
}

float angle , add_angle , add_yaw , patrol_follow_rpm;
void Decision::patrol(void)
{
    static int diraction=0;
    
    if (can_receive.robot_decision_receive.hp != can_receive.robot_decision_receive.hp_last)//受击检测
    {
        diraction = can_receive.robot_decision_receive.by_hurt;
        angle = gimbal.gimbal_yaw_motor.encode_angle;
        add_angle = zhuangjiafangxiang[diraction] - angle;
        /*---计算一个最小的追踪方向*/
        if (add_angle > 0)
        {
            if ((6.28 - add_angle) < 3.14) 
            {
                add_angle = - (6.28 - add_angle);
                /* code */
            }
            
            /* code */
        }
        else if (add_angle < 0)
        {
            if ((6.28 + add_angle) < 3.14 )
            {
                add_angle =  (6.28 + add_angle);
                /* code */
            }
            
            /* code */
        }
        /* code */
    }
    if ( -0.2 < add_angle && add_angle < 0.2)
    {
        add_yaw = 0;
		add_angle = 0;
        patrol_follow_rpm = 0.005;
        /* code */
    }
    else
    {
        patrol_follow_rpm = 0;
        if (add_angle > 0)
        {
            add_yaw = 0.005 ;
            add_angle -= 0.005;
            /* code */
        }
        else
        {
            add_yaw = -0.005 ;
            add_angle += 0.005;
        }
    }
    
    //remote_ctrl_yaw2.rc.ch[0] = 180;
}

void Decision::fight(void)
{
    if (yaw1_status.yaw_encoder_angle >= MAX_YAW1_ANGLE)
    {
        yaw2_follow_angle = 0.6;
        /* code */
    }
    if (yaw1_status.yaw_encoder_angle <= MIN_YAW1_ANGLE)
    {
        yaw2_follow_angle = -0.6;
        /* code */
    }
    if (yaw2_follow_angle > 0.2)
    {
        remote_ctrl_yaw2.rc.ch[0] = 90;
        yaw2_follow_angle -= 90*0.01;
        /* code */
    }
    else if (yaw2_follow_angle < -0.2)
    {
        remote_ctrl_yaw2.rc.ch[0] = -90;
        yaw2_follow_angle += 90*0.01;
        /* code */
    }
    else if ( -0.2 < yaw2_follow_angle < 0.2)
    {
        remote_ctrl_yaw2.rc.ch[0] = 0;
        /* code */
    }
}


//uint8_t hurt_direction=0;
void Decision::sentry_mode_set (void)
{
    sentry_behavior = GOTO_FIGHT;
    if (sentry_behavior != IS_FIGHTING  )
    {
        if (need_attack() == 1)
        {
            sentry_behavior_last = sentry_behavior; //行为切换保存
            sentry_behavior = IS_FIGHTING;
            /* code */
        }
        /* code */
    }
    else if (sentry_behavior == IS_FIGHTING)
    {
        if (need_attack() == 0) //小yaw未识别到目标
        {
            sentry_behavior = sentry_behavior_last; //丢失目标后，恢复行为
            /* code */
        }
        /* code */
    }

    if (low_blood_is_or_not() == 1) //血量过低需要回家补血
    {
        sentry_behavior = GOTO_HOME;
        /* code */
    }
    if (sentry_behavior == GOTO_HOME)
    {
        if (can_receive.robot_decision_receive.hp == FALL_HP)//回家补满血量
        {
            sentry_behavior = GOTO_FIGHT;
            /* code */
        }
        
        /* code */
    }

}

uint8_t Decision::low_blood_is_or_not(void)
{
    if (can_receive.robot_decision_receive.hp <= LOW_HP) //小于安全血量
    {
        return 1;
        /* code */
    }
    return 0;
    
}

uint8_t Decision::need_attack(void) //这里判断是否需要攻击
{
    if (yaw1_status.self_aiming_status == 1) //小头识别到敌人
    {
        return 1;
        /* code */
    }
    return 0;
    
}
        
uint8_t Decision::reach_target (void)
{
    if (receivenavistate.data.speed_vector.speedx == 0 && receivenavistate.data.speed_vector.speedy == 0)//说明此时已经导航到目标点
    {
        return 1;
        /* code */
    }
    return 0 ;

    
}

void Decision::sentry_mode_set_control (void)
{
    if (sentry_behavior == GOTO_HOME)
    {
        chassis_cmd = 1;
        /* code */
    }
    if (sentry_behavior == GOTO_FIGHT) //这里后续增加根据时间判断是否去堡垒还是中心点
    {
        chassis_cmd = 0;
        /* code */
    }
    if (sentry_behavior == IS_FIGHTING)
    {
        is_fighting_ctrl();
        /* code */
    }
    
    
    
}

float yaw2_follow_angle=0;
void Decision::is_fighting_ctrl(void)
{
    remote_ctrl_yaw2.rc.ch[2] = 0;
    remote_ctrl_yaw2.rc.ch[3] = 0;
    if (yaw1_status.yaw_encoder_angle >= MAX_YAW1_ANGLE)
    {
        yaw2_follow_angle = 0.6;
        /* code */
    }
    if (yaw1_status.yaw_encoder_angle <= MIN_YAW1_ANGLE)
    {
        yaw2_follow_angle = -0.6;
        /* code */
    }
    if (yaw2_follow_angle > 0.2)
    {
        remote_ctrl_yaw2.rc.ch[1] = 90;
        yaw2_follow_angle -= 90*0.01;
        /* code */
    }
    else if (yaw2_follow_angle < -0.2)
    {
        remote_ctrl_yaw2.rc.ch[1] = -90;
        yaw2_follow_angle += 90*0.01;
        /* code */
    }
    else if ( -0.2 < yaw2_follow_angle < 0.2)
    {
        remote_ctrl_yaw2.rc.ch[1] = 0;
        /* code */
    }
    
    
    
}

void Decision::injury_detection (void)
{
    if (hp_current != can_receive.robot_decision_receive.hp)
    {
        hp_last = hp_current;
        hp_current = can_receive.robot_decision_receive.hp;
        /* code */
    }
    
    if ( (can_receive.robot_decision_receive.hp_last - can_receive.robot_decision_receive.hp) >= 2 )//此时说明受到伤害
    {
        //hp_last = hp_current;
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
		//can_receive.robot_decision_receive.by_hurt = 0;
        /* code */
    }
    else if(avoid_damage_time > 0)//没有受伤
    {
        avoid_damage_time --;
        if (avoid_damage_time == 1) //一定时间没有受伤
        {
            using_small_gyroscope = 3; //请求关闭小陀螺
            /* code */
        }
        
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
    remote_ctrl_yaw1.rc.s[0] = 1;
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
