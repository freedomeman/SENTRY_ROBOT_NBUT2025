#include "Can_receive.h"
#include "cmsis_os.h"
#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_delay.h"

#ifdef __cplusplus
}
#endif
#include "bsp_can.h"
#include "can.h"

#include "struct_typedef.h"
#include "Communicate.h"
#include "decision.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Communicate communicate;
extern Decision decision;

void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_gimbal_motor_measure(uint8_t num, uint8_t data[8])
{
    //这里我们没有pitch电机，所以直接把yaw的数据转换成9025的解包协议
    // gimbal_motor[num].last_ecd = gimbal_motor[num].ecd;
    // gimbal_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    // gimbal_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    // gimbal_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    // gimbal_motor[num].temperate = data[6];

    gimbal_motor[num].last_ecd = gimbal_motor[num].ecd;
    gimbal_motor[num].ecd = (uint16_t)(data[7] << 8 | data[6]);
    gimbal_motor[num].speed_rpm = (uint16_t)(data[5] << 8 | data[4]);
    gimbal_motor[num].given_current = (uint16_t)(data[3] << 8 | data[2]);
    gimbal_motor[num].temperate = data[1];
}

/**
 * @brief          发送Yaw电机电流
 * @param[in]      yaw yaw轴电流
 */
void Can_receive::can_cmd_yaw_motor(int16_t yaw)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = yaw >> 8;
    can_send_data[1] = yaw;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &can_tx_message, can_send_data, &send_mail_box);
}
/**
 * @brief          发送大Yaw电机电流
 * @param[in]      yaw 大yaw轴电流
 */
void Can_receive::can_cmd_myaw_motor_9025(int16_t myaw)
{
   uint32_t send_mail_box;
   can_tx_message.StdId = CAN_MYAW_ALL_MOTOR_ID;
   can_tx_message.ExtId = 0x00;
   can_tx_message.RTR = CAN_RTR_DATA;
   can_tx_message.IDE = CAN_ID_STD;
   can_tx_message.DLC = 0x08;
   can_send_data[0] = 0;
   can_send_data[1] = 0;
   can_send_data[2] = 0;
   can_send_data[3] = 0;
   can_send_data[4] = myaw;
   can_send_data[5] = myaw >> 8;
   can_send_data[6] = 0;
   can_send_data[7] = 0;

   HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          发送Pitch电机电流
 * @param[in]      pitch pitch轴电流
 */
void Can_receive::can_cmd_pitch_motor(int16_t pitch)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = pitch >> 8;
    can_send_data[3] = pitch;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    //HAL_CAN_AddTxMessage(&GIMBAL_PITCH_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &gimbal_motor[i];
}

void Can_receive::get_shoot_motor_measure(uint8_t num, uint8_t data[8])
{
    shoot_motor[num].last_ecd = shoot_motor[num].ecd;
    shoot_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    shoot_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    shoot_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    shoot_motor[num].temperate = data[6];
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      left_fric: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      right_fric: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      trigger: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      cover: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void Can_receive::can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t cover)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SHOOT_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = left_fric >> 8;
    can_send_data[1] = left_fric;
    can_send_data[2] = right_fric >> 8;
    can_send_data[3] = right_fric;
    can_send_data[4] = trigger >> 8;
    can_send_data[5] = trigger;
    can_send_data[6] = cover >> 8;
    can_send_data[7] = cover;

    //HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void Can_receive::can_cmd_shoot_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x700;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    //HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_shoot_motor_measure_point(uint8_t i)
{
    return &shoot_motor[i];
}

//这里大yaw用不到裁判系统的数据，所以直接发给小yaw
void Can_receive::receive_cooling_and_id_board_com(uint8_t data[8])
{
    gimbal_receive.shoot_cooling_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.shoot_cooling_rate = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.shoot_cooling_heat = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_receive.color = (data[6]);
    gimbal_receive.robot_id = (data[7]);
}
void Can_receive::send_yaw_cooling_and_id_board_com(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_COOLING_BOARM_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = gimbal_receive.shoot_cooling_limit>> 8;
    can_send_data[1] = gimbal_receive.shoot_cooling_limit;
    can_send_data[2] = gimbal_receive.shoot_cooling_rate >> 8;
    can_send_data[3] = gimbal_receive.shoot_cooling_rate;
    can_send_data[4] = gimbal_receive.shoot_cooling_heat >> 8;
    can_send_data[5] = gimbal_receive.shoot_cooling_heat;
    can_send_data[6] = gimbal_receive.color  ;
    can_send_data[7] = gimbal_receive.robot_id;

    HAL_CAN_AddTxMessage(&BOARD_YAW_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::receive_shoot_speed_and_mode_board_com(uint8_t data[8])
{
    gimbal_receive.shoot_speed_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.bullet_speed = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.chassis_behaviour = data[4];
    gimbal_receive.game_progress = data[5];
    gimbal_receive.game_time = (uint16_t)(data[6] << 8 | data[7]);

}

void Can_receive::receive_robot_decision_receive_com(uint8_t data[8])
{
    robot_decision_receive.hp_last = robot_decision_receive.hp;
    robot_decision_receive.hp = (uint16_t)(data[0] << 8 | data[1]);
    robot_decision_receive.power_mode = data[2] ;
    robot_decision_receive.by_hurt = data[3] ;
    robot_decision_receive.allowance = (uint16_t)(data[4] << 8 | data[5]);
}

void Can_receive::send_yaw_shoot_speed_and_mode_board_com(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SHOOT_SPEED_BOARD_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = gimbal_receive.shoot_speed_limit>> 8;
    can_send_data[1] = gimbal_receive.shoot_speed_limit;
    can_send_data[2] = gimbal_receive.bullet_speed >> 8;
    can_send_data[3] = gimbal_receive.bullet_speed;
    can_send_data[4] = gimbal_receive.chassis_behaviour >> 8;
    can_send_data[5] = gimbal_receive.game_progress;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_YAW_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v)
{
    //数据填充
    gimbal_send.ch_0 = ch_0;
    gimbal_send.ch_2 = ch_2;
    gimbal_send.ch_3 = ch_3;
    gimbal_send.v = v;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_RC_BOARM_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = ch_0 >> 8;
    can_send_data[1] = ch_0;
    can_send_data[2] = ch_2 >> 8;
    can_send_data[3] = ch_2;
    can_send_data[4] = ch_3 >> 8;
    can_send_data[5] = ch_3;
    can_send_data[6] = v >> 8;
    can_send_data[7] = v;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_rc_board_yaw_com(int16_t ch_0, int16_t ch_1, int16_t ch_5, int8_t s1 , int8_t s2)
{
    //数据填充
    gimbal_send_yaw.ch_0 = ch_0;
    gimbal_send_yaw.ch_1 = ch_1;
    gimbal_send_yaw.ch_5 = ch_5;
    gimbal_send_yaw.s1   = s1;
    gimbal_send_yaw.s2   = s2;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_RC_BORAM_YAW_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = ch_0 >> 8;
    can_send_data[1] = ch_0;
    can_send_data[2] = ch_1 >> 8;
    can_send_data[3] = ch_1;
    can_send_data[4] = ch_5 >> 8;
    can_send_data[5] = ch_5;
    can_send_data[6] = s1;
    can_send_data[7] = s2;

    HAL_CAN_AddTxMessage(&BOARD_YAW_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_gimbal_board_com(uint8_t s0, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle, uint8_t auot_defence , uint8_t mode)
{
    int32_t temp_gimbal_yaw_angle = (int32_t)(gimbal_yaw_angle * 1000);

    //数据填充
    gimbal_send.s0 = s0;
    gimbal_send.gimbal_behaviour = gimbal_behaviour;
    gimbal_send.gimbal_yaw_angle = gimbal_yaw_angle;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_BOARD_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = s0;
    can_send_data[1] = gimbal_behaviour;
    can_send_data[2] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 24);
    can_send_data[3] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 16);
    can_send_data[4] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 8);
    can_send_data[5] = (uint8_t)((int32_t)temp_gimbal_yaw_angle);
    can_send_data[6] = auot_defence;
    can_send_data[7] = mode ; //pitch;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::send_UI_com(bool_t auto_s, bool_t aim_s, bool_t fric_s, bool_t cover_s, uint8_t vision_cmdid_s, uint16_t v)
{
    //数据填充
    gimbal_send.auto_state = auto_s;
    gimbal_send.aim_state = aim_s;
    gimbal_send.fric_state = fric_s;
    //gimbal_send.cover_state = cover_s;
    gimbal_send.v = v;
    gimbal_send.cover_state = cover_s;
    gimbal_send.vision_cmdid = vision_cmdid_s;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_UI_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = auto_s;
    can_send_data[1] = aim_s;
    can_send_data[2] = fric_s;
    can_send_data[3] = cover_s;
    can_send_data[4] = vision_cmdid_s;
    can_send_data[5] = v >> 8;
    can_send_data[6] = v;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

//导航发送数据
fp32 u_X;
fp32 u_Y;
fp32 u_Z;

void Can_receive::send_SendPacket(SendPacketTwist_t& SendPacketTwist)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_Sendpark;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    u_X=SendPacketTwist.linear_x;
    u_Y=SendPacketTwist.linear_y;
    u_Z=SendPacketTwist.angular_z;

    // 将 float 转换为 uint32_t
    uint32_t u_x = *reinterpret_cast<uint32_t*>(&u_X);
    uint32_t u_y = *reinterpret_cast<uint32_t*>(&u_Y);
    uint32_t u_z = *reinterpret_cast<uint32_t*>(&u_Z);

    // 将转换后的 uint32_t 数据按字节写入 can_send_data 数组
    can_send_data[1] = (uint8_t)(u_x >> 24);
    can_send_data[2] = (uint8_t)(u_x >> 16);
    can_send_data[3] = (uint8_t)(u_y >> 24);
    can_send_data[4] = (uint8_t)(u_y >> 16);

    can_send_data[5] = (uint8_t)(u_z >> 24);
    can_send_data[6] = (uint8_t)(u_z >> 16);
    can_send_data[7] = SendPacketTwist.mode;

    if( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) != 0 )
    {
       HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
    }
}


//在这里增加一个板间通讯用于传输小yaw角度和小yaw模式，以便于大yaw能够帮助小yaw达到跟踪角度
void Can_receive::receive_rc_board_yaw_mode(uint8_t data[8])
{
    decision.yaw1_status.yaw_encoder_angle = (fp32)(int32_t)(data[0]<< 24 | (uint32_t)data[1] << 16 |(uint32_t)data[2] << 8 | (uint32_t)data[3])/ 1000;
    decision.yaw1_status.self_aiming_status = data[4];
}

//chassis_receive.gimbal_yaw_angle = (fp32)(int32_t)(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000;