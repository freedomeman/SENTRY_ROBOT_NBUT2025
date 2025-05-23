#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN hcan2
#define BOARD_COM_CAN hcan1

//底盘动力电机编号
enum motive_chassis_motor_id_e
{
  //底盘动力电机接收
  MOTIVE_FR_MOTOR = 0,
  MOTIVE_FL_MOTOR,
  MOTIVE_BL_MOTOR,
  MOTIVE_BR_MOTOR,
};

/* CAN send and receive ID */
typedef enum
{
  //底盘动力电机接收ID  CAN2
  CAN_MOTIVE_FR_MOTOR_ID = 0x201,
  CAN_MOTIVE_FL_MOTOR_ID = 0x202,
  CAN_MOTIVE_BL_MOTOR_ID = 0x203,
  CAN_MOTIVE_BR_MOTOR_ID = 0x204,
  CAN_CHASSIS_MOTIVE_ALL_ID = 0x200,

  //板间通信ID
  CAN_RC_BOARM_COM_ID = 0x101,
  CAN_GIMBAL_BOARD_COM_ID = 0x102,
  CAN_COOLING_BOARM_COM_ID = 0x303 ,
  CAN_17MM_SPEED_BOARD_COM_ID = 0x304,
  CAN_SECISION_BOARD_COM_ID = 0x306,
  CAN_UI_COM_ID = 0x305,

  //超级电容接收ID
  CAN_SUPER_CAP_ID = 0x401,       //接收超电基础数据
  CAN_SUPER_CAP_ID_POWER = 0x402,//用于接收功率数据

} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  uint16_t last_ecd;
} motor_measure_t;

// TODO 超电还未对接
//  //rm motor data
//  typedef struct
//  {
//    fp32 input_vot;
//    fp32 supercap_vot;
//    fp32 input_current;
//    fp32 target_power;
//  } super_cap_measure_t;

//底盘接收数据结构体
typedef struct
{
  //遥控器数据
  int16_t ch_0;
  int16_t ch_2;
  int16_t ch_3;
  uint16_t v;

  //云台状态
  uint8_t s1;
  uint8_t auot_defence;
  uint8_t robo_mode;
  uint8_t gimbal_behaviour;
  fp32 gimbal_yaw_angle;
  fp32 gimbal_pitch_angle;
  int16_t gimbal_pitch_current;

  // UI状态
  bool_t auto_state;
  bool_t aim_state;
  bool_t fric_state;
  bool_t cover_state;
  uint8_t vision_cmdid;
} chassis_receive_t;

//底盘发送数据结构体
typedef struct
{
  //测试热量及ID
  uint16_t id1_17mm_cooling_limit;//17mm测速热量上限
  uint16_t id1_17mm_cooling_rate;//17mm测速热量冷却
  uint16_t id1_17mm_cooling_heat; //17mm测速实时热量
  uint8_t color;               //判断红蓝方
  uint8_t robot_id;            //机器人编号

  //测速速度及底盘模式
  uint16_t id1_17mm_speed_limi; // 17mm测速射速上限
  uint16_t bullet_speed;        // 17mm测速实时射速

  uint8_t chassis_behaviour;

  uint8_t game_progress;

} chassis_send_t;

typedef struct
{
  uint16_t robot_hp;//机器人当前血量
  uint8_t  robot_power_mode;//shooter:1bit；gimbal：2bit；chassis：3bit；1有电，2没电
  uint8_t  robot_by_hit;//机器人受到伤害装甲板受伤
  uint16_t robot_projectile_allowance;//剩余子弹数量

}robot_massega_send_t;

typedef struct
{
  float cap_vot;              //电容电压
  float cap_percentage;       //超电百分比
  float cap_mode;             //超电模式
  float bat_vot;              //电池电压
  float bat_power;            //电池通路功率
  float boot_out_power;       //升压输出功率
} cap_receive_t;

class Can_receive
{
public:
  //动力电机反馈数据结构体
  motor_measure_t chassis_motive_motor[4];

  //发送数据结构体
  CAN_TxHeaderTypeDef chassis_tx_message;
  uint8_t chassis_can_send_data[8];

  //板间通信
  //底盘接收信息
  chassis_receive_t chassis_receive;

  chassis_send_t chassis_send;

  robot_massega_send_t  robot_massega_send;

  //超电数据
  cap_receive_t cap_receive;

  void init();

  //电机数据接收
  void get_motive_motor_measure(uint8_t num, uint8_t data[8]);

  void can_cmd_chassis_motive_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4); //动力电机数据

  void can_cmd_chassis_motive_motor_reset_ID();

  const motor_measure_t *get_chassis_motive_motor_measure_point(uint8_t i);

  //板间通信函数
  void receive_rc_board_com(uint8_t data[8]);

  void receive_gimbal_board_com(uint8_t data[8]);

  void receive_ui_board_com(uint8_t data[8]);


  // 发送枪口热量及ID
  void send_cooling_and_id_board_com(uint16_t id1_17mm_cooling_limit, uint16_t id1_17mm_cooling_rate, uint16_t id1_17mm_cooling_heat, uint8_t color, uint8_t robot_id);
  //发送枪口速度及底盘模式
  void send_17mm_speed_and_mode_board_com(uint16_t id1_17mm_speed_limi, uint16_t bullet_speed, uint8_t chassis_behaviour,uint8_t temp_game_progress);
  //发送决策用的部分信息
  void send_robot_massega_com(uint16_t hp, uint8_t power_mode, uint8_t by_hit, uint16_t projectile_allowance);

  //发送超级电容设定功率
  void can_cmd_super_cap_power(uint16_t set_power,uint16_t buffer_power,uint8_t cap_mode);

  // 获取超电输入电压、电容电压、输入电流、设定功率
  void get_super_cap_data(uint8_t data[8]);
  // 获取超级电容功率
  void get_super_cap_data_power(uint8_t data[8]);
};

#endif
