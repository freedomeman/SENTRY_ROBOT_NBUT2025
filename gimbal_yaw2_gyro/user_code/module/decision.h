#ifndef __DECISION_H_
#define __DECISION_H_


#include "remote_control.h"

#define LOW_HP  200
#define FALL_HP 400
#define AVOID_TIME  2500

#define MAX_YAW1_ANGLE  0.8f
#define MIN_YAW1_ANGLE  -0.8f

typedef enum
{
    ROBOT_ZERO_FORCE,
    YAW2_CTRL,
    YAW1_CTRL,
    SENTRY_CTRL,
} robot_mode_e;

typedef enum
{
    GOTO_FIGHT, //去中心点战斗
    GOTO_HOME, //回家
    GOTO_GET_SUPPLY, //获取补给
    IS_FIGHTING, //正在战斗
} sentry_behavior_e;

typedef enum
{
    PATROL,
    FIGHT,
    FREE,
    BEHAVIOR_FULL,
} behavior_e;

typedef enum
{
    INIT,
    MID,
    HOME,
    FORTRESS,    
} location_e;

typedef __packed struct 
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} HeaderFramr_t;

typedef __packed struct 
{

  HeaderFramr_t HeaderFrame;
  uint32_t time_stamp;

  __packed struct
  {
  __packed struct
  {
    float speedx;
    float speedy;
    float rollz;
  } speed_vector;

  __packed struct
  {
    float roll;
    float pitch;
    float yaw;
    float leg_lenth;
  } chassis;

  __packed struct
  {
    float pitch;
    float yaw;
  }  gimbal;

  __packed struct
  {
    uint8_t fire;
    uint8_t fric_on;
  }  shoot;

  __packed struct
  {
    bool tracking;
  }  tracking;
  }  data;

  uint16_t checksum;
  uint16_t crc;

} ReceiveNaviState_t;

typedef __packed struct 
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t id;  // 数据段长度
  uint8_t len;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} HeaderFrame_t;

typedef __packed struct 
{

  HeaderFramr_t HeaderFrame;
  uint32_t time_stamp;

  __packed struct
  {
    uint8_t game_progress;
    uint16_t stage_remain_time;
  } data;
  uint16_t crc;

} SendGameStatusData_t;

typedef __packed struct 
{

  HeaderFramr_t HeaderFrame;
  uint32_t time_stamp;

  __packed struct
  {
    uint8_t flag;
    uint8_t chassis_cmd;
  } data;
  uint16_t crc;

} SendBaceStatusData_t;

//云台状态数据
typedef __packed struct 
{

  HeaderFrame_t HeaderFrame;
  uint32_t time_stamp;

  __packed struct
  {
    float pitch;
    float yaw;
  } data;
  uint16_t crc;

} SendJointState_t;

typedef struct 
{
    int16_t x_set;
    int16_t y_set;
    int16_t z_set;
    /* data */
}vset_to_remotset_t;

typedef struct 
{
  float yaw_encoder_angle;
  uint8_t self_aiming_status;
}yaw1_status_t;

class Decision;

class ctrl_buff_c
{
  public:
    //注册自身行为
    behavior_e behavior;
    // 模式控制指针
    //using behavior_set_ctrl = void (Decision::*)(void);
    void (Decision::*ctrl_ptr)(void) ;  // 成员变量
    // 模式切换指针
    //using behavior_set_sw = void (Decision::*)(void);
    void (Decision::*sw_ptr)(void) ;  // 成员变量
};

class Decision
{

public:
    robot_mode_e robot_mode;
    sentry_behavior_e sentry_behavior;
    sentry_behavior_e sentry_behavior_last;

    behavior_e behavior;
    location_e location;
    location_e location_last;

    uint8_t chassis_cmd;

    yaw1_status_t yaw1_status;

    ctrl_buff_c ctrl_buff[BEHAVIOR_FULL];

    const RC_ctrl_t   *remote_control_robot;
    RC_ctrl_t   *remote_control_robot_last;

    RC_ctrl_t   remote_sive;
    RC_ctrl_t   remote_ctrl_yaw2;
    RC_ctrl_t   remote_ctrl_yaw2_last;
    RC_ctrl_t   remote_ctrl_yaw1;
    RC_ctrl_t   remote_ctrl_yaw1_last;

    ReceiveNaviState_t  receivenavistate;
    SendGameStatusData_t  SendGameStatusData;
    SendJointState_t  SendJointState;
    SendBaceStatusData_t  SendBaceStatusData;
    vset_to_remotset_t  vset_to_remotset;

    float add_yaw_to_counterattack , add_yaw_to_follow;
    float yaw2_follow_angle;
    uint16_t avoid_damage_time , using_small_gyroscope , hp_current, hp_last;

    void remote_switch(void);
    void robot_set_mode(void);
    void robot_set_control(void);
    void decision_init(void);
    void sentry_mode_set (void);
    uint8_t find_armi(void);
    void set_mode(void);
    void set_contrl(void);
    void patrol(void);
    void patrol_mode_sw(void);
    void fight(void);

    const RC_ctrl_t* get_yaw2_ctrl_point(void);
    RC_ctrl_t* get_yaw2_ctrl_point_last(void);
    const RC_ctrl_t* get_yaw1_ctrl_point(void);
    RC_ctrl_t* get_yaw1_ctrl_point_last(void);

    void reveive_navi_status (uint8_t *data);
    void Send_Gmae_Status (void);
    void Send_Joint_Status (void);
    void Send_Bace_Status (void);

    void navi_set (void);
    void injury_detection (void);
    uint8_t reach_target (void);
    uint8_t need_attack(void); 
    uint8_t low_blood_is_or_not(void);
    void is_fighting_ctrl(void);
    void sentry_mode_set_control (void);

    void ctrl_buff_register(behavior_e be , void (Decision::*ctrl)() , void (Decision::*sw)());


};


extern Decision decision;




#endif
