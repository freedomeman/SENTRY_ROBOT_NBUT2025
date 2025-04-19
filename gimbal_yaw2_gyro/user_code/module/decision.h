#ifndef __DECISION_H_
#define __DECISION_H_


#include "remote_control.h"

typedef enum
{
    ROBOT_ZERO_FORCE,
    YAW2_CTRL,
    YAW1_CTRL,
    SENTRY_CTRL,
} robot_mode_e;

typedef enum
{
    GOTO_FIGHT,
    GOTO_HOME,
    GOTO_GET_SUPPLY,
} chassis_cmd_e;



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



class Decision
{

public:
    robot_mode_e robot_mode;
    chassis_cmd_e chassis_cmd;

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

    void remote_switch(void);
    void robot_set_mode(void);
    void robot_set_control(void);
    void decision_init(void);

    const RC_ctrl_t* get_yaw2_ctrl_point(void);
    RC_ctrl_t* get_yaw2_ctrl_point_last(void);
    const RC_ctrl_t* get_yaw1_ctrl_point(void);
    RC_ctrl_t* get_yaw1_ctrl_point_last(void);

    void reveive_navi_status (uint8_t *data);
    void Send_Gmae_Status (void);
    void Send_Joint_Status (void);
    void Send_Bace_Status (void);

    void sentry_mode_set (void);
    void navi_set (void);


};


extern Decision decision;




#endif
