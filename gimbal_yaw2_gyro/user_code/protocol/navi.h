// #ifndef __NAVI_H
// #define __NAVI_H

// #include "struct_typedef.h"


// typedef __packed struct 
// {
//   uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
//   uint8_t id;  // 数据段长度
//   uint8_t len;   // 数据段id
//   uint8_t crc;  // 数据帧头的 CRC8 校验
// } HeaderFrame_t;


// //比赛信息数据
// typedef __packed struct 
// {

//   HeaderFrame_t HeaderFrame;
//   uint32_t time_stamp;

//   __packed struct
//   {
//     uint8_t game_progress;
//     uint16_t stage_remain_time;
//   } data;
//   uint16_t crc;

// } SendGameStatusData_t;

// //云台状态数据
// typedef __packed struct 
// {

//   HeaderFrame_t HeaderFrame;
//   uint32_t time_stamp;

//   __packed struct
//   {
//     float pitch;
//     float yaw;
//   } data;
//   uint16_t crc;

// } SendJointState_t;

// typedef __packed struct 
// {
//   uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
//   uint8_t len;  // 数据段长度
//   uint8_t id;   // 数据段id
//   uint8_t crc;  // 数据帧头的 CRC8 校验
// } HeaderFramr_t;


// typedef __packed struct 
// {

//   HeaderFramr_t HeaderFrame;
//   uint32_t time_stamp;

//   __packed struct
//   {
//   __packed struct
//   {
//     float speedx;
//     float speedy;
//     float rollz;
//   } speed_vector;

//   __packed struct
//   {
//     float roll;
//     float pitch;
//     float yaw;
//     float leg_lenth;
//   } chassis;

//   __packed struct
//   {
//     float pitch;
//     float yaw;
//   }  gimbal;

//   __packed struct
//   {
//     uint8_t fire;
//     uint8_t fric_on;
//   }  shoot;

//   __packed struct
//   {
//     bool tracking;
//   }  tracking;
//   }  data;

//   uint16_t checksum;
//   uint16_t crc;

// } ReceiveNaviState_t;
// void Send_Gmae_Status (void);
// void Send_Joint_Status (void);
// void reveive_navi_status (uint8_t *data);
// #endif
