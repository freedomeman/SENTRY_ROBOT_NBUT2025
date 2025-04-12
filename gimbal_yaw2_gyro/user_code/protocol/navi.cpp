// #include "navi.h"
// #include "string.h"
// #include "Can_receive.h"
// #include "gimbal.h"
// #include "freertos.h"


// #ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
// extern "C"
// {
// #include "CRC8_CRC16.h"
// }
// #endif

// extern Can_receive  can_receive;
// extern Gimbal gimbal;

// SendGameStatusData_t game_status;
// SendJointState_t joint_status;
// ReceiveNaviState_t navi_status;

// enum
// {
//     SOF_RECEIVE = 0x5A,
//     SOF_SEND = 0x5A,  
//     ID_DEBUG = 0x01,
//     ID_IMU = 0x02,
//     ID_ROBOT_STATE_INFO = 0x03,
//     ID_EVENT_DATA = 0x04,
//     ID_PID_DEBUG = 0x05,
//     ID_ALL_ROBOT_HP = 0x06,
//     ID_GAME_STATUS = 0x07,
//     ID_ROBOT_MOTION = 0x08,
//     ID_GROUND_ROBOT_POSITION = 0x09,
//     ID_RFID_STATUS = 0x0A,
//     ID_ROBOT_STATUS = 0x0B,
//     ID_JOINT_STATE = 0x0C,
//     ID_BUFF = 0x0D,  
//     ID_ROBOT_CMD = 0x01,
//     DEBUG_PACKAGE_NUM = 10,
//     DEBUG_PACKAGE_NAME_LEN = 10,
// }ROBOT_CMD;

// TickType_t time_stamp;
// //TickType_t time_stamp = xTaskGetTickCount();

// uint8_t Navi_send_data[30];

// void Send_Gmae_Status (void)
// {
//     //uint8_t send_length;
//     game_status.HeaderFrame.sof = SOF_RECEIVE;
//     game_status.HeaderFrame.len = 4; //sizeof(game_status.HeaderFrame);
//     game_status.HeaderFrame.id  = ID_GAME_STATUS;
//     game_status.time_stamp = time_stamp;

//     //send_length = sizeof(game_status) - 2;

//     game_status.HeaderFrame.crc = get_CRC8_check_sum((uint8_t*)(&game_status.HeaderFrame),4,0xff);

//     game_status.data.game_progress = can_receive.gimbal_receive.game_progress;
//     game_status.data.stage_remain_time = can_receive.gimbal_receive.game_time;

//     game_status.crc = get_CRC16_check_sum((uint8_t*)&game_status,13,0xffff);

//     memcpy(Navi_send_data, &game_status, 13);

//     HAL_UART_Transmit(&huart1, Navi_send_data, 13, 0xFFF);
  
//     memset(Navi_send_data, 0, 13);


// }

// void Send_Joint_Status (void)
// {
//     //uint8_t send_length;
//     joint_status.HeaderFrame.sof = SOF_RECEIVE;
//     joint_status.HeaderFrame.len = 4;//sizeof(joint_status);
//     joint_status.HeaderFrame.id  = ID_JOINT_STATE;
//     joint_status.time_stamp = time_stamp;
    
//     //send_length = sizeof(joint_status) - 2;

//     joint_status.HeaderFrame.crc = get_CRC8_check_sum((uint8_t*)&joint_status.HeaderFrame,4,0xff);

//     joint_status.data.yaw = gimbal.gimbal_yaw_motor.gyro_angle;
//     joint_status.data.pitch = 0;

//     joint_status.crc = get_CRC16_check_sum((uint8_t*)&joint_status,18,0xffff);

//     memcpy(Navi_send_data, &joint_status, 18);

//     HAL_UART_Transmit(&huart1, Navi_send_data, 18, 0xFFF);

//     memset(Navi_send_data, 0, 18);
// }


// void reveive_navi_status (uint8_t *data)
// {
//      //判断帧头数据是否为0xA5
//     memcpy(&navi_status, data, 43);
// }