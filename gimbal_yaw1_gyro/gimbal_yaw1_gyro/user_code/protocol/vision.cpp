#include "vision.h"
#include "Remote_control.h"
#include "struct_typedef.h"
#include "string.h"
#include "INS.h"
#include "tim.h"
#include "CRC8_CRC16.h"
#include "gimbal.h"
#include "Shoot.h"

// #include "referee.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; //视觉数据暂存

extern RC_ctrl_t rc_ctrl;
extern INS imu;
extern Gimbal gimbal;
extern Shoot shoot;

//角度初始化补偿
float Vision_Comps_Yaw = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;           //固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST; //根据距离补偿

VisionSendHeader_t VisionSendHeader; //帧头

VisionActData_t VisionActData; //行动模式结构体

VisionRecvData_t VisionRecvData; //接收数据结构体

VisionSendData_t VisionSendData; //发送数据结构体

uint8_t Attack_Color_Choose = ATTACK_NONE; //默认不识别

//打符是否换装甲了
uint8_t Vision_Armor = FALSE;

//是否识别到装甲板
bool_t if_identify_target = FALSE;

//角度补偿,发送给视觉
float Vision_Comps_Yaw_Send = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;
float SB_K_comps = 3.0f;

//视觉误差判断
uint16_t yaw_para_rv2 = 125;
uint16_t pitch_para_rv2 = 150;
fp32 error_pitch = 0;//-0.05 ;
fp32 error_yaw = 0;//0.01;

extern bool auto_aim_flag;//火控状态,现在默认打开火控
extern Shoot shoot;


//滴答定时器计数变量 ,49天后溢出
volatile uint32_t sysTickUptime=0;

uint8_t error_ture_rv2_limit(float angle , float error);

void vision_init()
{
  usart1_init(Vision_Buffer[0], Vision_Buffer[1], VISION_BUFFER_LEN);

  VisionRecvData.pitch_angle = 0;
  VisionRecvData.yaw_angle = 0;
  VisionRecvData.distance = 0; //距离
  VisionRecvData.centre_lock = 0; //是否瞄准到了中间  0没有  1瞄准到了
  VisionRecvData.identify_target = 0; //视野内是否有目标/是否识别到了目标   0否  1是
  VisionRecvData.identify_buff = 0;   //打符时是否识别到了目标，1是，2识别到切换了装甲，0没识别到
}

/**
  * @brief  读取视觉信息
  * @param  uart1缓存数据
  * @retval void
  * @attention  IRQ执行
  */
uint8_t Vision_Time_Test[2] = {0}; //当前数据和上一次数据
uint8_t Vision_Ping = 0;           //发送时间间隔

void vision_read_data(uint8_t *ReadFormUart)
{

 //判断帧头数据是否为0xA5
 if (ReadFormUart[0] == VISION_BEGIN)
 {
   //判断帧头数据是否为0xff
   if (ReadFormUart[17] == VISION_END)
   {

     //接收数据拷贝
     memcpy(&VisionRecvData, ReadFormUart, VISION_READ_LEN_PACKED);

     if (VisionRecvData.identify_target == TRUE)
       if_identify_target = TRUE; // 识别到装甲板
     else
       if_identify_target = FALSE; // 未识别到装甲板

     // //帧计算
     // Vision_Time_Test[NOW] = xTaskGetTickCount();
     // Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
     // Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
   }
 }
}

/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  *				CmdID   0x00   自瞄击打红色
  *				CmdID   0x01   自瞄击打蓝色
  *				CmdID   0x02   击打红色符
  *				CmdID   0x03   击打蓝色符 
  */
uint8_t vision_send_pack[30] = {0};
uint8_t CmdID = 0;
void vision_send_data(uint8_t CmdID)
{
int i; //循环发送次数
  // uint16_t shoot_speed_limit;
  // uint16_t bullet_speed;
  //get_shooter_shoot_speed_limit_and_bullet_speed(&shoot_speed_limit, &bullet_speed);

  VisionSendData.BEGIN = VISION_BEGIN;

  VisionSendData.CmdID = CmdID;
   VisionSendData.speed = shoot.fric_speed_grade;
  VisionSendData.yaw = imu.INS_angle[0];
  VisionSendData.pitch = imu.INS_angle[1];
  VisionSendData.roll = imu.INS_angle[2];
  VisionSendData.END = 0xFF;

    memcpy(vision_send_pack, &VisionSendData, VISION_SEND_LEN_PACKED);
  //将打包好的数据通过串口移位发送到上位机
 // for (i = 0; i < VISION_SEND_LEN_PACKED; i++)
	// {
	// 	HAL_UART_Transmit(&huart1, &vision_send_pack[i], sizeof(vision_send_pack[0]), 0xFFF);
	// }
  HAL_UART_Transmit(&huart1, vision_send_pack, VISION_SEND_LEN_PACKED, 0xFFF);

  memset(vision_send_pack, 0, 30);
}

//调解自瞄的跟随速度
uint16_t yaw_para = 1;
uint16_t pitch_para = 1;

void vision_error_angle(float *yaw_angle_error, float *pitch_angle_error)
{
   *yaw_angle_error = -VisionRecvData.yaw_angle / yaw_para;
  *pitch_angle_error = -VisionRecvData.pitch_angle / pitch_para;

  if (VisionRecvData.yaw_angle == 0)
  {
    *yaw_angle_error = 0;
  }
  if (VisionRecvData.pitch_angle == 0)
  {
    *pitch_angle_error = 0;
  }
}

/**
  * @brief  判断是否识别到装甲板
  * @param  void
  * @retval TRUE识别到   FALSE未识别到
  * @attention  为自瞄做准备
  */
bool_t vision_if_find_target(void)
{
  return if_identify_target;
}

/**
  * @brief  判断换装甲板了吗
  * @param  void
  * @retval TRUE换了   FALSE没换
  * @attention  为自动打符做准备,串口空闲中断每触发一次且通过校验,则Vision_Armor置TRUE
  */
bool_t vision_if_armor(void)
{
  return Vision_Armor;
}

/**
  * @brief  换装甲标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void vision_clean_ammorflag(void)
{
  Vision_Armor = FALSE;
}

/**
  * @brief 获取us级时间
  * @return us级时间
  * @note 滴答定时器分频在此函数处理好像会损失精度
  */
uint64_t getSysTimeUs(void)
{
	uint64_t ms;
	uint64_t value;
	ms = sysTickUptime;
	value = (ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD) / MPRE;
	return value;
}



/*---------------------------------------------------- rv2 -----------------------------------------------------------*/

#define VISION_READ_LEN_PACKED_PC 16 //接受数据包长度

//extern bool auto_aim_flag;

// 添加计数器变量定义
uint32_t target_detect_count = 0;
uint32_t shoot_detect_count = 0;

VisionSendData_rv2_t VisionSendData_rv2[4]; //君瞄发送结构体

VisionRecvData_rv2_t VisionRecvData_rv2;    //君瞄接收结构体

uint8_t vision_send_pack_rv2[sizeof(VisionSendData_rv2)] = {0};

void vision_send_data_rv2(uint8_t CmdID)
{
   
  for(int i = 0;i < 4;i++)
  {
      VisionSendData_rv2[i].BEGIN = VISION_BEGIN;

      if(can_receive.gimbal_receive.color == 0)
      {
        CmdID = 1;
      }else
      {
        CmdID = 0;
      }
    
      VisionSendData_rv2[i].aim_color = CmdID;

      VisionSendData_rv2[i].roll = imu.INS_angle[2];
      VisionSendData_rv2[i].pitch = -gimbal.gimbal_pitch_motor.gyro_angle;//imu.INS_angle[1];
      VisionSendData_rv2[i].yaw = gimbal.gimbal_yaw_motor.gyro_angle;//imu.INS_angle[0];
      VisionSendData_rv2[i].detect_target = 0x00;
			VisionSendData_rv2[i].END = 0xFF;

    memcpy(vision_send_pack_rv2, &VisionSendData_rv2,sizeof(VisionSendData_rv2));
  }
  HAL_UART_Transmit(&huart1, vision_send_pack_rv2, sizeof(vision_send_pack_rv2), 0xFFF);

}

//瞄准补偿值
//float error_yaw_rv2_user = ERROR_YAW ;
//float error_pitch_rv2_user = ERROR_PITCH ;

// void vision_read_data_rv2(uint8_t *ReadFormUart)
// {

//  //判断帧头数据是否为0xA5
//  if (ReadFormUart[0] == VISION_BEGIN)
//  {
//    //判断帧头数据是否为0xff
//    if (ReadFormUart[15] == VISION_END)
//    {

//      //接收数据拷贝
//      memcpy(&VisionRecvData_rv2, ReadFormUart, VISION_READ_LEN_PACKED_PC);

//     //这个位置的逻辑暂时用不到
//     //  if(VisionRecvData_rv2.fire_or_not == 0x01) // 开火
//     //   {
//     //      tracking_state_rv2 = TRUE;
//     //   }
//     //    else
//     //   {
//     //      tracking_state_rv2 = FALSE;
//     //   }

//       if(VisionRecvData_rv2.yaw == 0 && VisionRecvData_rv2.pitch == 0)
//       {
//         if_identify_target = FALSE;
//       }
//       else
//       {
//         if_identify_target = TRUE;
//       }

//       VisionRecvData_rv2.pitch = -(57.3*VisionRecvData_rv2.pitch - gimbal.gimbal_pitch_motor.gyro_angle);
//       VisionRecvData_rv2.yaw = -(57.3*VisionRecvData_rv2.yaw - gimbal.gimbal_yaw_motor.gyro_angle);

//       // 当识别到目标时计数器加1,否则清零
//       // if(if_identify_target)
//       // {
//       //   target_detect_count++;
//       // }
//       // else
//       // {
//       //   target_detect_count = 0; 
//       // }

//       //自瞄可以开火逻辑，视觉允许开火，并且yaw和pitch都已经瞄准到
//       if (VisionRecvData_rv2.fire_or_not == 0x01 && error_ture_rv2_limit(VisionRecvData_rv2.yaw + error_yaw , ERROR_RV2_ANGLE) && error_ture_rv2_limit(VisionRecvData_rv2.pitch + error_pitch , ERROR_RV2_ANGLE))
//       {
        
        
//         //shoot.shoot_mode = SHOOT_BULLET;
        
        
//         target_detect_count++;//瞄到了，计数值加加，由于视觉的频率太快了，所以暂时，做多次判断
//         /* code */
//       }
//       else
//       {
//         //shoot.shoot_mode = SHOOT_READY ;
//         target_detect_count = 0 ; //丢失目标，重新计数
//       }

//       if (target_detect_count == 1)
//       {
//         //shoot.shoot_mode = SHOOT_BULLET;
//         /* code */
//       }
//       else
//       {
//         //shoot.shoot_mode = SHOOT_STOP;
//       }
      
      

//       //如果视觉发送开火指令，计数值加,如果自瞄发送是不开火指令就停止射击
//       // if(tracking_state_rv2 == TRUE)
//       // {
//       //   //shoot_detect_count = 0;
//       //   target_detect_count++;
//       // }
//       // else
//       // {
//       //   //shoot_detect_count ++
//       //   target_detect_count = 0; 
//       // }
//    }
//  }
// }
void vision_read_data_rv2(uint8_t *ReadFormUart)
{

 //判断帧头数据是否为0xA5
 if (ReadFormUart[0] == VISION_BEGIN)
 {
   //判断帧头数据是否为0xff
   if (ReadFormUart[15] == VISION_END)
   {

     //接收数据拷贝
     memcpy(&VisionRecvData_rv2, ReadFormUart, VISION_READ_LEN_PACKED_PC);

     if(VisionRecvData_rv2.fire_or_not == 0x01) // 开火
      {
         tracking_state_rv2 = TRUE;
      }
       else
      {
         tracking_state_rv2 = FALSE;
      }

      if(VisionRecvData_rv2.yaw == 0 && VisionRecvData_rv2.pitch == 0)
      {
        if_identify_target = FALSE;
      }
      else
      {
        if_identify_target = TRUE;
      }

      VisionRecvData_rv2.pitch = -(57.3*VisionRecvData_rv2.pitch - gimbal.gimbal_pitch_motor.gyro_angle);
      VisionRecvData_rv2.yaw = -(57.3*VisionRecvData_rv2.yaw - gimbal.gimbal_yaw_motor.gyro_angle);

      // 当识别到目标时计数器加1,否则清零
      if(if_identify_target)
      {
        target_detect_count++;
      }
      else
      {
        target_detect_count = 0; 
      }
   }
 }
}

 


uint8_t error_ture_rv2_limit(float angle , float error)
{
  if (error >  angle  > -error)
  {
    return 1;
    /* code */
  }
  return 0 ;
  
}


void vision_error_angle_rv2(float *yaw_angle_error, float *pitch_angle_error)
{
  
  *yaw_angle_error = (VisionRecvData_rv2.yaw + error_yaw )/ yaw_para_rv2;
  *pitch_angle_error = -(VisionRecvData_rv2.pitch + error_pitch) / pitch_para_rv2;

  if (VisionRecvData_rv2.yaw == 0)
  {
    *yaw_angle_error = 0;
  }
  if (VisionRecvData_rv2.pitch == 0)
  {
    *pitch_angle_error = 0;
  }
}
/*----------------------------------------------导航-------------------------------------------------*/
 
 
SendPacketTwist_t SendPacketTwist;  //视觉发送导航数据体
 
#define NAVIGATION_BEGIN 0xA4

void  vision_read_SendPacketTwist(uint8_t *ReadFormUart){
  if(ReadFormUart[0] == NAVIGATION_BEGIN)
  {
    //swapEndian(ReadFormUart,25);
     memcpy(&SendPacketTwist, ReadFormUart,25);
  }
}