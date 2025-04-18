#ifndef UI_H
#define UI_H



#include "struct_typedef.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stdarg.h"
#include "usart.h"
#include "Can_receive.h"

#pragma pack(1) //按1字节对齐

#define NULL 0
#define __FALSE 100

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0      //直线
#define UI_Graph_Rectangle 1 //矩形
#define UI_Graph_Circle 2    //整圆
#define UI_Graph_Ellipse 3   //椭圆
#define UI_Graph_Arc 4       //圆弧
#define UI_Graph_Float 5     //浮点型
#define UI_Graph_Int 6       //整形
#define UI_Graph_Char 7      //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0 //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6 //青色
#define UI_Color_Black 7
#define UI_Color_White 8

typedef unsigned char Uint8_t;
typedef unsigned char uint8_t;

typedef struct
{
    uint8_t SOF;     //起始字节,固定0xA5
    uint16_t Data_Length; //帧数据长度
    uint8_t Seq;     //包序号
    uint8_t CRC8;    //CRC8校验值
    uint16_t CMD_ID; //命令ID
} UI_Packhead;       //帧头

typedef struct
{
    uint16_t Data_ID; //内容ID
    uint16_t Sender_ID; //发送者ID
    uint16_t Receiver_ID; //接收者ID
} UI_Data_Operate;   //操作定义帧

typedef struct
{
    uint8_t Delete_Operate; //删除操作
    uint8_t Layer;          //删除图层
} UI_Data_Delete;      //删除图层帧

typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    float graph_Float; //浮点数据
} Float_Data;

typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11; //图形数据
} Graph_Data;

typedef struct
{
    Graph_Data Graph_Control;
    uint8_t show_Data[30];
} String_Data; //打印字符串数据

extern UART_HandleTypeDef huart6;

class Ui
{
public:
    unsigned char UI_Seq; //包序号

    uint8_t *Robot_ID;   //当前机器人的ID
    uint16_t *Cilent_ID; //发送者机器人对应的客户端ID
    uint8_t sent_cnt;
    uint8_t static_sent_end;


    Graph_Data G1, G2, G3, G4, G5, G6, G7; //瞄准线
    Graph_Data G_SHOOT;                    //摩擦轮状态
    Graph_Data G_TOP;                      //小陀螺状态
    Graph_Data G_RECOVER;                  //无敌状态
    Graph_Data G_AUTO_READY;               //自瞄准备状态
    Graph_Data G_AUTO_AIM;                 //自瞄识别状态
    Graph_Data G_SUPER_CAP;                //超电开关
    Graph_Data G_COVER;                    //弹仓开关
    Float_Data G_PITCH;                    // Pitch轴角度
    Float_Data G_YAW;                      // Yaw轴角度
    Float_Data G_SUPER_NUM;                //超电百分比
    Graph_Data G_CAP;                      //超电
    Graph_Data G_CAP_BORD;                //超电
    char shoot_arr[5];//摩擦轮
    char rotate_arr[6];  //小陀螺
    char super_arr[9];//超电
    char cover_arr[6];//弹仓
    char auto_arr[4];  
    char mag_arr[7] ;
    char vision_arr[6];
    char blue_arr[4];
    char red_arr[3];
    char board_arr[5];
    char symbol_arr[6];
    char gimbal_arr[6];
    char chassis_arr[6];
    char relax_arr[6];
    String_Data CH_SHOOT;
    String_Data CH_ROTATE;
    String_Data CH_SUPER_CAP;
    String_Data CH_MODE;
    String_Data CH_AUTO_READY;
    String_Data CH_MAG;
    String_Data CH_COVER;
    String_Data CH_VISION;
    String_Data CH_COLOR;
    String_Data CH_MODES;

    void init(uint8_t *Temp_Judge_Self_ID, uint16_t *Temp_Judge_SelfClient_ID);

    void run();
		
/*----------------------------UI功能函数----------------------------------------*/
		void feedback_update();
		void draw_super_cap(); //根据超电容量得到UI数据
		void start();//重置ui

    //基础图形绘制函数
    void UI_SendByte(unsigned char ch);

    void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer);
    void Line_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    int UI_ReFresh(int cnt, ...);
    unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
    uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
    void Circle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
    void Rectangle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    void Float_Draw(Float_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float);
    void Char_Draw(String_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char *Char_Data);
    int Char_ReFresh(String_Data string_Data);
    void Arc_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
};


#endif
#pragma pack() //按1字节对齐

