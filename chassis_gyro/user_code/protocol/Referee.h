#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "fifo.h"

#ifdef __cplusplus
}
#endif

#define USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH 1024

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE 128

#define REF_PROTOCOL_HEADER_SIZE sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE 2
#define REF_PROTOCOL_CRC16_SIZE 2
#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID = 0x0001,
    GAME_RESULT_CMD_ID = 0x0002,
    GAME_ROBOT_HP_CMD_ID = 0x0003,
    FIELD_EVENTS_CMD_ID = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID = 0x0103,
    REFEREE_WARNING_CMD_ID = 0x0104,
    DART_REMAINING_TIME_CMD_ID = 0x0105,
    ROBOT_STATE_CMD_ID = 0x0201,
    POWER_HEAT_DATA_CMD_ID = 0x0202,
    ROBOT_POS_CMD_ID = 0x0203,
    BUFF_MUSK_CMD_ID = 0x0204,
    //AERIAL_ROBOT_ENERGY_CMD_ID = 0x0205,
    AIR_SUPPORT_DATA_ID = 0x0205,
    ROBOT_HURT_CMD_ID = 0x0206,
    SHOOT_DATA_CMD_ID = 0x0207,
    BULLET_REMAINING_CMD_ID = 0x0208,
    RFID_STATUS_CMD_ID = 0x0209,

    DART_CLIENT_CMD_ID = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID = 0x020B,
    RADAR_MARK_DATA_CMD_ID = 0x020C,
    SENTRY_INFO_CMD_ID = 0x020D,
    RADAR_INFO_CMD_ID = 0x020E,

    STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301,
    IDCustomData,
} referee_cmd_id_t;
typedef struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct
{
    frame_header_struct_t *p_header;
    uint16_t data_len;
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t index;
} unpack_data_t;

#pragma pack(pop)

//***********************数据包结构体*******************//

#define RED 0
#define BLUE 1

typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    RED_RADAR = 9,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
    BLUE_RADAR = 109,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;
typedef __packed struct // 0x0001
{
    /*game_type:
     *1：RoboMaster 机甲大师赛
     *2：RoboMaster 机甲大师单项赛
     *3：ICRA RoboMaster 人工智能挑战赛
     *4：RoboMaster 联盟赛 3V3
     *5：RoboMaster 联盟赛 1V1
     */
    uint8_t game_type : 4;
    /*game_progress:
     *0：未开始比赛
     *1：准备阶段
     *2：自检阶段
     *3：5s 倒计时
     *4：对战中
     *5：比赛结算中
     */
    uint8_t game_progress : 4;
    /*stage_remain_time:
     *当前阶段剩余时间*/
    uint16_t stage_remain_time;
    /*SyncTimeStamp:
     * 机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效*/
    uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef __packed struct // 0x0002
{
    /*winner:
     * 0 平局 1 红方胜利 2 蓝方胜利
     */
    uint8_t winner;
} ext_game_result_t;
typedef __packed struct // 0x0003
{
    /*red_x_robot_HP:
     *红方x号机器人血量
     *1:英雄
     *2:工程
     *3:3号步兵
     *4:4号步兵
     *5:5号步兵
     *7:哨兵*/
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    /*red_outpost_HP:
     *红方前哨站血量*/
    uint16_t red_outpost_HP;
    /*red_base_HP:
     *红方基地血量*/
    uint16_t red_base_HP;

    /*blue_x_robot_HP:
     *蓝方x号机器人血量
     *1:英雄
     *2:工程
     *3:3号步兵
     *4:4号步兵
     *5:5号步兵
     *7:哨兵*/
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    /*blue_outpost_HP:
     *蓝方前哨站血量*/
    uint16_t blue_outpost_HP;
    /*blue_base_HP:
     *蓝方基地血量*/
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;
typedef __packed struct // 0x0101
{
    /*event_type:
    0：未占领/未激活
    1：已占领/已激活
    bit 0-2：
    bit 0：己方补给站前补血点的占领状态，1 为已占领
    bit 1：己方补给站内部补血点的占领状态，1 为已占领
    bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
    bit 3-5：己方能量机关状态
    bit 3：己方能量机关激活点的占领状态，1 为已占领
    bit 4：己方小能量机关的激活状态，1 为已激活
    bit 5：己方大能量机关的激活状态，1 为已激活
    bit 6-11：己方高地占领状态
    bit 6-7：己方环形高地的占领状态，1 为被己方占领，2 为被对方占领
    bit 8-9：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
    bit 10-11：己方梯形高地的占领状态，1 为被己方占领，2 为被对方占
            领
    bit 12-18：己方基地虚拟护盾的剩余值百分比（四舍五入，保留整数）
    bit 19-27：飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为
             0）
    bit 28-29：飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，
            1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机目标
    bit 30-31：中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被
            对方占领，3 为被双方占领。（仅 RMUL 适用）*/
    uint32_t event_data;
} ext_event_data_t;

typedef __packed struct // 0x0102
{
    /*保留位
     */
    uint8_t reserved;

    /*supply_robot_id:
     *补弹机器人ID：
     * 0 为当前无机器人补弹
     * 1 为红方英雄机器人补弹
     * 2 为红方工程机器人补弹
     * 3/4/5 为红方步兵机器人补弹
     * 101 为蓝方英雄机器人补弹
     * 102 为蓝方工程机器人补弹
     * 103/104/105 为蓝方步兵机器人补弹
     */
    uint8_t supply_robot_id;
    /*supply_projectile_step:
     * 出弹口开闭状态：
     * 0 为关闭，1 为子弹准备中，2 为子弹下落
     */
    uint8_t supply_projectile_step;
    /*supply_projectile_num:
     *补弹数量：
     * 50：50 颗子弹
     * 100：100 颗子弹
     * 150：150 颗子弹
     * 200：200 颗子弹
     */
    uint8_t supply_projectile_num;

} ext_supply_projectile_action_t;

/*
    被删除
typedef __packed struct // 0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;
*/
typedef __packed struct //0x104
{
    /*level:
     *警告等级：
     * 1：黄牌
     * 2：红牌
     * 3：判负
     */
    uint8_t level;
    /*foul_robot_id:
     * 犯规机器人 ID：
     * 判负时，机器人 ID 为 0
     * 黄牌、红牌时，机器人 ID 为犯规机器人 ID
     */
    uint8_t foul_robot_id;
    /*count
     *己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认
     *为 0。）
     */
    uint8_t count;
} ext_referee_warning_t;
typedef __packed struct // 0x0105
{
    /*dart_remaining_time:
    飞镖发射口倒计时:15s 倒计时*/
    uint8_t dart_remaining_time;
    /*bit 0-1：
     *最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中
     *基地固定目标，3 为击中基地随机目标
     *bit 2-4：
     *对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4
     *bit 5-6：
     *字节偏移量 大小 说明
     *飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基
     *地固定目标为 1，选中基地随机目标为 2
     *bit 7-15：保留*/
    uint16_t dart_info;
    
} ext_dart_remaining_time_t;


// typedef __packed struct // 0x0201
// {
//     /*robot_id:
//      *本机器人 ID：
//      *1：红方英雄机器人
//      *2：红方工程机器人
//      *3/4/5：红方步兵机器人
//      *6：红方空中机器人
//      *7：红方哨兵机器人
//      *8：红方飞镖机器人
//      *9：红方雷达站
//      *101：蓝方英雄机器人
//      *102：蓝方工程机器人
//      *103/104/105：蓝方步兵机器人
//      *106：蓝方空中机器人
//      *107：蓝方哨兵机器人
//      *108：蓝方飞镖机器人
//      *109：蓝方雷达站。
//      */
//     uint8_t robot_id;
//     /*robot_level:
//      *机器人等级：
//      *1：一级
//      *2：二级
//      *3：三级
//      */
//     uint8_t robot_level;
//     /*remain_HP:
//      *机器人剩余血量*/
//     uint16_t remain_HP;
//     /*max_HP:
//      * 机器人上限血量*/
//     uint16_t max_HP;

//     /*shooter_id1_17mm_cooling_rate:
//      * 机器人 1 号 17mm 枪口每秒冷却值*/
//     uint16_t shooter_id1_17mm_cooling_rate;
//     /*shooter_id1_17mm_cooling_limit:
//      * 机器人 1 号 17mm 枪口热量上限*/
//     uint16_t shooter_id1_17mm_cooling_limit;
//     /*shooter_id1_17mm_speed_limit:
//      * 2 机器人 1 号 17mm 枪口上限速度 单位 m/s*/
//     uint16_t shooter_id1_17mm_speed_limit;

//     /*shooter_id2_17mm_cooling_rate:
//      * 机器人 2 号 17mm 枪口每秒冷却值*/
//     uint16_t shooter_id2_17mm_cooling_rate;
//     /*shooter_id2_17mm_cooling_limit:
//      * 机器人 2 号 17mm 枪口热量上限*/
//     uint16_t shooter_id2_17mm_cooling_limit;
//     /*shooter_id2_17mm_speed_limit:
//      * 机器人 2 号 17mm 枪口上限速度 单位 m/s*/
//     uint16_t shooter_id2_17mm_speed_limit;

//     /*shooter_id1_42mm_cooling_rate:
//      * 机器人 42mm 枪口每秒冷却值*/
//     uint16_t shooter_id1_42mm_cooling_rate;
//     /*shooter_id1_42mm_cooling_limit:
//      * 机器人 42mm 枪口热量上限*/
//     uint16_t shooter_id1_42mm_cooling_limit;
//     /*shooter_id1_42mm_speed_limit:
//      * 机器人 42mm 枪口上限速度 单位 m/s*/
//     uint16_t shooter_id1_42mm_speed_limit;

//     /*chassis_power_limit
//      * 机器人底盘功率限制上限*/
//     uint16_t chassis_power_limit;
//     /*主控电源输出情况：
//         0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出
//         1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出
//         2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出
//      */
//     uint8_t mains_power_gimbal_output : 1;
//     uint8_t mains_power_chassis_output : 1;
//     uint8_t mains_power_shooter_output : 1;
 
// } ext_game_robot_state_t; 
 

typedef __packed struct // 0x0201
{
     /*robot_id:
     *本机器人 ID：
     *1：红方英雄机器人
     *2：红方工程机器人
     *3/4/5：红方步兵机器人
     *6：红方空中机器人
     *7：红方哨兵机器人
     *8：红方飞镖机器人
     *9：红方雷达站
     *101：蓝方英雄机器人
     *102：蓝方工程机器人
     *103/104/105：蓝方步兵机器人
     *106：蓝方空中机器人
     *107：蓝方哨兵机器人
     *108：蓝方飞镖机器人
     *109：蓝方雷达站。
     */
    uint8_t robot_id;
     /*robot_level:
     *机器人等级：
     *1：一级
     *2：二级
     *3：三级
     */
    uint8_t robot_level;
    /*current_HP:
     *机器人当前血量*/
    uint16_t current_HP; 
    /*maximum_HP:
     * 机器人上限血量*/
    uint16_t maximum_HP;
    /*shooter_barrel_cooling_value:
     * 机器人枪口热量每秒冷却值*/
    uint16_t shooter_barrel_cooling_value;
    /*shooter_barrel_heat_limit:
     * 机器人枪口热量上限*/
    uint16_t shooter_barrel_heat_limit;
     /*chassis_power_limit
     * 机器人底盘功率限制上限*/
    uint16_t chassis_power_limit; 
     /*主控电源输出情况：
        0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出
        1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出
        2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出
     */
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1; 
    uint8_t power_management_shooter_output : 1;
}ext_game_robot_state_t;

typedef __packed struct // 0x0202
{
    /*chassis_volt:
     * 底盘输出电压 单位 毫伏*/
    uint16_t chassis_volt;
    /*chassis_current:
     * 底盘输出电流 单位 毫安*/
    uint16_t chassis_current;
    /*chassis_power:
     * 底盘输出功率 单位 W 瓦*/
    float chassis_power;
    /*chassis_power_buffer:
     * 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J*/
    uint16_t chassis_power_buffer;
    /*shooter_id1_17mm_cooling_heat
     * 1 号 17mm 枪口热量*/
    uint16_t shooter_id1_17mm_cooling_heat;
    /*shooter_id2_17mm_cooling_heat
     *2 号 17mm 枪口热量*/
    uint16_t shooter_id2_17mm_cooling_heat;
    /*shooter_id1_42mm_cooling_heat
     * 42mm 枪口热量*/
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct // 0x0203
{
    /*x:
     * 位置 x 坐标，单位 m*/
    float x;
    /*y:
     * 位置 y 坐标，单位 m*/
    float y;
    /*z:
     * 位置 z 坐标，单位 m*/

    //float z;
    /*yaw
     * 位置枪口，单位度*/
    //float yaw;
    /*angle

     *本机器人测速模块的朝向，单位：度。正北为 0 度*/
    float angle;

} ext_game_robot_pos_t;

// typedef __packed struct // 0x0204
// {
//     /*power_rune_buff:
//      *机器人增益：
//      * bit 0：机器人血量补血状态
//      * bit 1：枪口热量冷却加速
//      * bit 2：机器人防御加成
//      * bit 3：机器人攻击加成
//      * 其他 bit 保留
//      */
//     uint8_t power_rune_buff;
// } ext_buff_musk_t;

typedef __packed struct // 0x0204
{
    /*power_rune_buff:
     *机器人增益：
     * bit 0：机器人血量补血状态
     * bit 1：枪口热量冷却加速
     * bit 2：机器人防御加成
     * bit 3：机器人攻击加成
     * 其他 bit 保留
     */
    /*recovery_buff
    机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）*/
    uint8_t recovery_buff;
    /* cooling_buff
    机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）*/
    uint8_t cooling_buff;
    /* defence_buff
    机器人防御增益（百分比，值为 50 表示 50%防御增益）*/
    uint8_t defence_buff;
    /* vulnerability_buff
    机器人负防御增益（百分比，值为 30 表示-30%防御增益）*/
    uint8_t vulnerability_buff;
    /* attack_buff
    机器人攻击增益（百分比，值为 50 表示 50%攻击增益）*/
    uint16_t attack_buff;
}ext_buff_musk_t;

//  typedef __packed struct // 0x0205
//  {
//      /*attack_time:
//       * 可攻击时间 单位 s。30s 递减至 0*/
//      uint8_t attack_time;
//  } aerial_robot_energy_t;

typedef  __packed struct // 0x0205
{
    /*airforce_status:
     *空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）*/
    uint8_t airforce_status;
    /*airforce_status:
     *此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此
     *值为 1）
     *若冷却时间为 0，但未呼叫空中支援，则该值为 0*/
     uint8_t time_remain;
}ext_air_support_data_t;

typedef __packed struct // 0x0206
{
    /*armor_type:
     * bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。
     * hurt_type:
     * bit 4-7：血量变化类型
     * 0x0 装甲伤害扣血
     * 0x1 模块掉线扣血
     * 0x2 超射速扣血
     * 0x3 超枪口热量扣血
     * 0x4 超底盘功率扣血
     * 0x5 装甲撞击扣血
     */
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct // 0x0207
{
    /*bullet_type:
     *子弹类型:
     *1：17mm 弹丸 2：42mm 弹丸
     */
    uint8_t bullet_type;
    /*shooter_id:
     *发射机构 ID：
     * 1：1 号 17mm 发射机构
     * 2：2 号 17mm 发射机构
     * 3：42mm 发射机构
     */
    uint8_t shooter_id;
    /*bullet_freq:
     *子弹射频 单位 Hz*/
    uint8_t bullet_freq;
    /*bullet_speed:
     *子弹射速 单位 m/s*/
    float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct //0x0208
{
    /*bullet_remaining_num_17mm
     *17mm 子弹剩余发射数目*/
    uint16_t bullet_remaining_num_17mm;
    /*bullet_remaining_num_42mm
     *42mm 子弹剩余发射数目*/
    uint16_t bullet_remaining_num_42mm;
    /*coin_remaining_num
     *剩余金币数量*/
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct //0x0208
{
    /*bullet_remaining_num_17mm
     *17mm 子弹剩余发射数目*/
    uint16_t projectile_allowance_17mm;
    /*bullet_remaining_num_42mm
     *42mm 子弹剩余发射数目*/
    uint16_t projectile_allowance_42mm;
    /*coin_remaining_num
     *剩余金币数量*/
    uint16_t remaining_gold_coin;
}ext_projectile_allowance_t;

typedef __packed struct //0x0209
{
    /*rfid_status:
     *机器人 RFID 状态:
     * bit 0：基地增益点 RFID 状态
     * bit 1：高地增益点 RFID 状态
     * bit 2：能量机关激活点 RFID 状态
     * bit 3：飞坡增益点 RFID 状态
     * bit 4：前哨岗增益点 RFID 状态
     * bit 6：补血点增益点 RFID 状态
     * bit 7：工程机器人复活卡 RFID 状态
     * bit 8：己方飞坡增益点（靠近己方一侧飞坡前）
     * bit 9：己方飞坡增益点（靠近己方一侧飞坡后）
     * bit 10：对方飞坡增益点（靠近对方一侧飞坡前）
     * bit 11：对方飞坡增益点（靠近对方一侧飞坡后）
     * bit 12：己方前哨站增益点
     * bit 13：己方补血点（检测到任一均视为激活）
     * bit 14：己方哨兵巡逻区
     * bit 15：对方哨兵巡逻区
     * bit 16：己方大资源岛增益点
     * bit 17：对方大资源岛增益点
     * bit 18：己方兑换区
     * bit 19：中心增益点（仅 RMUL 适用）
     * bit 20-31：保留
     * */
    uint32_t rfid_status;
} ext_rfid_status_t;

// typedef __packed struct // 0x020A
// {
//     /*dart_launch_opening_status
//      * 当前飞镖发射口的状态:
//      * 1：关闭
//      * 2：正在开启或者关闭中
//      * 0：已经开启
//      * */
//     uint8_t dart_launch_opening_status;
//     /*dart_attack_target:
//      * 飞镖的打击目标，默认为前哨站
//      * 0：前哨站
//      * 1：基地
//      * */
//     uint8_t dart_attack_target;
//     /*target_change_time:
//      * 切换打击目标时的比赛剩余时间，单位秒，从未切换默认为0*/
//     uint16_t target_change_time;
//     /*operate_launch_cmd_time:
//      * 最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0*/
//     uint16_t operate_launch_cmd_time;
// } ext_dart_client_cmd_t;

typedef __packed struct // 0x020A
{
    /*dart_launch_opening_status
     * 当前飞镖发射口的状态:
     * 1：关闭
     * 2：正在开启或者关闭中
     * 0：已经开启
     * */
    uint8_t dart_launch_opening_status;
    //保留位
    uint8_t reserved;
    /*target_change_time:
     * 切换打击目标时的比赛剩余时间，单位秒，从未切换默认为0*/
    uint16_t target_change_time;
    /*operate_launch_cmd_time:
     * 最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0*/
    uint16_t latest_launch_cmd_time;
}ext_dart_client_cmd_t;

/*场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边
向红方停机坪为 Y 轴正方向,x,y为坐标*/

typedef __packed struct //0x020B
{
 float hero_x; 
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float standard_5_x;
 float standard_5_y;
}ext_ground_robot_position_t;

/*对方的机器人被标记的进度*/

typedef __packed struct //x020C
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}ext_radar_mark_data_t;

typedef __packed struct //0x020D
{
    /*sentry_status:
     *bit 0-10：除远程兑换外，哨兵成功兑换的发弹量，开局为 0，在哨兵成功兑
     *换一定发弹量后，该值将变为哨兵成功兑换的发弹量值。
     *bit 11-14：哨兵成功远程兑换发弹量的次数，开局为 0，在哨兵成功远程兑   
     *换发弹量后，该值将变为哨兵成功远程兑换发弹量的次数。
     *bit 15-18：哨兵成功远程兑换血量的次数，开局为 0，在哨兵成功远程兑换
     *血量后，该值将变为哨兵成功远程兑换血量的次数。
     *bit 19-31：保留
     * */
 uint32_t sentry_info;
} ext_sentry_info_t;

typedef __packed struct //0x020E
{
    /*rader_status:
     *bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发
     *双倍易伤的机会，至多为 2
     *bit 2：对方是否正在被触发双倍易伤
     *    0：对方未被触发双倍易伤
     *    1：对方正在被触发双倍易伤
     *bit 3-7：保留
     * */
 uint8_t radar_info;
} ext_radar_info_t;


typedef __packed struct // 0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t;

//操作手可使用自定义控制器模拟键鼠操作选手端。
typedef __packed struct //0x0306
{
uint16_t key_value;
 uint16_t x_position:12;
 uint16_t mouse_left:4;
 uint16_t y_position:12;
 uint16_t mouse_right:4;
 uint16_t reserved;
}custom_client_data_t;

/*哨兵机器人或选择了半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据，
该路径会在小地图上显示。*/

typedef __packed struct //0x0307
{
uint8_t intention;
uint16_t start_position_x;
uint16_t start_position_y;
int8_t delta_x[49];
int8_t delta_y[49];
uint16_t sender_id;
}map_data_t;

//己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示。
typedef __packed struct //0x0308
{ 
uint16_t sender_id;
uint16_t receiver_id;
uint16_t user_data[30];
} custom_info_t;


typedef __packed struct
{
    fp32 data1;
    fp32 data2;
    fp32 data3;
    uint8_t data4;
} custom_data_t;

typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

extern UART_HandleTypeDef huart6;

class Referee
{

public:
    uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
    fifo_s_t referee_fifo;
    uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];

    //裁判结构体数据
    frame_header_struct_t referee_receive_header;                //接受报文
    frame_header_struct_t referee_send_header;                   //发送报文
    ext_game_state_t game_state;                                 //比赛状态
    ext_game_result_t game_result;                               //比赛结果
    ext_game_robot_HP_t game_robot_HP_t;                         //场上机器人的血量
    ext_event_data_t field_event;                                //场地事件数据
    ext_supply_projectile_action_t supply_projectile_action_t;   //补给站动作标识数据
    //delect ext_supply_projectile_booking_t supply_projectile_booking_t; //补给站动作标识数据
    ext_referee_warning_t referee_warning_t;                     //裁判警告信息  己方警告发生后发送
    ext_dart_remaining_time_t dart_remaining_time_t;             //飞镖发射口倒计时
    ext_game_robot_state_t robot_state;                          //比赛机器人状态
    ext_power_heat_data_t power_heat_data_t;                     //实时功率热量数据
    ext_game_robot_pos_t game_robot_pos_t;                       //机器人位置
    ext_buff_musk_t buff_musk_t;                                 //机器人增益
    ext_air_support_data_t air_support_data_t;                        //空中机器人能量状态(可攻击时间)
    ext_robot_hurt_t robot_hurt_t;                               //伤害来源
    ext_shoot_data_t shoot_data_t;                               //实时射击信息
    //ext_bullet_remaining_t bullet_remaining_t;                   //子弹剩余发射数
    ext_projectile_allowance_t projectile_allowance_t;           //子弹剩余发射数
    ext_rfid_status_t rfid_status_t;                             // RFID状态
    /*0x20?*/
    ext_dart_client_cmd_t dart_client_cmd_t;                       //飞镖发射架状态
    ext_ground_robot_position_t ground_robot_position_t;         //机器人坐标位置
    ext_radar_mark_data_t radar_mark_data_t;                     //对方机器人被标记进度 
    ext_sentry_info_t sentry_info_t;                             //哨兵兑换状态
    ext_radar_info_t  radar_info_t  ;                               //雷达易伤状态

    ext_student_interactive_data_t student_interactive_data_t;


    uint8_t Judge_Self_ID;        //当前机器人的ID
    uint16_t Judge_SelfClient_ID; //发送者机器人对应的客户端ID

    uint8_t Color;

    void init();

    void unpack(); //对数据进行解包

    void init_referee_struct_data();

    void referee_data_solve(uint8_t *frame);

    //*************************数据查询接口*********************************//
    //返回机器人ID
    void get_robot_id(uint8_t *color);
    //底盘输出功率,底盘功率缓存
    void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

    void get_chassis_power_limit(fp32 *power_limit);

    // 17mm枪口热量上限, 17mm枪口实时热量 默认ID1
    void get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat);
    void get_shooter_id2_17mm_cooling_limit_and_heat(uint16_t *id2_17mm_cooling_limit, uint16_t *id2_17mm_cooling_heat);
    // 17mm枪口枪口射速上限,17mm实时射速 默认ID1
    //void get_shooter_id1_17mm_speed_limit_and_bullet_speed(uint16_t *id1_17mm_speed_limit, fp32 *bullet_speed);
    // 17mm枪口热量冷却 默认ID1
    void get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate);
    // 42mm枪口热量上限, 42mm枪口实时热量
    void get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat);
    // 42mm枪口枪口射速上限,42mm实时射速
    //void get_shooter_id1_42mm_speed_limit_and_bullet_speed(uint16_t *id1_42mm_speed_limit, fp32 *bullet_speed);
    // 42mm枪口热量冷却
    void get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate);
    //当前血量
    void get_remain_hp(uint16_t *remain_HP);
    //获取当前阵营颜色
    void get_color(uint8_t *color);

    //是否被击打
    bool_t if_hit();

    void determine_ID(void);
};

#endif

