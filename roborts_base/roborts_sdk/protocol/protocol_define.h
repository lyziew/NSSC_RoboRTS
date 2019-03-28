/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_SDK_PROTOCOL_DEFINE_H
#define ROBORTS_SDK_PROTOCOL_DEFINE_H

namespace roborts_sdk {
/*
#pragma pack (n)              作用：C编译器将按照n个字节对齐。
#pragma pack ()               作用：取消自定义字节对齐方式。
#pragma  pack (push,1)        作用：是指把原来对齐方式设置压栈，并设新的对齐方式设置为一个字节对齐
等同于调用 #pragma pack(push) #pragma pack(1)
#pragma pack(pop)             作用：恢复对齐状态
*/
#pragma pack(push, 1)
//DEVICE_ADDRESS
#define MANIFOLD2_ADDRESS              (0x00u)
#define CHASSIS_ADDRESS                (0X01u)
#define GIMBAL_ADDRESS                 (0X02u)
#define BROADCAST_ADDRESS              (0Xffu)

//CMD_SET
//开启通讯的命令,用于初始化上位机与A型板的通讯                            
#define UNIVERSAL_CMD_SET              (0x00u)
//用于向裁判系统发送指令,用于补给
#define REFEREE_SEND_CMD_SET           (0x01u)
//地盘通信指令,用于发送和接收来自地盘的数据
#define CHASSIS_CMD_SET                (0x02u)
//云台通信指令,用于发送和接收来自云台的数据
#define GIMBAL_CMD_SET                 (0x03u)
//校准指令
#define COMPATIBLE_CMD_SET             (0x04u)

//裁判系统数据
//用于获取比赛状态数据,包括赛程状态,比赛结果,双方机器人存活状态
#define REFEREE_GAME_CMD_SET           (0x40u)
//用于获取场地B数据,Buff区和补给区状态
#define REFEREE_BATTLEFIELD_CMD_SET    (0x41u)
//用于获取机器人状态,机器人当前状态.热量状态,buff状态,是否受到伤害,射击数据
#define REFEREE_ROBOT_CMD_SET          (0x42u)
#define REFEREE_RECEIVE_CMD_SET        (0x43u)

#define TEST_CMD_SET                   (0xFFu)

/*----------------------------UNIVERSAL_CMD--- 0x00 ---------------------*/
#define CMD_HEARTBEAT                  (0x01u)
typedef struct{
  uint32_t heartbeat;
} cmd_heartbeat;

#define CMD_REPORT_VERSION             (0X02u)
typedef struct
{
  uint32_t version_id;
} cmd_version_id;

/*-----------------------------REFEREE_SEND_CMD--- 0x01 ---------------------*/
#define CMD_REFEREE_SEND_DATA               (0X01u)

typedef struct
{
  uint16_t cmd = 0x0103u;
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_num;
} cmd_supply_projectile_booking;

/*-------------------REFEREE_CMD--- 0x01 SEND 0x43 RECEIVE------------------*/
/*  Define your own protocol for comm between different robots here*/
/*    typedef __packed struct
/*    {
/*        uint16_t cmd;
/*        uint16_t sender_robot_id;
/*        uint16_t receiver_robot_id;
/*        CertainType data;
/*    } ext_robot_comm_t;
 */

/*-----------------------------CHASSIS_CMD---- 0x02 ---------------------*/

#define CMD_PUSH_CHASSIS_INFO          (0X01u)
typedef struct {
  int16_t gyro_angle;
  int16_t gyro_rate;
  int32_t position_x_mm;
  int32_t position_y_mm;
  int16_t angle_deg;
  int16_t v_x_mm;
  int16_t v_y_mm;
} cmd_chassis_info;

#define CMD_SET_CHASSIS_SPEED          (0X03u)
typedef struct {
  int16_t vx;
  int16_t vy;
  int16_t vw;
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
} cmd_chassis_speed;

#define CMD_GET_CHASSIS_PARAM          (0X04u)
typedef struct {
  uint16_t wheel_perimeter;
  uint16_t wheel_track;
  uint16_t wheel_base;
  int16_t gimbal_x_offset;
  int16_t gimbal_y_offset;
} cmd_chassis_param;

#define CMD_SET_CHASSIS_SPD_ACC        (0X05u)
typedef struct {
  int16_t vx;
  int16_t vy;
  int16_t vw;
  int16_t ax;
  int16_t ay;
  int16_t wz;
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
} cmd_chassis_spd_acc;

/*-----------------------------GIMBAL_CMD---- 0x03 ---------------------*/

#define CMD_PUSH_GIMBAL_INFO           (0X01u)
typedef struct {
  uint8_t mode;
  int16_t pitch_ecd_angle;
  int16_t yaw_ecd_angle;
  int16_t pitch_gyro_angle;
  int16_t yaw_gyro_angle;
  int16_t yaw_rate;
  int16_t pitch_rate;
} cmd_gimbal_info;

#define CMD_SET_GIMBAL_MODE            (0X02u)
typedef enum {
  GYRO_CONTROL,
  CODE_CONTROL,
  G_MODE_MAX_NUM,
} gimbal_mode_e;

#define CMD_SET_GIMBAL_ANGLE           (0x03u)
typedef struct{
  union{
    uint8_t flag;
    struct {
      uint8_t yaw_mode:   1;//0 means absolute, 1 means relative;
      uint8_t pitch_mode: 1;
    } bit;
  } ctrl;
  int16_t pitch;
  int16_t yaw;
}cmd_gimbal_angle;

#define CMD_SET_FRIC_WHEEL_SPEED       (0X04u)
typedef struct{
  uint16_t left;
  uint16_t right;
} cmd_fric_wheel_speed;

#define CMD_SET_SHOOT_INFO             (0x05u)
typedef enum {
  SHOOT_STOP = 0,
  SHOOT_ONCE,
  SHOOT_CONTINUOUS,
} shoot_cmd_e;

typedef struct {
  uint8_t shoot_cmd;
  uint32_t shoot_add_num;
  uint16_t shoot_freq;
} cmd_shoot_info;

/*------------------------COMPATIBLE_CMD---- 0x04 -------------------*/
#define CMD_RC_DATA_FORWARD            (0X01u)

#define CMD_PUSH_UWB_INFO              (0X02u)
typedef struct {
  int16_t x;
  int16_t y;
  uint16_t yaw;
  int16_t distance[6];
  uint16_t error;
  uint16_t res;
} cmd_uwb_info;

/*------------------------REFEREE_GAME_CMD---- 0x40 -------------------*/
#define CMD_GAME_STATUS            (0X01u)
typedef struct
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
} cmd_game_state;

#define CMD_GAME_RESULT            (0X02u)
typedef struct
{
  uint8_t winner;
} cmd_game_result;

#define CMD_GAME_SURVIVAL          (0X03u)
typedef struct
{
  uint16_t robot_legion;
} cmd_game_robot_survivors;


/*-------------------REFEREE_BATTLEFIELD_CMD_SET---- 0x41 -------------*/
#define CMD_BATTLEFIELD_EVENT      (0X01u)
typedef struct
{
  uint32_t event_type;
} cmd_event_data;

#define CMD_SUPPLIER_ACTION        (0X02u)
typedef struct
{
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} cmd_supply_projectile_action;


/*------------------------REFEREE_ROBOT_CMD---- 0x42 -------------------*/
#define CMD_ROBOT_STATUS           (0X01u)
typedef struct
{
  uint8_t robot_id; //红 3,4 蓝13,14
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;
  uint16_t shooter_heat0_cooling_rate;//冷却率
  uint16_t shooter_heat0_cooling_limit;//热量上限
  uint16_t shooter_heat1_cooling_rate;//无用
  uint16_t shooter_heat1_cooling_limit;//无用
  //C语言允许在一个结构体中以位为单位来指定其成员所占内存长度，这种以位为单位的成员称为“位段”或称“位域”( bit field)
  uint8_t mains_power_gimbal_output : 1; //电源输出情况,1为有电,0为没电
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} cmd_game_robot_state;

#define CMD_ROBOT_POWER_HEAT         (0X02u)
typedef struct
{
  uint16_t chassis_volt;//无用
  uint16_t chassis_current;//无用
  float chassis_power;//无用
  uint16_t chassis_power_buffer;//无用
  uint16_t shooter_heat0;//当前热量
  uint16_t shooter_heat1;//无用
} cmd_power_heat_data;

#define CMD_ROBOT_POSITION            (0X03u)
typedef struct
{
  float x;
  float y;
  float z;
  float yaw;
} cmd_game_robot_pos;

#define CMD_ROBOT_BUFF                (0X04u)
typedef struct
{
  uint8_t power_rune_buff;
} cmd_buff_musk;

#define CMD_AERIAL_ENERGY             (0X05u)
typedef struct
{
  uint8_t energy_point;
  uint8_t attack_time;
} cmd_aerial_robot_energy;

#define CMD_ROBOT_HURT                (0X06u)
typedef struct
{
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} cmd_robot_hurt;

#define CMD_ROBOT_SHOOT               (0X07u)
typedef struct
{
  uint8_t bullet_type;
  uint8_t bullet_freq;
  float bullet_speed;
} cmd_shoot_data;

/*-----------------------------TEST_CMD---- 0xFF ---------------------*/
#define TEXT_ECHO_TRANSMIT             (0x00u)
#pragma pack(pop)
}
#endif //ROBORTS_SDK_PROTOCOL_DEFINE_H
