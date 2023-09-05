#ifndef SHOOT_TASK_H__
#define SHOOT_TASK_H__
#include "main.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"
//射击遥控器通道
#define SHOOT_RC_MODE_CHANNEL       1
//射击摩擦轮激光打开按键
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
//射击摩擦轮激光关闭按键
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E
//鼠标长按判断
#define PRESS_LONG_TIME             400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
// #define RC_S_LONG_TIME              40
//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15

//微触开关按下
#define SWITCH_TRIGGER_ON           0

//微触开关松开
#define SWITCH_TRIGGER_OFF          1

//堵转反转速度
#define BLOCK_TRIGGER_SPEED         1.0f

//判定为堵转的时间
#define BLOCK_TIME                  700

//堵转后反转时间
#define REVERSE_TIME                800//500

//堵转后反转速度限制
#define REVERSE_SPEED_LIMIT         13.0f

//拨弹速度
#define TRIGGER_SPEED               7.0f//10

//连发拨弹速度
#define CONTINUE_TRIGGER_SPEED      10.0f//15

//拨弹准备速度
#define READY_TRIGGER_SPEED         2.0f//5

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

//拨弹轮电机PID射击模式输出限幅
#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

//拨弹轮电机PID上膛阶段输出限幅
#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

//拨弹轮计算周期
#define SHOOT_CONTROL_TIME 1

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191


//PI
// #define PI                          3.1415926535897932384626433832795f

//PI/4
#define PI_FOUR                     0.78539816339744830961566084581988f
//PI/10
#define PI_TEN                      0.314f

//电机编码值转化成角度值
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f

//电机编码值转化成角度值
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f

//电机圈数转化成角度值
#define FULL_COUNT                  18
typedef enum
{
    SHOOT_STOP = 0,//停止
    SHOOT_READY_FRIC,//摩擦轮准备
    SHOOT_READY_BULLET,//子弹准备上膛
    SHOOT_READY,//子弹上膛就绪
    SHOOT_BULLET,//子弹单发(SEMI)
    SHOOT_CONTINUE_BULLET,//子弹连发(AUTO)
    SHOOT_DONE,//单发完成
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;//射击模式
    const RC_ctrl_t *shoot_rc;//遥控器指针
    const motor_measure_t *shoot_motor_measure;//拨弹电机指针
    const motor_measure_t *fric1_motor_measure;//摩擦轮1指针
    const motor_measure_t *fric2_motor_measure;//摩擦轮2指针
    pid_type_def trigger_motor_pid;//拨弹电机PID
    pid_type_def fric1_motor_pid;//摩擦轮1PID
    pid_type_def fric2_motor_pid;//摩擦轮2PID
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;//拨弹电机转过的圈数
    bool_t press_l;//鼠标左键按下标志位
    bool_t press_r;//鼠标右键按下标志位
    bool_t last_press_l;//鼠标左键上一次按下标志位
    bool_t last_press_r;//鼠标右键上一次按下标志位
    uint16_t press_l_time;//鼠标左键按下时间
    uint16_t press_r_time;//鼠标右键按下时间
    uint16_t rc_s_time;//遥控器射击开关按下时间

    uint16_t block_time;//堵转时间
    uint16_t reverse_time;//反转时间
    bool_t move_flag;
    bool_t key;
    uint8_t key_time;
    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

#endif
