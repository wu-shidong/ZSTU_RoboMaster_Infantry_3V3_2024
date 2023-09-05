#ifndef SHOOT_TASK_H__
#define SHOOT_TASK_H__
#include "main.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "pid.h"
//���ң����ͨ��
#define SHOOT_RC_MODE_CHANNEL       1
//���Ħ���ּ���򿪰���
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
//���Ħ���ּ���رհ���
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E
//��곤���ж�
#define PRESS_LONG_TIME             400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
// #define RC_S_LONG_TIME              40
//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15

//΢�����ذ���
#define SWITCH_TRIGGER_ON           0

//΢�������ɿ�
#define SWITCH_TRIGGER_OFF          1

//��ת��ת�ٶ�
#define BLOCK_TRIGGER_SPEED         1.0f

//�ж�Ϊ��ת��ʱ��
#define BLOCK_TIME                  700

//��ת��תʱ��
#define REVERSE_TIME                800//500

//��ת��ת�ٶ�����
#define REVERSE_SPEED_LIMIT         13.0f

//�����ٶ�
#define TRIGGER_SPEED               7.0f//10

//���������ٶ�
#define CONTINUE_TRIGGER_SPEED      10.0f//15

//����׼���ٶ�
#define READY_TRIGGER_SPEED         2.0f//5

//�����ֵ��PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

//�����ֵ��PID���ģʽ����޷�
#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

//�����ֵ��PID���Ž׶�����޷�
#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

//�����ּ�������
#define SHOOT_CONTROL_TIME 1

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191


//PI
// #define PI                          3.1415926535897932384626433832795f

//PI/4
#define PI_FOUR                     0.78539816339744830961566084581988f
//PI/10
#define PI_TEN                      0.314f

//�������ֵת���ɽǶ�ֵ
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f

//�������ֵת���ɽǶ�ֵ
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f

//���Ȧ��ת���ɽǶ�ֵ
#define FULL_COUNT                  18
typedef enum
{
    SHOOT_STOP = 0,//ֹͣ
    SHOOT_READY_FRIC,//Ħ����׼��
    SHOOT_READY_BULLET,//�ӵ�׼������
    SHOOT_READY,//�ӵ����ž���
    SHOOT_BULLET,//�ӵ�����(SEMI)
    SHOOT_CONTINUE_BULLET,//�ӵ�����(AUTO)
    SHOOT_DONE,//�������
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;//���ģʽ
    const RC_ctrl_t *shoot_rc;//ң����ָ��
    const motor_measure_t *shoot_motor_measure;//�������ָ��
    const motor_measure_t *fric1_motor_measure;//Ħ����1ָ��
    const motor_measure_t *fric2_motor_measure;//Ħ����2ָ��
    pid_type_def trigger_motor_pid;//�������PID
    pid_type_def fric1_motor_pid;//Ħ����1PID
    pid_type_def fric2_motor_pid;//Ħ����2PID
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;//�������ת����Ȧ��
    bool_t press_l;//���������±�־λ
    bool_t press_r;//����Ҽ����±�־λ
    bool_t last_press_l;//��������һ�ΰ��±�־λ
    bool_t last_press_r;//����Ҽ���һ�ΰ��±�־λ
    uint16_t press_l_time;//����������ʱ��
    uint16_t press_r_time;//����Ҽ�����ʱ��
    uint16_t rc_s_time;//ң����������ذ���ʱ��

    uint16_t block_time;//��תʱ��
    uint16_t reverse_time;//��תʱ��
    bool_t move_flag;
    bool_t key;
    uint8_t key_time;
    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

#endif