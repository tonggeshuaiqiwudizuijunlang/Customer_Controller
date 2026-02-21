#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include "step_motor.h"
#include "dummy_kinematic.h"


/* app层定义 */

typedef enum
{
    ARM_ZERO_FORCE = 0,
    ARM_FREE_MODE,
    ARM_HOME_MODE,
    ARM_PC_MODE // 上位机模式
} arm_mode_e;

typedef enum
{
    GRIPPER_RELEASE = 0,    // 松开
    GRIPPER_AUTO_GRAB = 1,  // 自动抓取
    GRIPPER_MANUAL_GRAB     // 手动抓取
} gripper_mode_e;

typedef enum
{
    ARM_STATUS_IDLE = 0,    // 空闲
    ARM_STATUS_MOVING,      // 正在移动中
    ARM_STATUS_ARRIVED,     // 已到达目标点 (误差范围内)
    ARM_STATUS_ERROR        // 发生错误 (如逆解无解、碰撞保护)
} arm_state_e;



typedef struct
{
    /* --- 机械臂模式 (Arm Mode) --- */
    arm_mode_e arm_mode;
    gripper_mode_e gripper_mode;
    arm_state_e  arm_state;
    float    arm_max_speed; // 机械臂最大速度 (单位: mm/s)
    /* --- 目标末端位姿 (Target End-Effector Pose) --- */
    Pose6D_t target_pose;

} Dummy_Ctrl_Cmd_s;

typedef struct
{
    /* --- 机械臂模式 (Arm Mode) --- */
    arm_mode_e current_mode;
    gripper_mode_e gripper_mode;
    arm_state_e  arm_state;
    
    /* --- 当前末端位姿 (Current Pose) --- */
    float    cur_x;
    float    cur_y;
    float    cur_z;
    
    float    cur_roll;
    float    cur_pitch;
    float    cur_yaw;
    
    /* --- 关节详细信息 --- *
    * joint_motor[0~5] 对应 Axis 1-6
    * joint_motor[6]   对应 夹爪 */
    Joint_t joint_motor[7];
} Dummy_Upload_Data_s;

#endif // ROBOT_TYPES_H
