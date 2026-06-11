#include "dummy_motormatic.h"

DOF6Kinematic_Handle_t kinematic_handle; // 运动学处理句柄

Pose6D_t current_pose;             // 当前位姿监测
Joint6D_t target_joints_angle;     // 目标关节角度
Joint6D_t current_joints_feedback; // 当前关节角度反馈 (从电机读取)
IKSolves_t ik_results;             // 逆解结果
int best_sol_index = 1;            // 最优解索引
bool is_pose_initialized = false;  // 位姿是否初始化

Publisher_t *dummy_motormatic_pub;   // 控制消息发布者
Subscriber_t *dummy_motormatic_sub;  // 反馈信息订阅者
Dummy_Ctrl_Cmd_s dummy_cmd_data;     // 控制命令缓存
Dummy_Upload_Data_s dummy_feed_data; // 反馈数据缓存
Pose6D_t target_pose;                // 期望目标位姿

Joint_Angle_State_t joint_angle;
Gripper_Config_t gripper_config;     // 夹爪配置结构体

void Dummy_Joint_Update(void)
{
    Joint_Motor_Check_Connection();
    Joint_Motor_Get_All_Angles_And_State(&joint_angle);
    for (uint8_t i = 0; i < 6; i++)
    {
        // Joint_Motor_Query_State(i + 1); // 电机ID从1开始
        dummy_feed_data.joint_motor[i].reduction_angle = joint_angle.angle[i];
        dummy_feed_data.joint_motor[i].is_finished = joint_angle.state[i];
    }
    dummy_feed_data.current_mode = dummy_cmd_data.arm_mode;
    dummy_feed_data.arm_ctrl_mode = dummy_cmd_data.arm_ctrl_mode;
    dummy_feed_data.gripper_mode = dummy_cmd_data.gripper_mode;
}

void Dummy_Motormatic_Init(void)
{
    // 1. 填充配置结构体 (这里可以根据实际电机情况修改)
    ArmConfig_t my_arm_config = {
        // --- 几何尺寸 (mm) ---
        .L_BASE = 109.0f,
        .D_BASE = 35.0f,
        .L_ARM = 146.0f,
        .L_FOREARM = 115.0f,
        .D_ELBOW = 52.0f,
        .L_WRIST = 72.0f,
        // --- 关节软限位 (度) ---
        .joint_min_limit = {-1700.0f, -75.0f, -30.0f, -180.0f, -120.0f, -360.0f},
        .joint_max_limit = {1700.0f, 75.0f, 180.0f, 180.0f, 120.0, 360.0f}};
    // 1. 电机配置 (减速比 + 方向 + 关节限位)
    StepMotor_Config_t motor_configs[7] = {
        {50.0f, false, 0.0f, 340.0f}, // J1
        {50.0f, true, 0.0f, 155.0f},  // J2
        {50.0f, true, 0.0f, 215.0f},  // J3
        {50.0f, true, 0.0f, 360.0f},  // J4
        {50.0f, true, 0.0f, 240.0f},  // J5
        {50.0f, false, 0.0f, 720.0f}, // J6
        {8.0f, false, 0.0f, 3600.0f}   // 夹爪
    };
    // 初始化电机驱动 (传入配置)
    Joint_Motor_Init(motor_configs);

    // --- 夹爪专门配置参数 ---
    gripper_config.id = 7;
    gripper_config.target_grab_angle = 500.0f;
    gripper_config.stop_threshold_current = 0.5f;
    gripper_config.release_angle = 0.0f;
    gripper_config.speed_limit = 50.0f;
    gripper_config.timeout_loops = 500;

    dummy_motormatic_pub = PubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
    dummy_motormatic_sub = SubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));

    is_pose_initialized = true;
}

void Dummy_Motormatic_Task(void)
{
    if (!is_pose_initialized)
        return;
    SubGetMessage(dummy_motormatic_sub, &dummy_cmd_data);
    if (dummy_cmd_data.arm_mode == ARM_ZERO_FORCE)
    {
        for (size_t i = 0; i < 7; i++)
        {
            Joint_Motor_Set_Current(i + 1, 0.0f);
        }
    }
    else if (dummy_cmd_data.arm_mode == ARM_FREE_MODE)
    {
        if (dummy_cmd_data.arm_ctrl_mode == BIG_ARM_CTRL)
        {
            Joint_Motor_Set_Pos_With_SpeedLimit(1, dummy_cmd_data.joint1_angle, 30.0f);
            Joint_Motor_Set_Pos_With_SpeedLimit(2, dummy_cmd_data.joint2_angle, 30.0f);
            Joint_Motor_Set_Pos_With_SpeedLimit(3, dummy_cmd_data.joint3_angle, 15.0f);
            Joint_Motor_Set_Pos_With_SpeedLimit(4, dummy_cmd_data.joint4_angle, 30.0f);
        }
        else if (dummy_cmd_data.arm_ctrl_mode == SMALL_ARM_CTRL)
        {
            Joint_Motor_Set_Pos_With_SpeedLimit(3, dummy_cmd_data.joint3_angle, 15.0f);
            Joint_Motor_Set_Pos_With_SpeedLimit(4, dummy_cmd_data.joint4_angle, 30.0f);
            Joint_Motor_Set_Pos_With_SpeedLimit(5, dummy_cmd_data.joint5_angle, 30.0f);
            Joint_Motor_Set_Pos_With_SpeedLimit(6, dummy_cmd_data.joint6_angle, 30.0f);
        }
    }
    else if (dummy_cmd_data.arm_mode == ARM_PC_MODE)
    {
        Joint_Motor_Set_Pos_With_SpeedLimit(1, dummy_cmd_data.joint1_angle, 20.0f);
        Joint_Motor_Set_Pos_With_SpeedLimit(2, dummy_cmd_data.joint2_angle, 20.0f);
        Joint_Motor_Set_Pos_With_SpeedLimit(3, dummy_cmd_data.joint3_angle, 20.0f);
        Joint_Motor_Set_Pos_With_SpeedLimit(4, dummy_cmd_data.joint4_angle, 20.0f);
        Joint_Motor_Set_Pos_With_SpeedLimit(5, dummy_cmd_data.joint5_angle, 20.0f);
        Joint_Motor_Set_Pos_With_SpeedLimit(6, dummy_cmd_data.joint6_angle, 20.0f);
    }
    Dummy_Joint_Update();
    // 定期执行夹爪防错状态机，传入结构体配置以及模式枚举
    Joint_Motor_Gripper_Task(&gripper_config, dummy_cmd_data.gripper_mode);
    PubPushMessage(dummy_motormatic_pub, &dummy_feed_data);
}
