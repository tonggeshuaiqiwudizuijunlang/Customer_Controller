#include "dummy_motormatic.h"

 DOF6Kinematic_Handle_t kinematic_handle; // 运动学处理句柄
 Pose6D_t target_pose;                    // 期望目标位姿
 Pose6D_t current_pose;                   // 当前位姿监测
 Joint6D_t target_joints_angle;           // 目标关节角度
 Joint6D_t current_joints_feedback;       // 当前关节角度反馈 (从电机读取)
 IKSolves_t ik_results;                   // 逆解结果
 int best_sol_index;                      // 最优解索引
 bool is_pose_initialized = false;        // 位姿是否初始化

 Publisher_t *dummy_motormatic_pub;   // 控制消息发布者
 Subscriber_t *dummy_motormatic_sub;  // 反馈信息订阅者
 Dummy_Ctrl_Cmd_s dummy_cmd_data;     // 控制命令缓存
 Dummy_Upload_Data_s dummy_feed_data; // 反馈数据缓存

 void Dummy_Joint_Update(void)
{
    Joint_Motor_Check_Connection();
    dummy_feed_data.cur_x = current_pose.X;
    dummy_feed_data.cur_y = current_pose.Y;
    dummy_feed_data.cur_z = current_pose.Z;
    dummy_feed_data.cur_roll = current_pose.A;
    dummy_feed_data.cur_pitch = current_pose.B;
    dummy_feed_data.cur_yaw = current_pose.C;
    for (uint8_t i = 0; i < 7; i++)
    {
        Joint_Motor_Query_State(i + 1); // 电机ID从1开始
        dummy_feed_data.joint_motor[i].reduction_angle = current_joints_feedback.a[i];
    }
    dummy_feed_data.current_mode = dummy_cmd_data.arm_mode;
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
        .joint_min_limit = {-1700, -730, 350, -1800, -1200, -7200},
        .joint_max_limit = {1700, 900, 1800, 1800, 1200, 7200}};
        // .joint_min_limit = {-170, -73, 35, -180, -120, -720},
        // .joint_max_limit = {170, 90, 180, 180, 120, 720}};

    // 1. 电机配置 (减速比 + 方向)
    StepMotor_Config_t motor_configs[7] = {
        {50.0f, false, 0.0f}, // J1: 假设 1:1, 正向 (根据Dummy源码 ID1: 30, true)
        {50.0f, true, 0.0f},  // J2
        {50.0f, true, 0.0f}, // J3
        {50.0f, true, 0.0f},  // J4
        {30.0f, false, 0.0f}, // J5
        {50.0f, true, 0.0f},  // J6
        {8.0f, false, 0.0f}   // J7 (夹爪电机，假设 1:8, 正向)
    };
    // 初始化电机驱动 (传入配置)
    Joint_Motor_Init(motor_configs);
    // 2. 调用初始化函数
    Kinematic_Init(&kinematic_handle, &my_arm_config);
    // 3. 读取电机当前角度
    // Joint_Motor_Query_State(0);
    Joint_Motor_Get_All_Angles(&current_joints_feedback);
    // 4. 计算正向运动学
    Kinematic_SolveFK(&kinematic_handle, &current_joints_feedback, &target_pose);

    dummy_motormatic_pub = PubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
    dummy_motormatic_sub = SubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));

    is_pose_initialized = true;
}

void Dummy_Motormatic_Task(void)
{
    if (!is_pose_initialized)
        return;
    if (SubGetMessage(dummy_motormatic_sub, &dummy_cmd_data))
    {
        // 只有收到了有效指令，才更新目标
        // 防止全0数据覆盖初始化的有效Pose
        if (fabs(dummy_cmd_data.target_pose.X) > 0.001f ||
            fabs(dummy_cmd_data.target_pose.Z) > 0.001f)
        {
            target_pose = dummy_cmd_data.target_pose;
        }
    }
    // 从电机读取当前关节角度反馈
    Joint_Motor_Get_All_Angles(&current_joints_feedback);
    Kinematic_SolveFK(&kinematic_handle, &current_joints_feedback, &current_pose);
    // 2. 核心计算：求解逆运动学
    // 输入: target_pose (我想去哪)
    // 参考: current_joints_feedback (我现在在哪，用于选最近的解)
    bool has_solution = Kinematic_SolveIK(&kinematic_handle, &target_pose, &current_joints_feedback, &ik_results);
    if (has_solution)
    {
        // 2. 挑选最优解 (输入: 把刚才算出来的8组解, 和当前电机角度做比较)
        best_sol_index = Kinematic_Select_Best_Sol(&kinematic_handle, &ik_results, &current_joints_feedback);
        if (best_sol_index != -1)
        {
            // 找到了最佳解，将其存入 target_joints_angle
            target_joints_angle = ik_results.config[best_sol_index];
        }
    }
    if (best_sol_index != -1)
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            // 度 -> 圈
            float target_turns = target_joints_angle.a[i];
            if (dummy_cmd_data.arm_mode == ARM_ZERO_FORCE) // 零力模式，电机不动
                Joint_Motor_Enable(i + 1, false);
            else
                Joint_Motor_Set_Pos_With_SpeedLimit(i + 1, target_turns, 30.0f);
        }
    }
    // [新增] 必须持续发布反馈，否则 Cmd 任务无法完成初始化同步
    Dummy_Joint_Update();
    PubPushMessage(dummy_motormatic_pub, &dummy_feed_data);
}