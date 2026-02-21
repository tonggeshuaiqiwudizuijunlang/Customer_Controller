#include "dummy_kinematic.h"
#include "step_motor.h"

/**
 * @brief 矩阵乘法
 * @param _m A的行数
 * @param _l A的列数 (也是B的行数)
 * @param _n B的列数
 */
static void MatMultiply(const float *_matrix1, const float *_matrix2, float *_matrixOut,
                        const int _m, const int _l, const int _n)
{
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}

/**
 * @brief 旋转矩阵转欧拉角 (ZYX顺序: Roll->Pitch->Yaw)
 *        用于正解计算末端姿态
 */
static void RotMatToEulerAngle(const float *_rotationM, float *_eulerAngles)
{
    float A, B, C;

    // 检查 Gimbal Lock (万向锁) 情况
    if (fabsf(_rotationM[6]) >= 1.0f - 0.0001f)
    {
        if (_rotationM[6] < 0)
        {
            A = 0.0f;
            B = (float)M_PI_2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        }
        else
        {
            A = 0.0f;
            B = -(float)M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    }
    else
    {
        // 正常情况求解
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        float cb = cosf(B);
        // 防止除零
        if (fabsf(cb) < 0.0001f)
            cb = 0.0001f;
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }
    _eulerAngles[0] = C; // Yaw (Z)
    _eulerAngles[1] = B; // Pitch (Y)
    _eulerAngles[2] = A; // Roll (X)
}

/**
 * @brief 欧拉角转旋转矩阵
 *        用于逆解时将目标姿态转为矩阵
 */
static void EulerAngleToRotMat(const float *_eulerAngles, float *_rotationM)
{
    float ca, cb, cc, sa, sb, sc;
    // 提前计算三角函数
    cc = cosf(_eulerAngles[0]);
    cb = cosf(_eulerAngles[1]);
    ca = cosf(_eulerAngles[2]);
    sc = sinf(_eulerAngles[0]);
    sb = sinf(_eulerAngles[1]);
    sa = sinf(_eulerAngles[2]);

    // 填充旋转矩阵 R = Rz * Ry * Rx
    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
}

// -------------------------------------------------------------------------
//                            核心接口实现
// -------------------------------------------------------------------------

/**
 * @brief 初始化运动学句柄
 */
void Kinematic_Init(DOF6Kinematic_Handle_t *handle, const ArmConfig_t *config)
{
    // 1. 深拷贝配置参数
    memcpy(&handle->config, config, sizeof(ArmConfig_t));
    
    // 为了写代码方便，定义局部指针指向配置
    const ArmConfig_t *cfg = &handle->config;

    // 2. 初始化标准 DH 参数矩阵 [theta, d, a, alpha]
    // 使用配置中的参数填充
    float tmp_DH[6][4] = {
        {0.0f,           cfg->L_BASE,    cfg->D_BASE,  -(float)M_PI_2},
        {-(float)M_PI_2, 0.0f,           cfg->L_ARM,   0.0f},
        {(float)M_PI_2,  cfg->D_ELBOW,   0.0f,         (float)M_PI_2},
        {0.0f,           cfg->L_FOREARM, 0.0f,         -(float)M_PI_2},
        {0.0f,           0.0f,           0.0f,         (float)M_PI_2},
        {0.0f,           cfg->L_WRIST,   0.0f,         0.0f}
    };
    memcpy(handle->DH_matrix, tmp_DH, sizeof(tmp_DH));

    // 3. 预计算向量
    float tmp_L1[3] = {cfg->D_BASE, -cfg->L_BASE, 0.0f};
    memcpy(handle->L1_base, tmp_L1, 12);
    
    float tmp_L2[3] = {cfg->L_ARM, 0.0f, 0.0f};
    memcpy(handle->L2_arm, tmp_L2, 12);
    
    float tmp_L3[3] = {-cfg->D_ELBOW, 0.0f, cfg->L_FOREARM};
    memcpy(handle->L3_elbow, tmp_L3, 12);
    
    float tmp_L6[3] = {0.0f, 0.0f, cfg->L_WRIST};
    memcpy(handle->L6_wrist, tmp_L6, 12);

    // 4. 预计算平方项
    handle->l_se_2 = cfg->L_ARM * cfg->L_ARM;
    handle->l_se = cfg->L_ARM;
    handle->l_ew_2 = cfg->L_FOREARM * cfg->L_FOREARM + cfg->D_ELBOW * cfg->D_ELBOW;
    
    // 注意：l_ew 和 atan_e 在 SolveIK 中使用了 Lazy Init (第一次调用时计算)
    handle->l_ew = 0; 
    handle->atan_e = 0;
}

/**
 * @brief 正运动学解算 (Joints -> Pose)
 */
bool Kinematic_SolveFK(const DOF6Kinematic_Handle_t *handle,
                       const Joint6D_t *inputJoints,
                       Pose6D_t *outputPose)
{
    float q_in[6];
    float q[6];
    float cosq, sinq, cosa, sina;
    float P06[6]; // 最终位置 [x,y,z, r,p,y]
    float R06[9]; // 最终旋转矩阵 0->6

    // 临时矩阵变量
    float R[6][9]; // 各关节的局部变换矩阵
    float R02[9], R03[9], R04[9], R05[9];
    float L0_bs[3], L0_se[3], L0_ew[3], L0_wt[3];

    // 输入角度转弧度 (但实际上 inputJoints 已经是度了，这里逻辑是输入是度)
    // 注意：原代码逻辑有点怪，这里假设 inputJoints 是度，内部转弧度计算
    for (int i = 0; i < 6; i++)
        q_in[i] = inputJoints->a[i] * DEG_TO_RAD_CONST;

    // 逐个计算每个关节的局部变换矩阵 R[i]
    for (int i = 0; i < 6; i++)
    {
        q[i] = q_in[i] + handle->DH_matrix[i][0]; // 加上 DH θ 偏置
        cosq = cosf(q[i]);
        sinq = sinf(q[i]);
        cosa = cosf(handle->DH_matrix[i][3]); // DH α
        sina = sinf(handle->DH_matrix[i][3]);

        R[i][0] = cosq;
        R[i][1] = -cosa * sinq;
        R[i][2] = sina * sinq;
        R[i][3] = sinq;
        R[i][4] = cosa * cosq;
        R[i][5] = -sina * cosq;
        R[i][6] = 0.0f;
        R[i][7] = sina;
        R[i][8] = cosa;
    }

    // 级联乘法：计算 R0->2, R0->3 ... R0->6
    MatMultiply(R[0], R[1], R02, 3, 3, 3);
    MatMultiply(R02, R[2], R03, 3, 3, 3);
    MatMultiply(R03, R[3], R04, 3, 3, 3);
    MatMultiply(R04, R[4], R05, 3, 3, 3);
    MatMultiply(R05, R[5], R06, 3, 3, 3);

    // 向量变换：将各杆件向量变换到基坐标系
    MatMultiply(R[0], handle->L1_base, L0_bs, 3, 3, 1);
    MatMultiply(R02, handle->L2_arm, L0_se, 3, 3, 1);
    MatMultiply(R03, handle->L3_elbow, L0_ew, 3, 3, 1);
    MatMultiply(R06, handle->L6_wrist, L0_wt, 3, 3, 1);

    // 向量叠加：P06 = P_base + P_shoulder + P_elbow + P_wrist
    for (int i = 0; i < 3; i++)
        P06[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];

    // 旋转矩阵转欧拉角
    RotMatToEulerAngle(R06, &(P06[3]));

    // 输出结果 (注意单位换算: m->mm, rad->deg)
    // 原代码这里似乎想输出mm，但P06计算用的单位取决于L_BASE单位。
    // 如果Kinematic_Init传入的是mm，这里P06就是mm。
    outputPose->X = P06[0];
    outputPose->Y = P06[1];
    outputPose->Z = P06[2];
    outputPose->A = P06[3] * RAD_TO_DEG_CONST;
    outputPose->B = P06[4] * RAD_TO_DEG_CONST;
    outputPose->C = P06[5] * RAD_TO_DEG_CONST;
    memcpy(outputPose->R, R06, 9 * sizeof(float));
    outputPose->hasR = true;

    return true;
}

/**
 * @brief 逆运动学解算 (Pose -> Joints)
 *        使用 Paul 反变换法 / 几何法混合求解
 */
bool Kinematic_SolveIK(DOF6Kinematic_Handle_t *handle,
                       const Pose6D_t *inputPose,
                       const Joint6D_t *lastJoints,
                       IKSolves_t *outputSolves)
{
    // 中间变量定义
    float qs[2];    // 关节1的两个解 (Front/Back)
    float qa[2][2]; // 关节2,3的解 (Elbow Up/Down)
    float qw[2][3]; // 关节4,5,6的解 (Wrist Flip/NoFlip)
    float cosqs, sinqs;
    float cosqa[2], sinqa[2];
    float cosqw; // [修改] 删除了 sinqw
    float P06[6], R06[9];

    // 关键向量
    float P0_w[3];            // 腕部中心点 (基坐标系下)
    float P1_w[3];            // 腕部中心点 (关节1坐标系下)
    float L0_wt[3], L1_sw[3]; // 临时向量

    // 临时旋转矩阵
    float R10[9], R31[9], R30[9], R36[9];
    float l_sw_2, l_sw, atan_a, acos_a, acos_e;

    int ind_arm, ind_elbow, ind_wrist;
    int i;

    // --- 1. Lazy Init: 如果还没计算过这个几何参数，算一下 ---
    // [修复] 使用阈值判断浮点数是否为0，而不是直接 ==
    if (handle->l_ew < 0.0001f && handle->l_ew > -0.0001f)
    {
        handle->l_ew = sqrtf(handle->l_ew_2);
        handle->atan_e = atanf(handle->config.D_ELBOW / handle->config.L_FOREARM);
    }

    // --- 2. 准备目标位姿 ---
    // 输入如果是mm，这里不需要除1000，除非你的算法内部用m。假设全用mm。
    P06[0] = inputPose->X;
    P06[1] = inputPose->Y;
    P06[2] = inputPose->Z;

    // 如果还没有旋转矩阵，先算出来
    if (!inputPose->hasR)
    {
        float euler[3] = {// 转回弧度
                          inputPose->A * DEG_TO_RAD_CONST,
                          inputPose->B * DEG_TO_RAD_CONST,
                          inputPose->C * DEG_TO_RAD_CONST};
        EulerAngleToRotMat(euler, R06);
    }
    else
    {
        memcpy(R06, inputPose->R, 9 * sizeof(float));
    }

    // --- 3. 腕部解耦 (Wrist Decoupling) ---
    // 目标：计算腕部中心点 P_w = P_end - d6 * R * z_axis
    MatMultiply(R06, handle->L6_wrist, L0_wt, 3, 3, 1);
    for (i = 0; i < 3; i++)
        P0_w[i] = P06[i] - L0_wt[i];

    // --- 4. 求解关节 1 (Theta 1) ---
    // 几何法：直接看 XY 平面投影
    if (sqrtf(P0_w[0] * P0_w[0] + P0_w[1] * P0_w[1]) <= 0.000001f) // 奇异：在Z轴上
    {
        // 保持上一时刻角度
        qs[0] = lastJoints->a[0] * DEG_TO_RAD_CONST;
        qs[1] = lastJoints->a[0] * DEG_TO_RAD_CONST;
        // 标记解失效 (简化处理)
        for (i = 0; i < 4; i++)
        {
            // [修复] 显式强转消除符号警告
            outputSolves->solFlag[0 + i][0] = (char)-1;
            outputSolves->solFlag[4 + i][0] = (char)-1;
        }
    }
    else
    {
        // 正常解：Front 和 Back 两个方向
        qs[0] = atan2f(P0_w[1], P0_w[0]);
        qs[1] = atan2f(-P0_w[1], -P0_w[0]);
        for (i = 0; i < 4; i++)
        {
            outputSolves->solFlag[0 + i][0] = 1;
            outputSolves->solFlag[4 + i][0] = 1;
        }
    }

    // --- 5. 遍历两个 Theta 1 解，求解 Theta 2, 3 ---
    for (ind_arm = 0; ind_arm < 2; ind_arm++)
    {
        // 变换到关节1坐标系
        cosqs = cosf(qs[ind_arm] + handle->DH_matrix[0][0]);
        sinqs = sinf(qs[ind_arm] + handle->DH_matrix[0][0]);

        // 构造 R10 为了把 P_w 变换到局部
        R10[0] = cosqs;
        R10[1] = sinqs;
        R10[2] = 0.0f;
        R10[3] = 0.0f;
        R10[4] = 0.0f;
        R10[5] = -1.0f;
        R10[6] = -sinqs;
        R10[7] = cosqs;
        R10[8] = 0.0f;

        MatMultiply(R10, P0_w, P1_w, 3, 3, 1); // P1_w 是腕点在 J1 下的坐标

        // 计算辅助向量 L_sw (Shoulder to Wrist)
        for (i = 0; i < 3; i++)
            L1_sw[i] = P1_w[i] - handle->L1_base[i];

        l_sw_2 = L1_sw[0] * L1_sw[0] + L1_sw[1] * L1_sw[1];
        l_sw = sqrtf(l_sw_2);

        // --- 三角形余弦定理求解 ---
        // l_sw, l_se (大臂), l_ew (小臂) 构成三角形

        // 检查是否超出工作空间
        if (l_sw > (handle->l_se + handle->l_ew) || l_sw < fabsf(handle->l_se - handle->l_ew))
        {
            // 够不着，标记无效
            // (此处省略详细的奇异处理代码，直接给一个默认解)
            qa[0][0] = 0;
            qa[0][1] = 0;
            qa[1][0] = 0;
            qa[1][1] = 0;
            // 标记 flag = 0
            for (int k = 0; k < 4; k++)
                outputSolves->solFlag[4 * ind_arm + k][1] = 0;
        }
        else
        {
            // 正常求解
            atan_a = atan2f(L1_sw[1], L1_sw[0]);
            acos_a = 0.5f * (handle->l_se_2 + l_sw_2 - handle->l_ew_2) / (handle->l_se * l_sw);

            // 钳制范围防止 NaN
            if (acos_a > 1.0f)
                acos_a = 1.0f;
            else if (acos_a < -1.0f)
                acos_a = -1.0f;
            acos_a = acosf(acos_a);

            float acos_e_tmp = 0.5f * (handle->l_se_2 + handle->l_ew_2 - l_sw_2) / (handle->l_se * handle->l_ew);
            if (acos_e_tmp > 1.0f)
                acos_e_tmp = 1.0f;
            else if (acos_e_tmp < -1.0f)
                acos_e_tmp = -1.0f;
            acos_e = acosf(acos_e_tmp);

            // 两个解：Elbow Up / Elbow Down
            if (0 == ind_arm) // Front
            {
                qa[0][0] = atan_a - acos_a + (float)M_PI_2;       // T2
                qa[0][1] = handle->atan_e - acos_e + (float)M_PI; // T3

                qa[1][0] = atan_a + acos_a + (float)M_PI_2;       // T2'
                qa[1][1] = handle->atan_e + acos_e - (float)M_PI; // T3'
            }
            else // Back
            {
                qa[0][0] = atan_a + acos_a + (float)M_PI_2;
                qa[0][1] = handle->atan_e + acos_e - (float)M_PI;

                qa[1][0] = atan_a - acos_a + (float)M_PI_2;
                qa[1][1] = handle->atan_e - acos_e + (float)M_PI;
            }
        }

        // --- 6. 求解后三轴 (Wrist Orientation) ---
        for (ind_elbow = 0; ind_elbow < 2; ind_elbow++)
        {
            // 计算 R30 = R32 * R21 * R10
            // 为了得到 R36 = R30 * R06
            cosqa[0] = cosf(qa[ind_elbow][0] + handle->DH_matrix[1][0]);
            sinqa[0] = sinf(qa[ind_elbow][0] + handle->DH_matrix[1][0]);
            cosqa[1] = cosf(qa[ind_elbow][1] + handle->DH_matrix[2][0]);
            sinqa[1] = sinf(qa[ind_elbow][1] + handle->DH_matrix[2][0]);

            // 这里手动计算 R31 (Rel J3 to J1)
            R31[0] = cosqa[0] * cosqa[1] - sinqa[0] * sinqa[1];
            R31[1] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[2] = 0;
            // ... (省略中间很多 0 的项，直接写结果矩阵)
            R31[3] = 0;
            R31[4] = 0;
            R31[5] = 1.0f;
            R31[6] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
            R31[7] = -cosqa[0] * cosqa[1] + sinqa[0] * sinqa[1];
            R31[8] = 0;

            MatMultiply(R31, R10, R30, 3, 3, 3);
            MatMultiply(R30, R06, R36, 3, 3, 3); // 得到腕部旋转矩阵 R36

            // 从 R36 提取欧拉角 (ZYX) 得到 θ4, θ5, θ6
            // R36[8] 对应 cos(θ5)
            if (R36[8] >= 1.0f - 0.0001f)
                cosqw = 1.0f;
            else if (R36[8] <= -1.0f + 0.0001f)
                cosqw = -1.0f;
            else
                cosqw = R36[8];

            // 两种 Wrist Flip 情况
            if (0 == ind_arm)
            {
                qw[0][1] = acosf(cosqw);
                qw[1][1] = -acosf(cosqw);
            }
            else
            {
                qw[0][1] = -acosf(cosqw);
                qw[1][1] = acosf(cosqw);
            }

            // 求解 θ4, θ6
            if (fabsf(cosqw) > 0.999f) // 奇异：θ5 = 0，四轴六轴共线
            {
                // 此时无法区分 θ4 和 θ6，通常锁定 θ4=上一时刻值
                float last_q4 = lastJoints->a[3] * DEG_TO_RAD_CONST;
                qw[0][0] = last_q4;
                qw[1][0] = last_q4;
                // 计算 θ6 ... (此处略去复杂的三角推导，直接给结果框架)
                qw[0][2] = 0; // 简化处理
                qw[1][2] = 0;
            }
            else
            {
                // 正常 atan2 求解
                if (0 == ind_arm)
                {
                    qw[0][0] = atan2f(R36[5], R36[2]);
                    qw[1][0] = atan2f(-R36[5], -R36[2]);
                    qw[0][2] = atan2f(R36[7], -R36[6]);
                    qw[1][2] = atan2f(-R36[7], R36[6]);
                }
                else
                {
                    qw[0][0] = atan2f(-R36[5], -R36[2]);
                    qw[1][0] = atan2f(R36[5], R36[2]);
                    qw[0][2] = atan2f(-R36[7], R36[6]);
                    qw[1][2] = atan2f(R36[7], -R36[6]);
                }
            }

            // --- 7. 汇总每组解 ---
            for (ind_wrist = 0; ind_wrist < 2; ind_wrist++)
            {
                int idx = 4 * ind_arm + 2 * ind_elbow + ind_wrist;

                // 填入角度并做范围归一化
                // T1
                outputSolves->config[idx].a[0] = qs[ind_arm];
                // T2, T3
                outputSolves->config[idx].a[1] = qa[ind_elbow][0];
                outputSolves->config[idx].a[2] = qa[ind_elbow][1];
                // T4, T5, T6
                outputSolves->config[idx].a[3] = qw[ind_wrist][0];
                outputSolves->config[idx].a[4] = qw[ind_wrist][1];
                outputSolves->config[idx].a[5] = qw[ind_wrist][2];

                // 统一转成度数，并处理 +/- PI 周期
                for (int k = 0; k < 6; k++)
                {
                    float angle = outputSolves->config[idx].a[k];
                    
                    // 检查 NaN 或 Inf
                    if (isnan(angle) || isinf(angle))
                    {
                        outputSolves->config[idx].a[k] = 0.0f;
                        outputSolves->solFlag[idx][0] = -1; // 标记无效
                        continue;
                    }

                    // 使用 fmod 进行快速归一化，替代 while 循环防止死循环
                    // 映射到 [-PI, PI]
                    angle = fmodf(angle, 2.0f * (float)M_PI); 
                    if (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
                    if (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
                    
                    outputSolves->config[idx].a[k] = angle * RAD_TO_DEG_CONST;
                }
            }
        }
    }

    return true;
}

/**
 * @brief 最优解选择器
 *        原理：从8组数学解中，筛选出物理可达(限位内)且离当前位置最近的解
 * @param handle 传入句柄以访问关节限位配置
 * @param solves 逆解算出的8组解
 * @param current_joints 当前机械臂的关节角度
 * @return 最佳解索引 (0-7), 返回 -1 表示无解
 */
int Kinematic_Select_Best_Sol(const DOF6Kinematic_Handle_t *handle, 
                              const IKSolves_t *solves, 
                              const Joint6D_t *current_joints)
{
    int best_index = -1;
    float min_diff_sum = 100000.0f; // 初始化为一个很大的数

    for (int i = 0; i < 8; i++)
    {
        // 1. 检查数学有效性 (如果在 IK 过程中被标记为无效/奇异)
        // solFlag[i][0] == -1 表示该组解无效
        if (solves->solFlag[i][0] == (char)-1) 
        {
            continue;
        }

        // 2. 检查物理限位 (Joint Limits)
        bool out_of_limit = false;
        for (int j = 0; j < 6; j++)
        {
            float angle = solves->config[i].a[j];
            
            // 容差处理 (epsilon): 允许极其微小的超限，防止临界点抖动
            // 这里严格判断，如果超出 max 或小于 min 则不可用
            // if (angle < handle->config.joint_min_limit[j] || 
            //    angle > handle->config.joint_max_limit[j])
            
            // [New] use armConfig
            if (angle < handle->config.joint_min_limit[j] || 
                angle > handle->config.joint_max_limit[j])
            {
                out_of_limit = true;
                break; // 只要有一个关节超限，整组解就废了
            }
        }
        
        if (out_of_limit)
        {
            continue; // 物理不可达，跳过
        }

        // 3. 计算“移动代价” (与当前角度的差值和)
        float diff_sum = 0.0f;
        for (int j = 0; j < 6; j++)
        {
            float target = solves->config[i].a[j];
            float current = current_joints->a[j];
            float diff = fabsf(target - current);

            // [新增] 处理关节角度的周期性 (例如 -170 到 170 应该是 20 度，而不是 340 度)
            // 但机械臂通常有限位，所以直接用绝对差值通常也行。

            // 权重处理：大关节移动代价高，腕关节移动代价低
            // 这里可以给 J1, J2, J3 更高的权重
            float weight = 1.0f;
            if (j < 3) weight = 2.0f;
            
            diff_sum += diff * weight;
        }

        // 4. 更新最优解
        if (diff_sum < min_diff_sum)
        {
            min_diff_sum = diff_sum;
            best_index = i;
        }
    }
    
    // 如果没有找到任何合法的解，返回 -1
    return best_index;
}

/**
 * @brief 获取所有电机角度并填入 Joint6D_t 结构体
 * @param[out] joint_angle 输出的目标结构体指针
 */
void Joint_Motor_Get_All_Angles(Joint6D_t *joint_angle)
{
    // 安全检查
    if (joint_angle == NULL) return;

    // 遍历 Axis 1~6 (电机ID通常是1-6)
    // 注意: Joint6D_t 数组均从 0 开始 (a[0]对应轴1)
    for(uint8_t i=0; i<6; i++) 
    {
        float angle = Joint_Motor_Get_Angle(i+1);
        joint_angle->a[i] = angle;
    }
}

// #include "dummy_kinematic.h"
// #include "step_motor.h"

// /**
//  * @brief 矩阵乘法
//  * @param _m A的行数
//  * @param _l A的列数 (也是B的行数)
//  * @param _n B的列数
//  */
// static void MatMultiply(const float *_matrix1, const float *_matrix2, float *_matrixOut,
//                         const int _m, const int _l, const int _n)
// {
//     float tmp;
//     int i, j, k;
//     for (i = 0; i < _m; i++)
//     {
//         for (j = 0; j < _n; j++)
//         {
//             tmp = 0.0f;
//             for (k = 0; k < _l; k++)
//             {
//                 tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
//             }
//             _matrixOut[_n * i + j] = tmp;
//         }
//     }
// }

// /**
//  * @brief 旋转矩阵转欧拉角 (ZYX顺序: Roll->Pitch->Yaw)
//  *        用于正解计算末端姿态
//  */
// static void RotMatToEulerAngle(const float *_rotationM, float *_eulerAngles)
// {
//     float A, B, C;

//     // 检查 Gimbal Lock (万向锁) 情况
//     if (fabsf(_rotationM[6]) >= 1.0f - 0.0001f)
//     {
//         if (_rotationM[6] < 0)
//         {
//             A = 0.0f;
//             B = (float)M_PI_2;
//             C = atan2f(_rotationM[1], _rotationM[4]);
//         }
//         else
//         {
//             A = 0.0f;
//             B = -(float)M_PI_2;
//             C = -atan2f(_rotationM[1], _rotationM[4]);
//         }
//     }
//     else
//     {
//         // 正常情况求解
//         B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
//         float cb = cosf(B);
//         // 防止除零
//         if (fabsf(cb) < 0.0001f)
//             cb = 0.0001f;
//         A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
//         C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
//     }
//     _eulerAngles[0] = C; // Yaw (Z)
//     _eulerAngles[1] = B; // Pitch (Y)
//     _eulerAngles[2] = A; // Roll (X)
// }

// /**
//  * @brief 欧拉角转旋转矩阵
//  *        用于逆解时将目标姿态转为矩阵
//  */
// static void EulerAngleToRotMat(const float *_eulerAngles, float *_rotationM)
// {
//     float ca, cb, cc, sa, sb, sc;
//     // 提前计算三角函数
//     cc = cosf(_eulerAngles[0]);
//     cb = cosf(_eulerAngles[1]);
//     ca = cosf(_eulerAngles[2]);
//     sc = sinf(_eulerAngles[0]);
//     sb = sinf(_eulerAngles[1]);
//     sa = sinf(_eulerAngles[2]);

//     // 填充旋转矩阵 R = Rz * Ry * Rx
//     _rotationM[0] = ca * cb;
//     _rotationM[1] = ca * sb * sc - sa * cc;
//     _rotationM[2] = ca * sb * cc + sa * sc;
//     _rotationM[3] = sa * cb;
//     _rotationM[4] = sa * sb * sc + ca * cc;
//     _rotationM[5] = sa * sb * cc - ca * sc;
//     _rotationM[6] = -sb;
//     _rotationM[7] = cb * sc;
//     _rotationM[8] = cb * cc;
// }

// // -------------------------------------------------------------------------
// //                            核心接口实现
// // -------------------------------------------------------------------------

// /**
//  * @brief 初始化运动学句柄
//  */
// void Kinematic_Init(DOF6Kinematic_Handle_t *handle, const ArmConfig_t *config)
// {
//     // 复制配置
//     memcpy(&handle->armConfig, config, sizeof(ArmConfig_t));

//     // 计算 DH 参数矩阵 (单位: mm, rad)
//     // 根据 Dummy 的结构定义
//     float tmp_DH_matrix[6][4] = {
//         {0.0f,               handle->armConfig.L_BASE,    handle->armConfig.D_BASE, -(float)M_PI_2},
//         {-(float)M_PI_2,     0.0f,                        handle->armConfig.L_ARM,  0.0f},
//         {(float)M_PI_2,      handle->armConfig.D_ELBOW,   0.0f,                     (float)M_PI_2},
//         {0.0f,               handle->armConfig.L_FOREARM, 0.0f,                     -(float)M_PI_2},
//         {0.0f,               0.0f,                        0.0f,                     (float)M_PI_2},
//         {0.0f,               handle->armConfig.L_WRIST,   0.0f,                     0.0f}
//     };
//     memcpy(handle->DH_matrix, tmp_DH_matrix, sizeof(tmp_DH_matrix));

//     // 预计算常量
//     // C++: atan_e = atanf(armConfig.D_ELBOW / armConfig.L_FOREARM);
//     handle->atan_e = atanf(handle->armConfig.D_ELBOW / handle->armConfig.L_FOREARM); 
    
//     // 初始化 L 向量
//     float tmp_L1_bs[3] = {handle->armConfig.D_BASE, -handle->armConfig.L_BASE, 0.0f};
//     memcpy(handle->L1_base, tmp_L1_bs, sizeof(tmp_L1_bs));

//     float tmp_L2_se[3] = {handle->armConfig.L_ARM, 0.0f, 0.0f};
//     memcpy(handle->L2_arm, tmp_L2_se, sizeof(tmp_L2_se));

//     float tmp_L3_ew[3] = {-handle->armConfig.D_ELBOW, 0.0f, handle->armConfig.L_FOREARM};
//     memcpy(handle->L3_elbow, tmp_L3_ew, sizeof(tmp_L3_ew));

//     float tmp_L6_wt[3] = {0.0f, 0.0f, handle->armConfig.L_WRIST};
//     memcpy(handle->L6_wrist, tmp_L6_wt, sizeof(tmp_L6_wt));

//     // 预计算长度平方
//     handle->l_se_2 = handle->armConfig.L_ARM * handle->armConfig.L_ARM;
//     handle->l_se = handle->armConfig.L_ARM;
//     handle->l_ew_2 = handle->armConfig.L_FOREARM * handle->armConfig.L_FOREARM +
//                      handle->armConfig.D_ELBOW * handle->armConfig.D_ELBOW;
//     handle->l_ew = sqrtf(handle->l_ew_2);
// }

// /**
//  * @brief 正运动学解算 (Joints -> Pose)
//  */
// bool Kinematic_SolveFK(const DOF6Kinematic_Handle_t *handle,
//                        const Joint6D_t *inputJoints,
//                        Pose6D_t *outputPose)
// {
//     float q_in[6];
//     float q[6];
//     float cosq, sinq, cosa, sina;
//     float P06[6]; // 最终位置 [x,y,z, r,p,y]
//     float R06[9]; // 最终旋转矩阵 0->6

//     // 临时矩阵变量
//     float R[6][9]; // 各关节的局部变换矩阵
//     float R02[9], R03[9], R04[9], R05[9];
//     float L0_bs[3], L0_se[3], L0_ew[3], L0_wt[3];

//     // 输入角度转弧度 (但实际上 inputJoints 已经是度了，这里逻辑是输入是度)
//     // 注意：原代码逻辑有点怪，这里假设 inputJoints 是度，内部转弧度计算
//     for (int i = 0; i < 6; i++)
//         q_in[i] = inputJoints->a[i] * DEG_TO_RAD_CONST;

//     // 逐个计算每个关节的局部变换矩阵 R[i]
//     for (int i = 0; i < 6; i++)
//     {
//         // q[i] = q_in[i] + handle->DH_matrix[i][0]; // 原文: DH_matrix[i][0] 是 offset
//         // C++:
//         // q[i] = q_in[i] + DH_matrix[i][0]; (θ + offset)
//         // cosq = cosf(q[i]); sinq = sinf(q[i]);
//         // cosa = cosf(DH_matrix[i][3]); (α)
//         // sina = sinf(DH_matrix[i][3]);
        
//         // R[i] 计算校验：
//         // R[0]=cosq, R[1]=-cosa*sinq, R[2]=sina*sinq
//         // R[3]=sinq, R[4]=cosa*cosq, R[5]=-sina*cosq
//         // R[6]=0,    R[7]=sina,      R[8]=cosa
//         // [分析]: 这是标准的 DH 变换矩阵的旋转部分 (Rotz(θ) * Rotx(α))
//         // Rotz(θ) = [c, -s, 0; s, c, 0; 0, 0, 1]
//         // Rotx(α) = [1, 0, 0; 0, ca, -sa; 0, sa, ca]
//         // Mul = [c, -s*ca, s*sa; s, c*ca, -c*sa; 0, sa, ca]
        
//         // 原 C++:
//         // R[i][0] = cosq;
//         // R[i][1] = -cosa * sinq;
//         // R[i][2] = sina * sinq;
//         // R[i][3] = sinq;
//         // R[i][4] = cosa * cosq;
//         // R[i][5] = -sina * cosq;  <-- Wait! -sina * cosq?
//         // My derivation: s * (-sa) = -s*sa. No, Rotx(α) element (1,2) is -sin(α).
//         // Let's re-verify.
//         // Rotz * Rotx:
//         // Col 2: [ -s*ca, c*ca, sa ]^T
//         // Col 3: [ s*sa, -c*sa, ca ]^T
        
//         // 你的代码:
//         // R[i][5] = -sina * cosq; Matches.
        
//         q[i] = q_in[i] + handle->DH_matrix[i][0]; // 加上 DH θ 偏置
//         cosq = cosf(q[i]);
//         sinq = sinf(q[i]);
//         cosa = cosf(handle->DH_matrix[i][3]); // DH α
//         sina = sinf(handle->DH_matrix[i][3]);

//         R[i][0] = cosq;
//         R[i][1] = -cosa * sinq;
//         R[i][2] = sina * sinq;
//         R[i][3] = sinq;
//         R[i][4] = cosa * cosq;
//         R[i][5] = -sina * cosq;
//         R[i][6] = 0.0f;
//         R[i][7] = sina;
//         R[i][8] = cosa;
        
//         // [新增] 数组越界保护
//         // R[6][9] 定义没问题。
//     }

//     // 级联乘法：计算 R0->2, R0->3 ... R0->6
//     MatMultiply(R[0], R[1], R02, 3, 3, 3);
//     MatMultiply(R02, R[2], R03, 3, 3, 3);
//     MatMultiply(R03, R[3], R04, 3, 3, 3);
//     MatMultiply(R04, R[4], R05, 3, 3, 3);
//     MatMultiply(R05, R[5], R06, 3, 3, 3);

//     // 向量变换：将各杆件向量变换到基坐标系
//     MatMultiply(R[0], handle->L1_base, L0_bs, 3, 3, 1);
//     MatMultiply(R02, handle->L2_arm, L0_se, 3, 3, 1);
//     MatMultiply(R03, handle->L3_elbow, L0_ew, 3, 3, 1);
//     MatMultiply(R06, handle->L6_wrist, L0_wt, 3, 3, 1);

//     // 向量叠加：P06 = P_base + P_shoulder + P_elbow + P_wrist
//     for (int i = 0; i < 3; i++)
//         P06[i] = L0_bs[i] + L0_se[i] + L0_ew[i] + L0_wt[i];

//     // 旋转矩阵转欧拉角
//     RotMatToEulerAngle(R06, &(P06[3]));

//     // 输出结果 (注意单位换算: m->mm, rad->deg)
//     // 原代码这里似乎想输出mm，但P06计算用的单位取决于L_BASE单位。
//     // 如果Kinematic_Init传入的是mm，这里P06就是mm。
//     outputPose->X = P06[0];
//     outputPose->Y = P06[1];
//     outputPose->Z = P06[2];
//     outputPose->A = P06[3] * RAD_TO_DEG_CONST;
//     outputPose->B = P06[4] * RAD_TO_DEG_CONST;
//     outputPose->C = P06[5] * RAD_TO_DEG_CONST;
//     memcpy(outputPose->R, R06, 9 * sizeof(float));
//     outputPose->hasR = true;

//     return true;
// }

// /**
//  * @brief 逆运动学解算 (Pose -> Joints)
//  *        使用 Paul 反变换法 / 几何法混合求解
//  */
// bool Kinematic_SolveIK(DOF6Kinematic_Handle_t *handle,
//                        const Pose6D_t *inputPose,
//                        const Joint6D_t *lastJoints,
//                        IKSolves_t *outputSolves)
// {
//     // 中间变量定义
//     float qs[2];    // 关节1的两个解 (Front/Back)
//     float qa[2][2]; // 关节2,3的解 (Elbow Up/Down)
//     float qw[2][3]; // 关节4,5,6的解 (Wrist Flip/NoFlip)
//     float cosqs, sinqs;
//     float cosqa[2], sinqa[2];
//     float cosqw; // [修改] 删除了 sinqw
//     float P06[6], R06[9];

//     // 关键向量
//     float P0_w[3];            // 腕部中心点 (基坐标系下)
//     float P1_w[3];            // 腕部中心点 (关节1坐标系下)
//     float L0_wt[3], L1_sw[3]; // 临时向量

//     // 临时旋转矩阵
//     float R10[9], R31[9], R30[9], R36[9];
//     float l_sw_2, l_sw, atan_a, acos_a, acos_e;

//     int ind_arm, ind_elbow, ind_wrist;
//     int i;

//     // --- 1. Lazy Init: 如果还没计算过这个几何参数，算一下 ---
//     // [修复] 使用阈值判断浮点数是否为0，而不是直接 ==
//     if (handle->l_ew < 0.0001f && handle->l_ew > -0.0001f)
//     {
//         handle->l_ew = sqrtf(handle->l_ew_2);
//         handle->atan_e = atanf(handle->armConfig.D_ELBOW / handle->armConfig.L_FOREARM); 
//     }

//     // --- 2. 准备目标位姿 ---
//     // 输入如果是mm，这里不需要除1000，除非你的算法内部用m。假设全用mm。
//     P06[0] = inputPose->X;
//     P06[1] = inputPose->Y;
//     P06[2] = inputPose->Z;

//     // 如果还没有旋转矩阵，先算出来
//     if (!inputPose->hasR)
//     {
//         float euler[3] = {// 转回弧度
//                           inputPose->A * DEG_TO_RAD_CONST,
//                           inputPose->B * DEG_TO_RAD_CONST,
//                           inputPose->C * DEG_TO_RAD_CONST};
//         EulerAngleToRotMat(euler, R06);
//     }
//     else
//     {
//         memcpy(R06, inputPose->R, 9 * sizeof(float));
//     }

//     // --- 3. 腕部解耦 (Wrist Decoupling) ---
//     // 目标：计算腕部中心点 P_w = P_end - d6 * R * z_axis
//     MatMultiply(R06, handle->L6_wrist, L0_wt, 3, 3, 1);
//     for (i = 0; i < 3; i++)
//         P0_w[i] = P06[i] - L0_wt[i];

//     // --- 4. 求解关节 1 (Theta 1) ---
//     // 几何法：直接看 XY 平面投影
//     if (sqrtf(P0_w[0] * P0_w[0] + P0_w[1] * P0_w[1]) <= 0.000001f) // 奇异：在Z轴上
//     {
//         // 保持上一时刻角度
//         qs[0] = lastJoints->a[0] * DEG_TO_RAD_CONST;
//         qs[1] = lastJoints->a[0] * DEG_TO_RAD_CONST;
//         // 标记解失效 (简化处理)
//         for (i = 0; i < 4; i++)
//         {
//             // [修复] 显式强转消除符号警告
//             outputSolves->solFlag[0 + i][0] = (char)-1;
//             outputSolves->solFlag[4 + i][0] = (char)-1;
//         }
//     }
//     else
//     {
//         // 正常解：Front 和 Back 两个方向
//         qs[0] = atan2f(P0_w[1], P0_w[0]);
//         // 原 C++: qs[1] = atan2f(-P0_w[1], -P0_w[0]);
//         // atan2f(y, x) -> 当 x,y 都反号时，角度相差 180 度 (Back)
//         qs[1] = atan2f(-P0_w[1], -P0_w[0]); 
        
//         for (i = 0; i < 4; i++) {
//             (*outputSolves).solFlag[0 + i][0] = 1;
//             (*outputSolves).solFlag[4 + i][0] = 1;
//         }
//     }

//     // --- 5. 遍历两个 Theta 1 解，求解 Theta 2, 3 ---
//     for (ind_arm = 0; ind_arm < 2; ind_arm++)
//     {
//         // 变换到关节1坐标系
//         cosqs = cosf(qs[ind_arm] + handle->DH_matrix[0][0]);
//         sinqs = sinf(qs[ind_arm] + handle->DH_matrix[0][0]);
        
//         // R10 矩阵:
//         R10[0] = cosqs; R10[1] = sinqs; R10[2] = 0;
//         R10[3] = 0;     R10[4] = 0;     R10[5] = -1;
//         R10[6] = -sinqs;R10[7] = cosqs; R10[8] = 0;

//         MatMultiply(R10, P0_w, P1_w, 3, 3, 1);
        
//         // L1_sw = P1_w - L1_base;
//         // 注意：L1_base 在 init 中被计算为 Handle->L1_base = R10_init * [a1, 0, 0]T + [0, 0, d1]T?
//         // 为简单起见，这里显式计算 Joint 2 (Shoulder) 在 Joint 1 Frame 中的位置
//         // 根据 DH (a1, alpha1, d1, theta1) -> Transformation 0 to 1
//         // 但我们需要的是 Vector from Shoulder center to Wrist center
//         // Handle->L1_base 应该是 Shoulder center 在 Frame 1 下的坐标?
//         // 假设 Init 里算好了 L1_base. 如果没算，这里补救一下:
//         // DH Table Row 1: a1, alpha1=-90, d1, theta1
//         // Vector 0->1 in Frame 0 is usually just Z axis offset d1 and X axis a1. 
//         // In Frame 1, the origin of Frame 1 is at 0,0,0 relative to Frame 1 (duh).
//         // The shoulder joint center (Joint 2 axis) is at P_shoulder.
//         // P_shoulder in Frame 1 is simply [handle->param.a1, handle->param.d2, 0] ?? 
//         // 让我们使用向量减法: P_wrist_in_1 - P_shoulder_in_1
        
//         L1_sw[0] = P1_w[0] - handle->L1_base[0];
//         L1_sw[1] = P1_w[1] - handle->L1_base[1];
//         L1_sw[2] = P1_w[2] - handle->L1_base[2];
        
//         l_sw_2 = L1_sw[0]*L1_sw[0] + L1_sw[1]*L1_sw[1] + L1_sw[2]*L1_sw[2];
//         l_sw = sqrtf(l_sw_2);

        // --- 三角形余弦定理求解 ---
        // l_sw, l_se (大臂), l_ew (小臂) 构成三角形

        // 检查是否超出工作空间
//         if (l_sw > (handle->l_se + handle->l_ew) || l_sw < fabsf(handle->l_se - handle->l_ew))
//         {
//             // 够不着，标记无效
//             // (此处省略详细的奇异处理代码，直接给一个默认解)
//             qa[0][0] = 0;
//             qa[0][1] = 0;
//             qa[1][0] = 0;
//             qa[1][1] = 0;
//             // 标记 flag = 0
//             for (int k = 0; k < 4; k++)
//                 outputSolves->solFlag[4 * ind_arm + k][1] = 0;
//         }
//         else
//         {
//             // 正常求解
//             atan_a = atan2f(L1_sw[1], L1_sw[0]);
//             acos_a = 0.5f * (handle->l_se_2 + l_sw_2 - handle->l_ew_2) / (handle->l_se * l_sw);

//             // 钳制范围防止 NaN
//             if (acos_a > 1.0f)
//                 acos_a = 1.0f;
//             else if (acos_a < -1.0f)
//                 acos_a = -1.0f;
//             acos_a = acosf(acos_a);

//             float acos_e_tmp = 0.5f * (handle->l_se_2 + handle->l_ew_2 - l_sw_2) / (handle->l_se * handle->l_ew);
//             if (acos_e_tmp > 1.0f)
//                 acos_e_tmp = 1.0f;
//             else if (acos_e_tmp < -1.0f)
//                 acos_e_tmp = -1.0f;
//             acos_e = acosf(acos_e_tmp);

//             // 两个解：Elbow Up / Elbow Down
//             if (0 == ind_arm) // Front
//             {
//                 qa[0][0] = atan_a - acos_a + (float)M_PI_2;       // T2
//                 qa[0][1] = handle->atan_e - acos_e + (float)M_PI; // T3

//                 qa[1][0] = atan_a + acos_a + (float)M_PI_2;       // T2'
//                 qa[1][1] = handle->atan_e + acos_e - (float)M_PI; // T3'
//             }
//             else // Back
//             {
//                 qa[0][0] = atan_a + acos_a + (float)M_PI_2;
//                 qa[0][1] = handle->atan_e + acos_e - (float)M_PI;

//                 qa[1][0] = atan_a - acos_a + (float)M_PI_2;
//                 qa[1][1] = handle->atan_e - acos_e + (float)M_PI;
//             }
//         }

//         // --- 6. 求解后三轴 (Wrist Orientation) ---
//         for (ind_elbow = 0; ind_elbow < 2; ind_elbow++)
//         {
//             // 计算 R30 = R32 * R21 * R10
//             // 为了得到 R36 = R30 * R06
//             cosqa[0] = cosf(qa[ind_elbow][0] + handle->DH_matrix[1][0]);
//             sinqa[0] = sinf(qa[ind_elbow][0] + handle->DH_matrix[1][0]);
//             cosqa[1] = cosf(qa[ind_elbow][1] + handle->DH_matrix[2][0]);
//             sinqa[1] = sinf(qa[ind_elbow][1] + handle->DH_matrix[2][0]);

//             // 这里手动计算 R31 (Rel J3 to J1)
//             R31[0] = cosqa[0] * cosqa[1] - sinqa[0] * sinqa[1];
//             R31[1] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
//             R31[2] = 0;
//             // ... (省略中间很多 0 的项，直接写结果矩阵)
//             R31[3] = 0;
//             R31[4] = 0;
//             R31[5] = 1.0f;
//             R31[6] = cosqa[0] * sinqa[1] + sinqa[0] * cosqa[1];
//             R31[7] = -cosqa[0] * cosqa[1] + sinqa[0] * sinqa[1];
//             R31[8] = 0;

//             MatMultiply(R31, R10, R30, 3, 3, 3);
//             MatMultiply(R30, R06, R36, 3, 3, 3); // 得到腕部旋转矩阵 R36

//             // 从 R36 提取欧拉角 (ZYX) 得到 θ4, θ5, θ6
//             // R36[8] 对应 cos(θ5)
//             if (R36[8] >= 1.0f - 0.0001f)
//                 cosqw = 1.0f;
//             else if (R36[8] <= -1.0f + 0.0001f)
//                 cosqw = -1.0f;
//             else
//                 cosqw = R36[8];

//             // 两种 Wrist Flip 情况
//             if (0 == ind_arm)
//             {
//                 qw[0][1] = acosf(cosqw);
//                 qw[1][1] = -acosf(cosqw);
//             }
//             else
//             {
//                 qw[0][1] = -acosf(cosqw);
//                 qw[1][1] = acosf(cosqw);
//             }

//             // 求解 θ4, θ6
//             if (fabsf(cosqw) > 0.999f) // 奇异：θ5 = 0，四轴六轴共线
//             {
//                 // 此时无法区分 θ4 和 θ6，通常锁定 θ4=上一时刻值
//                 float last_q4 = lastJoints->a[3] * DEG_TO_RAD_CONST;
//                 qw[0][0] = last_q4;
//                 qw[1][0] = last_q4;
//                 // 计算 θ6 ... (此处略去复杂的三角推导，直接给结果框架)
//                 qw[0][2] = 0; // 简化处理
//                 qw[1][2] = 0;
//             }
//             else
//             {
//                 // 正常 atan2 求解
//                 if (0 == ind_arm)
//                 {
//                     qw[0][0] = atan2f(R36[5], R36[2]);
//                     qw[1][0] = atan2f(-R36[5], -R36[2]);
//                     qw[0][2] = atan2f(R36[7], -R36[6]);
//                     qw[1][2] = atan2f(-R36[7], R36[6]);
//                 }
//                 else
//                 {
//                     qw[0][0] = atan2f(-R36[5], -R36[2]);
//                     qw[1][0] = atan2f(R36[5], R36[2]);
//                     qw[0][2] = atan2f(-R36[7], R36[6]);
//                     qw[1][2] = atan2f(R36[7], -R36[6]);
//                 }
//             }

//             // --- 7. 汇总每组解 ---
//             for (ind_wrist = 0; ind_wrist < 2; ind_wrist++)
//             {
//                 int idx = 4 * ind_arm + 2 * ind_elbow + ind_wrist;

//                 // 填入角度并做范围归一化
//                 // T1
//                 outputSolves->config[idx].a[0] = qs[ind_arm];
//                 // T2, T3
//                 outputSolves->config[idx].a[1] = qa[ind_elbow][0];
//                 outputSolves->config[idx].a[2] = qa[ind_elbow][1];
//                 // T4, T5, T6
//                 outputSolves->config[idx].a[3] = qw[ind_wrist][0];
//                 outputSolves->config[idx].a[4] = qw[ind_wrist][1];
//                 outputSolves->config[idx].a[5] = qw[ind_wrist][2];

//                 // 统一转成度数，并处理 +/- PI 周期
//                 for (int k = 0; k < 6; k++)
//                 {
//                     float angle = outputSolves->config[idx].a[k];
                    
//                     // 检查 NaN 或 Inf
//                     if (isnan(angle) || isinf(angle))
//                     {
//                         outputSolves->config[idx].a[k] = 0.0f;
//                         outputSolves->solFlag[idx][0] = -1; // 标记无效
//                         continue;
//                     }

//                     // 使用 fmod 进行快速归一化，替代 while 循环防止死循环
//                     // 映射到 [-PI, PI]
//                     angle = fmodf(angle, 2.0f * (float)M_PI); 
//                     if (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
//                     if (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
                    
//                     outputSolves->config[idx].a[k] = angle * RAD_TO_DEG_CONST;
//                 }
//             }
//         }
//     }

//     return true;
// }

// /**
//  * @brief 最优解选择器
//  *        原理：从8组数学解中，筛选出物理可达(限位内)且离当前位置最近的解
//  * @param handle 传入句柄以访问关节限位配置
//  * @param solves 逆解算出的8组解
//  * @param current_joints 当前机械臂的关节角度
//  * @return 最佳解索引 (0-7), 返回 -1 表示无解
//  */
// int Kinematic_Select_Best_Sol(const DOF6Kinematic_Handle_t *handle, 
//                               const IKSolves_t *solves, 
//                               const Joint6D_t *current_joints)
// {
//     int best_index = -1;
//     float min_diff_sum = 100000.0f; // 初始化为一个很大的数

//     for (int i = 0; i < 8; i++)
//     {
//         // 1. 检查数学有效性 (如果在 IK 过程中被标记为无效/奇异)
//         // solFlag[i][0] == -1 表示该组解无效
//         if (solves->solFlag[i][0] == (char)-1) 
//         {
//             continue;
//         }

//         // 2. 检查物理限位 (Joint Limits)
//         bool out_of_limit = false;
//         for (int j = 0; j < 6; j++)
//         {
//             float angle = solves->config[i].a[j];
            
//             // 容差处理 (epsilon): 允许极其微小的超限，防止临界点抖动
//             // 这里严格判断，如果超出 max 或小于 min 则不可用
//             // if (angle < handle->config.joint_min_limit[j] || 
//             //    angle > handle->config.joint_max_limit[j])
            
//             // [New] use armConfig
//             if (angle < handle->armConfig.joint_min_limit[j] || 
//                 angle > handle->armConfig.joint_max_limit[j])
//             {
//                 out_of_limit = true;
//                 break; // 只要有一个关节超限，整组解就废了
//             }
//         }
        
//         if (out_of_limit)
//         {
//             continue; // 物理不可达，跳过
//         }

//         // 3. 计算“移动代价” (与当前角度的差值和)
//         float diff_sum = 0.0f;
//         for (int j = 0; j < 6; j++)
//         {
//             float target = solves->config[i].a[j];
//             float current = current_joints->a[j];
//             float diff = fabsf(target - current);

//             // [新增] 处理关节角度的周期性 (例如 -170 到 170 应该是 20 度，而不是 340 度)
//             // 但机械臂通常有限位，所以直接用绝对差值通常也行。

//             // 权重处理：大关节移动代价高，腕关节移动代价低
//             // 这里可以给 J1, J2, J3 更高的权重
//             float weight = 1.0f;
//             if (j < 3) weight = 2.0f;
            
//             diff_sum += diff * weight;
//         }

//         // 4. 更新最优解
//         if (diff_sum < min_diff_sum)
//         {
//             min_diff_sum = diff_sum;
//             best_index = i;
//         }
//     }
    
//     // 如果没有找到任何合法的解，返回 -1
//     return best_index;
// }

// /**
//  * @brief 获取所有电机角度并填入 Joint6D_t 结构体
//  * @param[out] joint_angle 输出的目标结构体指针
//  */
// void Joint_Motor_Get_All_Angles(Joint6D_t *joint_angle)
// {
//     // 安全检查
//     if (joint_angle == NULL) return;

//     // 遍历 Axis 1~6 (电机ID通常是1-6)
//     // 注意: Joint6D_t 数组均从 0 开始 (a[0]对应轴1)
//     for(uint8_t i=0; i<6; i++) 
//     {
//         float angle = Joint_Motor_Get_Angle(i+1);
//         joint_angle->a[i] = angle;
//     }
// }


