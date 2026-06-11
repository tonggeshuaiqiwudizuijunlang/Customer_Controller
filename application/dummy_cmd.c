#include "dummy_cmd.h"
#include "message_center.h"
#include "mc6c.h"
#include "bluetooth.h"
#include "vision.h"

MC_ctrl_t *mc_data;
Publisher_t *dummy_cmd_pub;           // 控制消息发布者
Subscriber_t *dummy_feed_sub;         // 反馈信息订阅者
Dummy_Ctrl_Cmd_s dummy_cmd_send;      // 控制命令缓存
Dummy_Upload_Data_s dummy_fetch_data; // 反馈数据缓存

Transmit_Data_s vision_tx_data;  // 发送给上位机的数据缓存
Received_Data_s *vision_rx_data; // 从上位机接收的数据缓存

RX_BT_Data_s *bt_rx_data;
TX_BT_Data_s bt_tx_data;

void MC_Remote_Ctrl(void);
void Vision_Set_FeedData(void);
static void limit_joint_angles(Dummy_Ctrl_Cmd_s *cmd);

void DummyCmd_Init(void)
{
    DWT_Init(200);
    bt_rx_data = BT_Init(&bt_uart_ctrl, &bt_uart_cfg);
    vision_rx_data = Vision_Init(&pc_uart_ctrl, &pc_uart_cfg);
    mc_data = MCControlInit(&sbus_ctrl, &sbus_cfg);
    dummy_cmd_pub = PubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));
    dummy_feed_sub = SubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
}

void DummyCmd_Task(void)
{
    SubGetMessage(dummy_feed_sub, &dummy_fetch_data);
    MC_Remote_Ctrl();
    Vision_Set_FeedData();
    PubPushMessage(dummy_cmd_pub, (void *)&dummy_cmd_send);
}

void MC_Remote_Ctrl(void)
{

    if (mc_data[TEMP].switch_r == MC_SW_DOWN)
    {
        dummy_cmd_send.arm_mode = ARM_ZERO_FORCE;
    }
    else if (mc_data[TEMP].switch_r == MC_SW_MID)
    {
        dummy_cmd_send.arm_mode = ARM_FREE_MODE;
        
        // 当左侧拨杆状态发生变化时，根据当前状态刷新夹爪命令
        if (mc_data[TEMP].switch_l != mc_data[LAST].switch_l)
        {
            if (mc_data[TEMP].switch_l == MC_SW_DOWN)
            {
                dummy_cmd_send.gripper_mode = GRIPPER_AUTO_GRAB;
            }
            else
            {
                dummy_cmd_send.gripper_mode = GRIPPER_RELEASE; // 其他摇杆状态下松开夹爪
            }
        }

        if (mc_data[TEMP].switch_l == MC_SW_DOWN)
        {
            // 通过摇杆控制发送自动抓取命令给下游处理
        }
        else if (mc_data[TEMP].switch_l == MC_SW_MID)
        {
            dummy_cmd_send.arm_ctrl_mode = SMALL_ARM_CTRL;
            dummy_cmd_send.joint3_angle = map_float_clamp(mc_data[TEMP].rocker_l1, -800, 800, 0, 210);
            dummy_cmd_send.joint4_angle += (float)(0.0003f * mc_data[TEMP].rocker_l_);
            dummy_cmd_send.joint5_angle += (float)(0.0003f * mc_data[TEMP].rocker_r1);
            dummy_cmd_send.joint6_angle += (float)(0.0003f * mc_data[TEMP].rocker_r_);
        }
        else if (mc_data[TEMP].switch_l == MC_SW_UP)
        {
            dummy_cmd_send.arm_ctrl_mode = BIG_ARM_CTRL;
            dummy_cmd_send.joint1_angle += (float)(0.0003f * mc_data[TEMP].rocker_r_);
            dummy_cmd_send.joint2_angle += (float)(0.0003f * mc_data[TEMP].rocker_r1);
            dummy_cmd_send.joint3_angle = map_float_clamp(mc_data[TEMP].rocker_l1, -800, 800, 0, 210);
            dummy_cmd_send.joint4_angle += (float)(0.0003f * mc_data[TEMP].rocker_l_);
        }
    }
    else if (mc_data[TEMP].switch_r == MC_SW_UP)
    {
        dummy_cmd_send.arm_mode = ARM_PC_MODE;
        dummy_cmd_send.joint1_angle = -vision_rx_data->joint1;
        dummy_cmd_send.joint2_angle = -vision_rx_data->joint2 + 75.0f;
        dummy_cmd_send.joint3_angle = -vision_rx_data->joint3 + 90.0f;
        dummy_cmd_send.joint4_angle = vision_rx_data->joint4;
        dummy_cmd_send.joint5_angle = -vision_rx_data->joint5;
        dummy_cmd_send.joint6_angle = vision_rx_data->joint6;
    }

    limit_joint_angles(&dummy_cmd_send);
}

static void limit_joint_angles(Dummy_Ctrl_Cmd_s *cmd)
{
    // 关节角度限幅 (基于 Dummy_Motormatic_Init 中的配置)
    // joint_min_limit: {0, 0, 0,  0,   0,   0}
    // joint_max_limit: {340, 155, 215, 360, 240, 720}

    // if (cmd->joint1_angle < -170.0f)
    //     cmd->joint1_angle = -170.0f;
    // if (cmd->joint1_angle > 170.0f)
    //     cmd->joint1_angle = 170.0f;

    if (cmd->joint2_angle < -30.0f)
        cmd->joint2_angle = -30.0f;
    if (cmd->joint2_angle > 150.0f)
        cmd->joint2_angle = 150.0f;

    if (cmd->joint3_angle < -35.0f)
        cmd->joint3_angle = 35.0f;
    if (cmd->joint3_angle > 180.0f)
        cmd->joint3_angle = 180.0f;

    if (cmd->joint4_angle < -180.0f)
        cmd->joint4_angle = -180.0f;
    if (cmd->joint4_angle > 180.0f)
        cmd->joint4_angle = 180.0f;

    if (cmd->joint5_angle < -120.0f)
        cmd->joint5_angle = -120.0f;
    if (cmd->joint5_angle > 120.0f)
        cmd->joint5_angle = 120.0f;

    if (cmd->joint6_angle < -360.0f)
        cmd->joint6_angle = -360.0f;
    if (cmd->joint6_angle > 360.0f)
        cmd->joint6_angle = 360.0f;
}

void Vision_Set_FeedData(void)
{
    vision_tx_data.header = 0x5A;
    vision_tx_data.joint1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    vision_tx_data.joint2 = dummy_fetch_data.joint_motor[1].reduction_angle - 75.0f;
    vision_tx_data.joint3 = dummy_fetch_data.joint_motor[2].reduction_angle - 90.0f;
    vision_tx_data.joint4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    vision_tx_data.joint5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    vision_tx_data.joint6 = dummy_fetch_data.joint_motor[5].reduction_angle;

    uint8_t all_finished = 1;
    for (int i = 1; i <= 6; i++)
    {
        if (dummy_fetch_data.joint_motor[i].is_finished == 0)
        {
            all_finished = 0;
            break;
        }
    }

    // 根据要求，当所有电机完成时设置为11，否则为0
    vision_tx_data.is_finished = all_finished;
    vision_tx_data.tailer = 0XFFFB;
    Vision_Send_Data((uint8_t *)&vision_tx_data, sizeof(Transmit_Data_s));
}

void Bt_Set_FeedData(void)
{
    bt_tx_data.header = 0xAA;
    bt_tx_data.joint1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    bt_tx_data.joint2 = dummy_fetch_data.joint_motor[1].reduction_angle - 75.0f;
    bt_tx_data.joint3 = dummy_fetch_data.joint_motor[2].reduction_angle - 90.0f;
    bt_tx_data.joint4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    bt_tx_data.joint5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    bt_tx_data.joint6 = dummy_fetch_data.joint_motor[5].reduction_angle;

    uint8_t all_finished = 1;
    for (int i = 1; i <= 6; i++)
    {
        if (dummy_fetch_data.joint_motor[i].is_finished == 0)
        {
            all_finished = 0;
            break;
        }
    }
    // 根据要求，当所有电机完成时设置为11，否则为0
    bt_tx_data.is_finished = all_finished;
    bt_tx_data.tailer = 0XFFFB;
    BT_SendData((uint8_t *)&bt_tx_data, sizeof(TX_BT_Data_s));
}