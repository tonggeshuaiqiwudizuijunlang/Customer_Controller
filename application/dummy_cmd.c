#include "dummy_cmd.h"
#include "message_center.h"
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
#include "serial_debug.h"
#else
#include "bluetooth.h"
#endif
#include "vision.h"
#include "bsp_dwt.h"

Publisher_t *dummy_cmd_pub;
Subscriber_t *dummy_feed_sub;
Dummy_Ctrl_Cmd_s dummy_cmd_send;
Dummy_Upload_Data_s dummy_fetch_data;

Transmit_Data_s vision_tx_data;
Received_Data_s *vision_rx_data;

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
static SerialDebug_Instance_s *serial_debug_instance;
#else
static TX_BT_Data_s bt_tx_data;
#endif

static void DummyCmd_Set_Default_Command(void);
static void Vision_Set_FeedData(void);
#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
static void DummyCmd_Serial_Debug_Send(void);
#else
static void Bt_Set_FeedData(void);
#endif

void DummyCmd_Init(void)
{
    DWT_Init(200);

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
    SerialDebug_Init_Config_s serial_debug_config = {
        .usart_config = {
            .p_uart_ctrl = &bt_uart_ctrl,
            .p_uart_cfg = &bt_uart_cfg,
        },
        .pid_callback = NULL,
    };
    serial_debug_instance = SerialDebug_Init(&serial_debug_config);
#else
    (void)BT_Init(&bt_uart_ctrl, &bt_uart_cfg);
#endif

    vision_rx_data = Vision_Init(&pc_uart_ctrl, &pc_uart_cfg);
    dummy_cmd_pub = PubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));
    dummy_feed_sub = SubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
    DummyCmd_Set_Default_Command();
}

void DummyCmd_Task(void)
{
    SubGetMessage(dummy_feed_sub, &dummy_fetch_data);
    DummyCmd_Set_Default_Command();
    Vision_Set_FeedData();

#if (DUMMY_CMD_UART_MODE == DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
    DummyCmd_Serial_Debug_Send();
#else
    Bt_Set_FeedData();
#endif

    PubPushMessage(dummy_cmd_pub, (void *)&dummy_cmd_send);
}

static void DummyCmd_Set_Default_Command(void)
{
    dummy_cmd_send.arm_mode = ARM_FREE_MODE;
    dummy_cmd_send.arm_ctrl_mode = BIG_ARM_CTRL;
    dummy_cmd_send.gripper_mode = GRIPPER_RELEASE;
}

static void Vision_Set_FeedData(void)
{
    vision_tx_data.header = 0x5A;
    vision_tx_data.joint1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    vision_tx_data.joint2 = dummy_fetch_data.joint_motor[1].reduction_angle - 75.0f;
    vision_tx_data.joint3 = dummy_fetch_data.joint_motor[2].reduction_angle - 90.0f;
    vision_tx_data.joint4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    vision_tx_data.joint5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    vision_tx_data.joint6 = dummy_fetch_data.joint_motor[5].reduction_angle;

    uint8_t all_finished = 1U;
    for (uint8_t i = 0U; i < 6U; i++)
    {
        if (dummy_fetch_data.joint_motor[i].is_finished == 0U)
        {
            all_finished = 0U;
            break;
        }
    }

    vision_tx_data.is_finished = all_finished;
    vision_tx_data.tailer = 0XFFFB;
    Vision_Send_Data((uint8_t *)&vision_tx_data, sizeof(Transmit_Data_s));
}

#if (DUMMY_CMD_UART_MODE != DUMMY_CMD_UART_MODE_SERIAL_DEBUG)
static void Bt_Set_FeedData(void)
{
    bt_tx_data.header = 0x5A;
#if (BT_TX_PACKET_MODE == BT_TX_MODE_POSE)
    bt_tx_data.x = dummy_fetch_data.cur_x;
    bt_tx_data.y = dummy_fetch_data.cur_y;
    bt_tx_data.z = dummy_fetch_data.cur_z;
    bt_tx_data.roll = dummy_fetch_data.cur_roll;
    bt_tx_data.pitch = dummy_fetch_data.cur_pitch;
    bt_tx_data.yaw = dummy_fetch_data.cur_yaw;
#else
    bt_tx_data.theta1 = dummy_fetch_data.joint_motor[0].reduction_angle;
    bt_tx_data.theta2 = dummy_fetch_data.joint_motor[1].reduction_angle;
    bt_tx_data.theta3 = dummy_fetch_data.joint_motor[2].reduction_angle;
    bt_tx_data.theta4 = dummy_fetch_data.joint_motor[3].reduction_angle;
    bt_tx_data.theta5 = dummy_fetch_data.joint_motor[4].reduction_angle;
    bt_tx_data.theta6 = dummy_fetch_data.joint_motor[5].reduction_angle;
#endif

    uint8_t all_finished = 1U;
    for (uint8_t i = 0U; i < 6U; i++)
    {
        if (dummy_fetch_data.joint_motor[i].is_finished == 0U)
        {
            all_finished = 0U;
            break;
        }
    }

    bt_tx_data.is_finished = all_finished;
    bt_tx_data.tailer = 0XFFFB;
    BT_SendData((uint8_t *)&bt_tx_data, sizeof(TX_BT_Data_s));
}
#else
static void DummyCmd_Serial_Debug_Send(void)
{
    static uint16_t send_div_count = 0U;

    send_div_count++;
    if (send_div_count < SERIAL_DEBUG_SEND_DIV)
        return;
    send_div_count = 0U;

    float channels[DUMMY_CMD_SERIAL_DEBUG_CHANNELS] = {
        dummy_fetch_data.controller_motor_angle[0],
        dummy_fetch_data.controller_motor_angle[1],
        dummy_fetch_data.controller_motor_angle[2],
        dummy_fetch_data.imu_euler[0],
        dummy_fetch_data.imu_euler[1],
        dummy_fetch_data.imu_euler[2],
        dummy_fetch_data.cur_x,
        dummy_fetch_data.cur_y,
        dummy_fetch_data.cur_z,
        dummy_fetch_data.cur_roll,
        dummy_fetch_data.cur_pitch,
        dummy_fetch_data.cur_yaw,
    };

    SerialDebug_Send_JustFloat(serial_debug_instance, channels, DUMMY_CMD_SERIAL_DEBUG_CHANNELS);
}
#endif
