#include "dummy_cmd.h"
#include "message_center.h"
#include "mc6c.h"

#define MAX_SPEED_XYZ 0.5f
#define MAX_SPEED_YAW 0.3f

 MC_ctrl_t *mc_data;

 Publisher_t *dummy_cmd_pub;           // 控制消息发布者
 Subscriber_t *dummy_feed_sub;         // 反馈信息订阅者
 Dummy_Ctrl_Cmd_s dummy_cmd_send;      // 控制命令缓存
 Dummy_Upload_Data_s dummy_fetch_data; // 反馈数据缓存

 bool is_synced = false;

 bool check_first_run(void)
{

    // 同步初始位置
    dummy_cmd_send.target_pose.X = dummy_fetch_data.cur_x;
    dummy_cmd_send.target_pose.Y = dummy_fetch_data.cur_y;
    dummy_cmd_send.target_pose.Z = dummy_fetch_data.cur_z;
    // 同步 RPY
    dummy_cmd_send.target_pose.A = dummy_fetch_data.cur_roll;
    dummy_cmd_send.target_pose.B = dummy_fetch_data.cur_pitch;
    dummy_cmd_send.target_pose.C = dummy_fetch_data.cur_yaw;
    
    is_synced = true;
	
    return is_synced;

}

void MC_Remote_Ctrl(void)
{
    // [调试] 强制覆盖控制模式，以确定是否是开关档位问题
    // dummy_cmd_send.arm_mode = ARM_FREE_MODE;

    // 右拨下档 -> 遥控器离线/紧急停机
    if (mc_data[TEMP].switch_r == MC_SW_DOWN)
    {
        dummy_cmd_send.arm_mode = ARM_ZERO_FORCE;
        // return; // 移除 return，确保数据始终被发布，即使是停止模式
    }
    else
    {
        // 修正: switch_r 判断逻辑
        if (mc_data[TEMP].switch_r == MC_SW_MID) 
            dummy_cmd_send.arm_mode = ARM_HOME_MODE;
        else if (mc_data[TEMP].switch_r == MC_SW_UP) 
            dummy_cmd_send.arm_mode = ARM_FREE_MODE;
        

        // 修正: switch_l 判断逻辑
        if (mc_data[TEMP].switch_l == MC_SW_MID) 
            dummy_cmd_send.gripper_mode = GRIPPER_AUTO_GRAB;
        else if (mc_data[TEMP].switch_l == MC_SW_UP && dummy_cmd_send.arm_mode == ARM_FREE_MODE) 
            dummy_cmd_send.arm_mode = ARM_PC_MODE; 
        // 3. 只有在工作模式下才处理摇杆积分
        if (dummy_cmd_send.arm_mode == ARM_FREE_MODE)
        {
            // dummy_cmd_send.target_pose.X += (float)mc_data[TEMP].rocker_r1; // 右竖直 (X)
            // // Y轴(左右): 左摇杆横向(lx) -> 左推(+Y), 右推(-Y)
            // dummy_cmd_send.target_pose.Y += (float)mc_data[TEMP].rocker_l_; // 左水平 (Y)
            // // Z轴(上下): 左摇杆纵向(ly) -> 前推(+Z), 后拉(-Z)
            // dummy_cmd_send.target_pose.Z += (float)mc_data[TEMP].rocker_l1;
            dummy_cmd_send.target_pose.X = 0.35f; // 右竖直 (X)
            // Y轴(左右): 左摇杆横向(lx) -> 左推(+Y), 右推(-Y)
            dummy_cmd_send.target_pose.Y = 0.5f; // 左水平 (Y)
            // Z轴(上下): 左摇杆纵向(ly) -> 前推(+Z), 后拉(-Z)
            dummy_cmd_send.target_pose.Z = 0.5f;
            // Arm_Limit();
        }
    }
}

void DummyCmd_Init(void)
{
    DWT_Init(200);
    mc_data = MCControlInit(&sbus_ctrl, &sbus_cfg);
    dummy_cmd_pub = PubRegister("dummy_cmd", sizeof(Dummy_Ctrl_Cmd_s));
    dummy_feed_sub = SubRegister("dummy_feed", sizeof(Dummy_Upload_Data_s));
}

void DummyCmd_Task(void)
{
    if(!check_first_run()) return; // 等待第一次成功获取反馈数据以同步初始位置
    SubGetMessage(dummy_feed_sub, &dummy_fetch_data);
    MC_Remote_Ctrl();
    PubPushMessage(dummy_cmd_pub, (void *)&dummy_cmd_send);
}