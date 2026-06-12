#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include "bsp_uart.h"

// 保持1字节对齐，防止编译器自动插入填充字节导致结构体大小不匹配
#pragma pack(1)
typedef struct {
    uint8_t header; // 固定为0xAA

    float target_x;     // 末端目标 X，单位 mm
    float target_y;     // 末端目标 Y，单位 mm
    float target_z;     // 末端目标 Z，单位 mm
    float target_roll;  // 末端目标 Roll，单位 deg
    float target_pitch; // 末端目标 Pitch，单位 deg
    float target_yaw;   // 末端目标 Yaw，单位 deg

    uint8_t gripper_state; // 0:关闭 1:打开

    uint16_t tailer; // 固定为0xFFFB
} RX_BT_Data_s;

typedef struct {
    uint8_t header; // 固定为0x5A

    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    float theta6;
    uint8_t is_finished; // 0:未完成 1:已完成

    uint16_t tailer; // 固定为0xFFFB
} TX_BT_Data_s;
#pragma pack()

RX_BT_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg);
bool BT_SendData(uint8_t* tx_data, uint16_t len);

#endif // BLUETOOTH_H