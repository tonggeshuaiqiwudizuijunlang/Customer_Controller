#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include "bsp_uart.h"

// 保持1字节对齐，防止编译器自动插入填充字节导致结构体大小不匹配
#pragma pack(1)
typedef struct {
    uint8_t header; // 固定为0xAA

    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;

    uint8_t gripper_state; // 0:关闭 1:打开

    uint16_t tailer; // 固定为0xFFFB   
} RX_BT_Data_s;

typedef struct {
    uint8_t header; // 固定为0x5A

    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
    uint8_t is_finished; // 0:未完成 1:已完成

    uint16_t tailer; // 固定为0xFFFB   
} TX_BT_Data_s;
#pragma pack()

RX_BT_Data_s* BT_Init(uart_ctrl_t *p_ctrl, uart_cfg_t const *p_cfg);
bool BT_SendData(uint8_t* tx_data, uint16_t len);

#endif // BLUETOOTH_H