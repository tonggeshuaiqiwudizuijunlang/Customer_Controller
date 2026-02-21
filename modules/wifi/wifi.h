// #ifndef WIFI_H
// #define WIFI_H

// #include "bsp_uart.h"
// #include "bsp_gpio.h"


// /* --- 用户配置区 --- */
// #define WIFI_SSID           "RA_WiFi"
// #define WIFI_PWD            "12345678"
// #define WIFI_SERVER_IP      "192.168.4.1"     // 远端服务器IP (AP模式默认IP)
// #define WIFI_SERVER_PORT    8080

// /* WiFi 模块工作状态 */
// typedef enum {
//     WIFI_STEP_IDLE,         // 空闲
//     WIFI_STEP_HARD_RESET,   // 硬件复位阶段
//     WIFI_STEP_WAIT_READY,   // 等待模块启动日志(ready)
//     WIFI_STEP_SEND_AT,      // 发送 AT
//     WIFI_STEP_SEND_ECHO,    // 发送 ATE0
//     WIFI_STEP_SET_MODE,     // 发送 CWMODE
//     WIFI_STEP_JOIN_AP,      // 发送 CWJAP
//     WIFI_STEP_START_TCP,    // 发送 CIPSTART
//     WIFI_STEP_CIPMODE,      // 设置透传模式 (可选)
//     WIFI_STEP_CIPSEND,      // 发送进入透传指令
//     WIFI_STEP_DATA_TRANS,   // 数据透传阶段 (最终状态)
//     WIFI_STEP_ERROR         // 错误状态
// } wifi_step_t;


// /* --- API --- */
// void WiFi_Init(void);

// void WiFi_Task_Entry(void); // 放在你的 RTOS 任务While(1)里调用，或者作为单独任务

// bool WiFi_SendData(uint8_t *data, uint16_t len);



// #endif /* WIFI_H */
