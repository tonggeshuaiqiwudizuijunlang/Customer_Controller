// #include "wifi.h"

// /* 硬件句柄 */
// static USARTInstance *wifi_uart = NULL;

// /* 状态控制 */
// static wifi_step_t current_step = WIFI_STEP_IDLE;
// static bool tx_awaiting_response = false; // true 表示指令已发，正在等 OK

// /*行缓存：用于从 BSP 字节流中组装出一行完整回复 */
// static char line_buffer[128]; 
// static uint8_t line_idx = 0;

// /* --- 内部函数声明 --- */
// static void WiFi_Hardware_Reset(void);
// static void WiFi_Process_Incoming_Byte(uint8_t byte);
// static void WiFi_Parse_Line(char *line);
// static void WiFi_Send_Async(char *cmd);

// /* --- 回调函数 (由 BSP 中断调用) --- */
// /* bsp_uart.c 收到数据后调用这个，我们仅做通知或直接在这里处理也可以 */
// /* 但为了不阻塞中断，建议这里什么都不做，或者只置一个 flag。
//    鉴于你的 bsp 已经有了 ring buffer，我们直接在 Task 里的 while 循环读 buffer 即可。
//    不需要这个 callback 实际做事。
// */
// static void WiFi_RxCallback(void) 
// {
//     // 空实现，依靠主循环轮询 BSP 的 Buffer
// }

// /* --- 核心业务循环 (在 RTOS 任务中调用) --- */
// void WiFi_Task_Entry(void)
// {
//     /* 1. 处理接收：从 BSP RingBuffer 捞数据拼凑成行 */
//     // 假设 bsp_uart 提供了读取接口，类似 USART_Read 或 直接访问 ringbuffer
//     // 这里模拟逐字节取出
//     uint8_t ch;
    
//     // 这里的循环是为了把积压在 RingBuffer 的数据处理完
//     while (USART_Read(wifi_uart, &ch, 1) > 0) 
//     {
//         // 拼凑行逻辑
//         if (line_idx < sizeof(line_buffer) - 1) {
//             line_buffer[line_idx++] = (char)ch;
//         }
        
//         // 遇到换行符，认为一行结束，进行解析
//         if (ch == '\n') {
//             line_buffer[line_idx] = '\0'; // 封口
//             WiFi_Parse_Line(line_buffer); // 解析这一行
//             line_idx = 0;                 // 重置下标
//             memset(line_buffer, 0, sizeof(line_buffer));
//         }
//     }

//     /* 2. 处理发送状态机 (仅当没在等回复时执行) */
//     if (tx_awaiting_response == false) 
//     {
//         char temp_cmd[64];
        
//         switch (current_step)
//         {
//             case WIFI_STEP_IDLE:
//                 // 等待外部触发，或者初始化直接进入 Reset
//                 current_step = WIFI_STEP_HARD_RESET;
//                 break;

//             case WIFI_STEP_HARD_RESET:
//                 WiFi_Hardware_Reset();
//                 tx_awaiting_response = true; // 复位后要等 "ready" 或 "jump to run"
//                 // 设置一个初始等待，防止模块不吐 ready 导致死锁
//                 // 实际在 Reset 函数里稍作延时更稳妥，这里略过
//                 break;
                
//             case WIFI_STEP_SEND_AT:
//                 WiFi_Send_Async("AT\r\n");
//                 break;

//             case WIFI_STEP_SEND_ECHO:
//                 WiFi_Send_Async("ATE0\r\n");
//                 break;

//             case WIFI_STEP_SET_MODE:
//                 WiFi_Send_Async("AT+CWMODE=1\r\n");
//                 break;

//             case WIFI_STEP_JOIN_AP:
//                 sprintf(temp_cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
//                 WiFi_Send_Async(temp_cmd);
//                 break;
                
//             case WIFI_STEP_START_TCP:
//                 sprintf(temp_cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", WIFI_SERVER_IP, WIFI_SERVER_PORT);
//                 WiFi_Send_Async(temp_cmd);
//                 break;
                
//             case WIFI_STEP_CIPMODE:
//                 // 开启透传模式
//                 WiFi_Send_Async("AT+CIPMODE=1\r\n");
//                 break;
                
//             case WIFI_STEP_CIPSEND:
//                 // 开始透传指令
//                 WiFi_Send_Async("AT+CIPSEND\r\n");
//                 // 这一步比较特殊，期望回复是 ">" 而不是 "OK"
//                 break;

//             case WIFI_STEP_DATA_TRANS:
//                 // 已经建立透传，这里不做指令维护，直接在 SendData 接口发数据
//                 break;
                
//             case WIFI_STEP_ERROR:
//                 // 错误处理，可以尝试重置 step 到 RESET
//                 break;
                
//             default:
//                 break;
//         }
//     }
// }

// /* --- 解析接收到的行，更新状态机 --- */
// static void WiFi_Parse_Line(char *line)
// {
//     // 1. 如果还在 reset 阶段，等待 ready
//     if (current_step == WIFI_STEP_HARD_RESET || current_step == WIFI_STEP_WAIT_READY) {
//         if (strstr(line, "ready") || strstr(line, "jump to run")) {
//             tx_awaiting_response = false;
//             current_step = WIFI_STEP_SEND_AT; // 复位完成，下一步
//         }
//         return;
//     }
    
//     // 2. 特殊阶段：等待透传符号 ">"
//     if (current_step == WIFI_STEP_CIPSEND) {
//         if (strstr(line, ">")) {
//             tx_awaiting_response = false;
//             current_step = WIFI_STEP_DATA_TRANS; // 进入透传状态，完成所有初始化
//         }
//         return;
//     }

//     // 3. 通用指令响应 "OK"
//     if (strstr(line, "OK")) {
//         if (tx_awaiting_response) {
//             tx_awaiting_response = false; // 解除等待，允许主循环发下一条
            
//             // 状态流转 (既然收到 OK 了，去下一步)
//             // 注意：STEP_DATA_TRANS 状态下收到 OK 可能是发包确认，不要乱转
//             if (current_step < WIFI_STEP_DATA_TRANS) {
//                 current_step++; // 简单递增，前提枚举顺序要对
//             }
//         }
//     }
    
//     // 4. 错误处理 "ERROR" / "FAIL"
//     else if (strstr(line, "ERROR") || strstr(line, "FAIL")) {
//         tx_awaiting_response = false;
//         // 策略：重试当前步 或者 跳到 error
//         current_step = WIFI_STEP_ERROR; 
//     }
// }

// /* --- 硬件动作 --- */

// static void WiFi_Send_Async(char *cmd)
// {
//     // 调用 BSP 的非阻塞发送
//     R_SCI_UART_Write(wifi_uart->p_ctrl, (uint8_t *)cmd, strlen(cmd));
//     tx_awaiting_response = true; // 锁住发送，直到收到 Parse_Line 解锁
// }

// static void WiFi_Hardware_Reset(void)
// {
//     R_BSP_PinWrite(WIFI_PIN_RESET, BSP_IO_LEVEL_LOW);
//     // 这里不得不稍微阻塞一下因为脉冲太短，但相对于整个任务周期可以忽略
//     R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS); 
//     R_BSP_PinWrite(WIFI_PIN_RESET, BSP_IO_LEVEL_HIGH);
// }

// static void WiFi_GPIO_Init(void)
// {
//     R_BSP_PinWrite(WIFI_PIN_BOOT, BSP_IO_LEVEL_HIGH);
//     R_BSP_PinWrite(WIFI_PIN_WAKE, BSP_IO_LEVEL_HIGH);
//     R_BSP_PinWrite(WIFI_PIN_RESET, BSP_IO_LEVEL_HIGH);
// }

// /* --- API 实现 --- */

// void WiFi_Init(void)
// {
//     WiFi_GPIO_Init();
    
//     USART_Init_Config_s config;
//     config.p_uart_ctrl = &wifi_uart_ctrl;
//     config.p_uart_cfg  = &wifi_uart_cfg;
//     config.recv_buff_size = 1; // 1字节中断
//     config.module_callback = WiFi_RxCallback; 
    
//     wifi_uart = USARTRegister(&config);
    
//     current_step = WIFI_STEP_IDLE;
// }

// bool WiFi_SendData(uint8_t *data, uint16_t len)
// {
//     // 只有在透传模式下才允许直接发送
//     if (current_step != WIFI_STEP_DATA_TRANS) return false;
    
//     // 这里最好检查一下 UART 是否 TxBusy，防止 RTOS 抢占导致数据错乱
//     // if (wifi_uart->tx_busy) return false;
    
//     R_SCI_UART_Write(wifi_uart->p_ctrl, data, len);
//     return true;
// }