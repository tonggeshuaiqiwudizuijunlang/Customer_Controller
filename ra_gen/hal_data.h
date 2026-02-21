/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_dtc.h"
#include "r_transfer_api.h"
#include "r_sci_uart.h"
            #include "r_uart_api.h"
#include "r_canfd.h"
#include "r_can_api.h"
FSP_HEADER
/* Transfer on DTC Instance. */
extern const transfer_instance_t wifi_dtc_transfer;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t wifi_dtc_transfer_ctrl;
extern const transfer_cfg_t wifi_dtc_transfer_cfg;
/** UART on SCI Instance. */
            extern const uart_instance_t      wifi_uart;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     wifi_uart_ctrl;
            extern const uart_cfg_t wifi_uart_cfg;
            extern const sci_uart_extended_cfg_t wifi_uart_cfg_extend;

            #ifndef UART_Global_Callback
            void UART_Global_Callback(uart_callback_args_t * p_args);
            #endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer0;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer0_ctrl;
extern const transfer_cfg_t g_transfer0_cfg;
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer1;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer1_ctrl;
extern const transfer_cfg_t g_transfer1_cfg;
/** UART on SCI Instance. */
            extern const uart_instance_t      uart5;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     uart5_ctrl;
            extern const uart_cfg_t uart5_cfg;
            extern const sci_uart_extended_cfg_t uart5_cfg_extend;

            #ifndef UART_Global_Callback
            void UART_Global_Callback(uart_callback_args_t * p_args);
            #endif
/* Transfer on DTC Instance. */
extern const transfer_instance_t sbus_dtc_transfer;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t sbus_dtc_transfer_ctrl;
extern const transfer_cfg_t sbus_dtc_transfer_cfg;
/** UART on SCI Instance. */
            extern const uart_instance_t      sbus;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     sbus_ctrl;
            extern const uart_cfg_t sbus_cfg;
            extern const sci_uart_extended_cfg_t sbus_cfg_extend;

            #ifndef UART_Global_Callback
            void UART_Global_Callback(uart_callback_args_t * p_args);
            #endif
/** CANFD on CANFD Instance. */
extern const can_instance_t motor_ctrl_can;
/** Access the CANFD instance using these structures when calling API functions directly (::p_api is not used). */
extern canfd_instance_ctrl_t motor_ctrl_can_ctrl;
extern const can_cfg_t motor_ctrl_can_cfg;
extern const canfd_extended_cfg_t motor_ctrl_can_cfg_extend;

#ifndef can_callback
void can_callback(can_callback_args_t * p_args);
#endif

/* Global configuration (referenced by all instances) */
extern canfd_global_cfg_t g_canfd_global_cfg;
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
