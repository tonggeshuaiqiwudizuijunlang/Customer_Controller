/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (17)
        #endif
        /* ISR prototypes */
        void canfd_error_isr(void);
        void canfd_channel_tx_isr(void);
        void canfd_common_fifo_rx_isr(void);
        void canfd_rx_fifo_isr(void);
        void sci_uart_rxi_isr(void);
        void sci_uart_txi_isr(void);
        void sci_uart_tei_isr(void);
        void sci_uart_eri_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_CAN0_CHERR ((IRQn_Type) 0) /* CAN0 CHERR (Channel  error) */
        #define CAN0_CHERR_IRQn          ((IRQn_Type) 0) /* CAN0 CHERR (Channel  error) */
        #define VECTOR_NUMBER_CAN0_TX ((IRQn_Type) 1) /* CAN0 TX (Transmit interrupt) */
        #define CAN0_TX_IRQn          ((IRQn_Type) 1) /* CAN0 TX (Transmit interrupt) */
        #define VECTOR_NUMBER_CAN0_COMFRX ((IRQn_Type) 2) /* CAN0 COMFRX (Common FIFO receive interrupt) */
        #define CAN0_COMFRX_IRQn          ((IRQn_Type) 2) /* CAN0 COMFRX (Common FIFO receive interrupt) */
        #define VECTOR_NUMBER_CAN_GLERR ((IRQn_Type) 3) /* CAN GLERR (Global error) */
        #define CAN_GLERR_IRQn          ((IRQn_Type) 3) /* CAN GLERR (Global error) */
        #define VECTOR_NUMBER_CAN_RXF ((IRQn_Type) 4) /* CAN RXF (Global receive FIFO interrupt) */
        #define CAN_RXF_IRQn          ((IRQn_Type) 4) /* CAN RXF (Global receive FIFO interrupt) */
        #define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 5) /* SCI9 RXI (Receive data full) */
        #define SCI9_RXI_IRQn          ((IRQn_Type) 5) /* SCI9 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 6) /* SCI9 TXI (Transmit data empty) */
        #define SCI9_TXI_IRQn          ((IRQn_Type) 6) /* SCI9 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 7) /* SCI9 TEI (Transmit end) */
        #define SCI9_TEI_IRQn          ((IRQn_Type) 7) /* SCI9 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 8) /* SCI9 ERI (Receive error) */
        #define SCI9_ERI_IRQn          ((IRQn_Type) 8) /* SCI9 ERI (Receive error) */
        #define VECTOR_NUMBER_SCI5_RXI ((IRQn_Type) 9) /* SCI5 RXI (Receive data full) */
        #define SCI5_RXI_IRQn          ((IRQn_Type) 9) /* SCI5 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI5_TXI ((IRQn_Type) 10) /* SCI5 TXI (Transmit data empty) */
        #define SCI5_TXI_IRQn          ((IRQn_Type) 10) /* SCI5 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI5_TEI ((IRQn_Type) 11) /* SCI5 TEI (Transmit end) */
        #define SCI5_TEI_IRQn          ((IRQn_Type) 11) /* SCI5 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI5_ERI ((IRQn_Type) 12) /* SCI5 ERI (Receive error) */
        #define SCI5_ERI_IRQn          ((IRQn_Type) 12) /* SCI5 ERI (Receive error) */
        #define VECTOR_NUMBER_SCI6_RXI ((IRQn_Type) 13) /* SCI6 RXI (Receive data full) */
        #define SCI6_RXI_IRQn          ((IRQn_Type) 13) /* SCI6 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI6_TXI ((IRQn_Type) 14) /* SCI6 TXI (Transmit data empty) */
        #define SCI6_TXI_IRQn          ((IRQn_Type) 14) /* SCI6 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI6_TEI ((IRQn_Type) 15) /* SCI6 TEI (Transmit end) */
        #define SCI6_TEI_IRQn          ((IRQn_Type) 15) /* SCI6 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI6_ERI ((IRQn_Type) 16) /* SCI6 ERI (Receive error) */
        #define SCI6_ERI_IRQn          ((IRQn_Type) 16) /* SCI6 ERI (Receive error) */
        /* The number of entries required for the ICU vector table. */
        #define BSP_ICU_VECTOR_NUM_ENTRIES (17)

        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */