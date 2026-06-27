/*
 * uart_callback.h  —  串口通信 (蓝牙/UART 协议解析)
 * 参考: WHEELTEC_C07A_Bluetooth/Control/uart_callback.h
 *
 * 架构: UART1_RX → DMA(CH0) → gBTPacket[200] → BTBufferHandler() → bt_protocol()
 */
#ifndef UART_CALLBACK_H
#define UART_CALLBACK_H
#include "board.h"

/* ── 蓝牙控制标志位 ── */
extern int Flag_Left, Flag_Right, Flag_Direction, Turn_Flag;

/* ── API ── */
void BT_DMAConfig(void);          /* 配置 DMA 接收 */
void BTBufferHandler(void);       /* 主循环中轮询处理 DMA 缓冲区 */
void bt_protocol(uint8_t byte);   /* 逐字节协议解析 */

#endif
