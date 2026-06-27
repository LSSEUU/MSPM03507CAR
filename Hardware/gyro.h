/*
 * gyro.h  —  JY61P 陀螺仪驱动 (UART2, 9600 baud)
 *
 * 硬件: JY61P TX → PA22 (UART2_RX), RX → PA21 (UART2_TX)
 * 数据: 角度输出模式, 每帧 11 字节, 自动发送
 */
#ifndef GYRO_H
#define GYRO_H
#include "board.h"

typedef struct {
    float   yaw;        /* 偏航角 -180~180°, 逆时针为正 */
    float   pitch;      /* 俯仰角 */
    float   roll;       /* 翻滚角 */
    uint8_t updated;    /* 本周期有新数据 (读取后可清零) */
} Gyro_t;

extern Gyro_t Gyro;

void Gyro_Init(void);
void Gyro_Update(void);     /* 每 5ms 调用 */

#endif
