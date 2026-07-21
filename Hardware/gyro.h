/*
 * gyro.h  —  JY61P 陀螺仪驱动 (UART2, 9600 baud)
 *
 * 硬件: JY61P TX → PA22 (UART2_RX); PA21 保持高阻，不启用 UART2_TX
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

typedef struct {
    Gyro_t sample;
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t frame_errors;
    uint8_t recent_count;
    uint8_t recent_bytes[32];
} GyroSnapshot_t;

typedef struct {
    uint32_t pa21_transitions;
    uint32_t pa22_transitions;
    uint8_t pa21_level;
    uint8_t pa22_level;
} GyroPinProbe_t;

extern Gyro_t Gyro;

void Gyro_Init(void);
void Gyro_Update(void);     /* 每 5ms 调用 */
void Gyro_GetSnapshot(GyroSnapshot_t *snapshot);
void Gyro_ProbePins(GyroPinProbe_t *probe);

#endif
