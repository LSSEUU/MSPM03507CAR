#include "board.h"
#include "control.h"
#include <math.h>

#define MOVE_SPEED        400
#define MOVE_DISTANCE     1000
#define TURN_KP           20.0f
#define TURN_KI           0.3f
#define TURN_KD           35.0f
#define TURN_LIMIT        650.0f
#define TURN_TOLERANCE    1.5f
#define TURN_STABLE_CNT   15

enum { ST_IDLE, ST_MOVING, ST_PAUSE, ST_TURNING, ST_SETTLE, ST_DONE };
static int state   = ST_IDLE;
static int step    = 0;
static bool turn_active = false;
static float turn_target = 0.0f;
static int   turn_stable = 0;
static int   pause_tick  = 0;
static float kp = TURN_KP, ki = TURN_KI, kd = TURN_KD, tlimit = TURN_LIMIT;
static PID_HandleTypeDef turn_pid;

static float angle_diff(float t, float c)
{
    float d = t - c;
    while (d > 180.0f) d -= 360.0f;
    while (d <-180.0f) d += 360.0f;
    return d;
}

static void turn_start(float delta)
{
    float t = Gyro.yaw + delta;
    while (t > 180.0f) t -= 360.0f;
    while (t <-180.0f) t += 360.0f;
    turn_target = t;
    turn_stable = 0;
    turn_active = true;
    PID_Init(&turn_pid, kp, ki, kd, -tlimit, tlimit);
}

static void turn_update(void)
{
    if (!turn_active) return;
    float e = angle_diff(turn_target, Gyro.yaw);
    float o = PID_Compute(&turn_pid, 0.0f, e);
    Motor_SetPWM((int16_t)o, -(int16_t)o);
    if (fabsf(e) < TURN_TOLERANCE) {
        if (++turn_stable >= TURN_STABLE_CNT) {
            turn_active = false;
            Motor_Stop();
        }
    } else {
        turn_stable = 0;
    }
}

static void pattern_start(void)
{
    state = ST_MOVING;
    step  = 0;
    Encoder_ClearCount(0);
    Motor_SetPWM(MOVE_SPEED, MOVE_SPEED);
    debug_println("=== Right square ===");
}

static void pattern_update(void)
{
    int32_t cnt;

    switch (state) {

    case ST_IDLE:
        break;

    case ST_MOVING:
        cnt = Encoder_GetCount(0);
        if (cnt < 0) cnt = -cnt;
        { static int t=0; if (++t>=200){t=0;debug_printf("E=%ld\r\n",(long)cnt);} }
        if (cnt >= MOVE_DISTANCE) {
            Motor_Stop();
            pause_tick = 80;    /* 停 400ms */
            state = ST_PAUSE;
            debug_printf("--- Step %d done, pause ---\r\n", step);
        }
        break;

    case ST_PAUSE:
        if (--pause_tick <= 0) {
            step++;
                if (step >= 5) {
                    state = ST_DONE;
                    debug_println("=== All done! ===");
                } else {
                    Encoder_ClearCount(0);
                    turn_start(-90.0f);    /* 右转 */
                    state = ST_TURNING;
                    debug_println("Turning -90...");
            }
        }
        break;

    case ST_TURNING:
        turn_update();
        if (!turn_active) {
            Encoder_ClearCount(0);
            pause_tick = 40;     /* 转弯后停 200ms */
            state = ST_SETTLE;
            debug_println("Turn done, settle...");
        }
        break;

    case ST_SETTLE:
        if (--pause_tick <= 0) {
            Encoder_ClearCount(0);
            Motor_SetPWM(MOVE_SPEED, MOVE_SPEED);
            state = ST_MOVING;
            debug_printf("Go step %d\r\n", step);
        }
        break;

    case ST_DONE:
        break;
    }
}

static void handle_command(char cmd)
{
    switch (cmd) {
        case 'G': case 'g': pattern_start(); break;
        case 'S': case 's': case ' ':
            state=ST_IDLE; turn_active=false; Motor_Stop();
            debug_println("Stop"); break;
        case 'Y': case 'y':
            debug_printf("Y=%.1f s=%d\r\n", (double)Gyro.yaw, step);
            break;
        case 'E': case 'e':
            debug_printf("Enc=%ld\r\n", (long)Encoder_GetCount(0));
            break;
        case 'R': case 'r':
            /* 单独右转 90°, 不跑正方形 */
            state = ST_IDLE; turn_active = false; Motor_Stop();
            delay_ms(100);
            turn_start(-90.0f);
            debug_println("Turn test: -90 deg");
            while (turn_active) { Gyro_Update(); turn_update(); delay_ms(5); }
            Motor_Stop();
            debug_printf("Done. Yaw=%.1f\r\n", (double)Gyro.yaw);
            break;
        case '1': kp += 1.0f; debug_printf("KP=%.1f\r\n", (double)kp); break;
        case '2': kp -= 1.0f; debug_printf("KP=%.1f\r\n", (double)kp); break;
        case '3': ki += 0.05f; debug_printf("KI=%.2f\r\n", (double)ki); break;
        case '4': ki -= 0.05f; debug_printf("KI=%.2f\r\n", (double)ki); break;
        case '5': kd += 2.0f; debug_printf("KD=%.1f\r\n", (double)kd); break;
        case '6': kd -= 2.0f; debug_printf("KD=%.1f\r\n", (double)kd); break;
        case '7': tlimit += 25; debug_printf("LIM=%.0f\r\n", (double)tlimit); break;
        case '8': tlimit -= 25; debug_printf("LIM=%.0f\r\n", (double)tlimit); break;
        case 'Z': case 'z':   /* 全部归零 */
            kp=0; ki=0; kd=0; tlimit=300;
            debug_println("PID all zero");
            break;
        case '0':
            kp=TURN_KP; ki=TURN_KI; kd=TURN_KD; tlimit=TURN_LIMIT;
            debug_println("PID reset to default");
            break;
        default: break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    Grayscale_Init();
    Gyro_Init();
    DL_SYSTICK_init(0x01000000);
    DL_SYSTICK_enable();
    debug_println("\r\n=== RC Car ===");
    debug_println("G=正方形 Y=角度 E=编码器 S=停");
    while (1) {
        Gyro_Update();
        if (state != ST_IDLE) pattern_update();
        if (debug_kbhit()) {
            char c = debug_getc();
            debug_printf("[0x%02X]", (unsigned char)c);
            handle_command(c);
        }
        delay_ms(5);
    }
}
