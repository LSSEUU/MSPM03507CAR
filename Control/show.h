/*
 * show.h  —  显示管理 (OLED + 蓝牙 APP 数据上报)
 * 参考: WHEELTEC_C07A_Bluetooth/Control/show.h
 */
#ifndef SHOW_H
#define SHOW_H
#include "board.h"

void Show_Init(void);
void oled_show(void);                        /* OLED 显示刷新 */
void APP_Show(void);                         /* 蓝牙 APP 数据上报 */

#endif
