
#include "uart_callback.h"
#include "control.h"


#define BT_PACKET_SIZE  200
static volatile uint8_t gBTPacket[BT_PACKET_SIZE];
static volatile uint8_t gLastSize  = 0;


int Flag_Left, Flag_Right, Flag_Direction, Turn_Flag;


void BT_DMAConfig(void) {
    
    
    
    
    
    
    
    
}


void BTBufferHandler(void) {
    
    
    uint8_t recvSize = 0;  

    if (recvSize == gLastSize) return;  
    gLastSize = recvSize;

    
    for (uint8_t i = 0; i < recvSize; i++) {
        bt_protocol(gBTPacket[i]);
    }

    
    if (recvSize >= BT_PACKET_SIZE / 2) {
        gLastSize = 0;
        BT_DMAConfig();
    }
}


void bt_protocol(uint8_t recv) {

    /* ── 原有方向控制 ── */
    if (recv == 0x4B) Turn_Flag = 1;           
    else if (recv == 0x49 || recv == 0x4A) Turn_Flag = 0;  

    
    if (Turn_Flag == 0) {
        if (recv >= 0x41 && recv <= 0x48)
            Flag_Direction = recv - 0x40;       
        else if (recv <= 8)
            Flag_Direction = recv;
        else
            Flag_Direction = 0;
    }
    
    else {
        if      (recv == 0x43) { Flag_Left = 0; Flag_Right = 1; }
        else if (recv == 0x47) { Flag_Left = 1; Flag_Right = 0; }
        else                    { Flag_Left = 0; Flag_Right = 0; }
        if (recv == 0x41 || recv == 0x45)
            Flag_Direction = recv - 0x40;
        else
            Flag_Direction = 0;
    }

    
    if (recv == 0x58) Target_Velocity += 100.0f;
    if (recv == 0x59) Target_Velocity -= 100.0f;

    
}



