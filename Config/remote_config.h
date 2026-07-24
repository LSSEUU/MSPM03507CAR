#ifndef REMOTE_CONFIG_H
#define REMOTE_CONFIG_H

/* Stage-one production gate. Set to 1 only after all 17 keys have been
 * captured on the actual handset and the address/command table below has
 * been reconciled with that evidence. Host application tests override it. */
#ifndef IR_REMOTE_KEYMAP_VERIFIED
#define IR_REMOTE_KEYMAP_VERIFIED              0
#endif

/* Provisional KEYES 17-key NEC values; not yet hardware-verified. */
#define IR_REMOTE_EXPECTED_ADDRESS          0x00U
#define IR_REMOTE_KEY_UP                    0x62U
#define IR_REMOTE_KEY_DOWN                  0xA8U
#define IR_REMOTE_KEY_LEFT                  0x22U
#define IR_REMOTE_KEY_RIGHT                 0xC2U
#define IR_REMOTE_KEY_OK                    0x02U
#define IR_REMOTE_KEY_STAR                  0x98U
#define IR_REMOTE_KEY_HASH                  0xB0U
#define IR_REMOTE_KEY_0                     0x68U
#define IR_REMOTE_KEY_1                     0x30U
#define IR_REMOTE_KEY_2                     0x18U
#define IR_REMOTE_KEY_3                     0x7AU
#define IR_REMOTE_KEY_4                     0x10U
#define IR_REMOTE_KEY_5                     0x38U
#define IR_REMOTE_KEY_6                     0x5AU
#define IR_REMOTE_KEY_7                     0x42U
#define IR_REMOTE_KEY_8                     0x4AU
#define IR_REMOTE_KEY_9                     0x52U

#define IR_REMOTE_SPEED_LOW_MM_S             100U
#define IR_REMOTE_SPEED_DEFAULT_MM_S         160U
#define IR_REMOTE_SPEED_HIGH_MM_S            220U
#define IR_REMOTE_HEARTBEAT_MS               250U
#define IR_REMOTE_SAFETY_DEADLINE_MS         500U
#define IR_REMOTE_PENDING_MS               10000U
#define IR_REMOTE_CAPTURE_MS               30000U
#define IR_REMOTE_STRAIGHT_DEMO_MS          2000U

#endif
