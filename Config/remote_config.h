#ifndef REMOTE_CONFIG_H
#define REMOTE_CONFIG_H

/* All 17 keys were captured from the actual handset on 2026-07-24. */
#ifndef IR_REMOTE_KEYMAP_VERIFIED
#define IR_REMOTE_KEYMAP_VERIFIED              1
#endif

/* Measured KEYES 17-key NEC command bytes, address 0x00. */
#define IR_REMOTE_EXPECTED_ADDRESS          0x00U
#define IR_REMOTE_KEY_UP                    0x18U
#define IR_REMOTE_KEY_DOWN                  0x52U
#define IR_REMOTE_KEY_LEFT                  0x08U
#define IR_REMOTE_KEY_RIGHT                 0x5AU
#define IR_REMOTE_KEY_OK                    0x1CU
#define IR_REMOTE_KEY_STAR                  0x16U
#define IR_REMOTE_KEY_HASH                  0x0DU
#define IR_REMOTE_KEY_0                     0x19U
#define IR_REMOTE_KEY_1                     0x45U
#define IR_REMOTE_KEY_2                     0x46U
#define IR_REMOTE_KEY_3                     0x47U
#define IR_REMOTE_KEY_4                     0x44U
#define IR_REMOTE_KEY_5                     0x40U
#define IR_REMOTE_KEY_6                     0x43U
#define IR_REMOTE_KEY_7                     0x07U
#define IR_REMOTE_KEY_8                     0x15U
#define IR_REMOTE_KEY_9                     0x09U

#define IR_REMOTE_SPEED_LOW_MM_S             100U
#define IR_REMOTE_SPEED_DEFAULT_MM_S         160U
#define IR_REMOTE_SPEED_HIGH_MM_S            220U
#define IR_REMOTE_HEARTBEAT_MS               250U
#define IR_REMOTE_SAFETY_DEADLINE_MS         500U
#define IR_REMOTE_PENDING_MS               10000U
#define IR_REMOTE_CAPTURE_MS               30000U
#define IR_REMOTE_STRAIGHT_DEMO_MS          2000U

#endif
