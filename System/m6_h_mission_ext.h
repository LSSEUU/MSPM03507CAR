#ifndef M6_H_MISSION_EXT_H
#define M6_H_MISSION_EXT_H

/* Generated from Tests/vectors/m6_h_mission_ext_v1.json.
 * registry-sha256: ab018fd70060e42f7a7ac40237032f394e98f1e1afcb35cebf16ceeb4d055a9a */
#define M6_CAP_H_MISSION_CONTROL          (UINT64_C(1) << 29)
#define M6_RPC_H_MISSION_STATUS_GET       0x000FU
#define M6_RPC_H_MISSION_COMMAND          0x8309U
#define M6_H_MISSION_COMMAND_REQUEST_SIZE 30U
#define M6_H_MISSION_SNAPSHOT_SIZE        64U

#define M6_H_MISSION_COMMAND_STAGE        1U
#define M6_H_MISSION_COMMAND_APPLY        2U
#define M6_H_MISSION_COMMAND_REVERT       3U
#define M6_H_MISSION_COMMAND_SELECT_MODE  4U
#define M6_H_MISSION_COMMAND_ABORT        5U
#define M6_H_MISSION_COMMAND_START        6U

#endif
