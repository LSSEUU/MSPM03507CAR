#ifndef M6_H_MISSION_EXT_H
#define M6_H_MISSION_EXT_H

/* Generated from Tests/vectors/m6_h_mission_ext_v1.json.
 * registry-sha256: 973cb83992eaa81bd4a021391e8c98498e78ddf261c7c13949c163338bab9b67 */
#define M6_CAP_H_MISSION_CONTROL          (UINT64_C(1) << 29)
#define M6_RPC_H_MISSION_STATUS_GET       0x000FU
#define M6_RPC_H_MISSION_COMMAND          0x8309U
#define M6_H_MISSION_COMMAND_REQUEST_SIZE 32U
#define M6_H_MISSION_SNAPSHOT_SIZE        64U

#define M6_H_MISSION_COMMAND_STAGE        1U
#define M6_H_MISSION_COMMAND_APPLY        2U
#define M6_H_MISSION_COMMAND_REVERT       3U
#define M6_H_MISSION_COMMAND_SELECT_MODE  4U
#define M6_H_MISSION_COMMAND_ABORT        5U

#endif
