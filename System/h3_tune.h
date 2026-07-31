#ifndef H3_TUNE_H
#define H3_TUNE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    H3_TUNE_PARAM_PLUS_START_MDEG = 1,
    H3_TUNE_PARAM_PLUS_MAX_MDEG,
    H3_TUNE_PARAM_MINUS_START_MDEG,
    H3_TUNE_PARAM_MINUS_MAX_MDEG,
    H3_TUNE_PARAM_KP,
    H3_TUNE_PARAM_KD,
    H3_TUNE_PARAM_NORMAL_SLEW,
    H3_TUNE_PARAM_BREAKAWAY_SLEW,
    H3_TUNE_PARAM_BRAKE_SLEW,
    H3_TUNE_PARAM_TRACKING_RPM,
    H3_TUNE_PARAM_PLUS_LAUNCH_RPM,
    H3_TUNE_PARAM_MINUS_LAUNCH_RPM,
    H3_TUNE_PARAM_KI,
    H3_TUNE_PARAM_PLUS_TOPUP_MDEG,
    H3_TUNE_PARAM_PLUS_BRAKE_MDEG,
    H3_TUNE_PARAM_MINUS_BRAKE_TRIGGER_X10,
    H3_TUNE_PARAM_MINUS_BRAKE_MDEG,
    H3_TUNE_PARAM_PLUS_BRAKE_TRIGGER_X10,
    H3_TUNE_PARAM_PLUS_TOPUP_BRAKE_TRIGGER_X10,
    H3_TUNE_PARAM_MINUS_BRAKE_RETURN_PREDICT_MS,
    H3_TUNE_PARAM_MINUS_RETURN_RPM,
    H3_TUNE_PARAM_PLUS_TOPUP_MAX_SPEED_MM_S,
    H3_TUNE_PARAM_COUNT
} H3TuneParameter_t;

typedef enum {
    H3_TUNE_OK = 0,
    H3_TUNE_INVALID_ARGUMENT,
    H3_TUNE_OUT_OF_RANGE,
    H3_TUNE_WRONG_STATE,
    H3_TUNE_STALE_ACTIVE,
    H3_TUNE_STALE_STAGE,
    H3_TUNE_NOT_AUTHORIZED,
    H3_TUNE_AUTH_EXPIRED,
    H3_TUNE_AUTH_EXHAUSTED,
    H3_TUNE_START_REJECTED
} H3TuneResult_t;

typedef enum {
    H3_TUNE_STOP_NONE = 0,
    H3_TUNE_STOP_HOST,
    H3_TUNE_STOP_REVOKED,
    H3_TUNE_STOP_HEARTBEAT,
    H3_TUNE_STOP_RUN_TIMEOUT,
    H3_TUNE_STOP_AUTH_EXPIRED,
    H3_TUNE_STOP_DISCONNECT,
    H3_TUNE_STOP_FAULT
} H3TuneStopReason_t;

typedef struct {
    uint16_t plus_start_mdeg;
    uint16_t plus_max_mdeg;
    uint16_t minus_start_mdeg;
    uint16_t minus_max_mdeg;
    uint16_t kp_mdeg_per_mm;
    uint16_t kd_mdeg_per_mm_s;
    uint16_t normal_slew_mdeg_per_20ms;
    uint16_t breakaway_slew_mdeg_per_20ms;
    uint16_t brake_slew_mdeg_per_20ms;
    uint16_t tracking_rpm;
    uint16_t plus_launch_rpm;
    uint16_t minus_launch_rpm;
    uint16_t ki_mdeg_per_mm_s;
    uint16_t plus_topup_mdeg;
    uint16_t plus_brake_mdeg;
    uint16_t minus_brake_trigger_x10;
    uint16_t minus_brake_mdeg;
    uint16_t plus_brake_trigger_x10;
    uint16_t plus_topup_brake_trigger_x10;
    uint16_t minus_brake_return_predict_ms;
    uint16_t minus_return_rpm;
    uint16_t plus_topup_max_speed_mm_s;
} H3TuneProfile_t;

typedef struct {
    H3TuneProfile_t active;
    H3TuneProfile_t staged;
    uint32_t active_revision;
    uint32_t stage_revision;
    uint32_t run_id;
    uint32_t auth_token;
    uint32_t auth_expires_ms;
    uint32_t run_deadline_ms;
    uint32_t last_heartbeat_ms;
    H3TuneStopReason_t stop_reason;
    H3TuneResult_t last_result;
    uint8_t remaining_runs;
    bool dirty;
    bool authorized;
    bool host_run_active;
} H3TuneSnapshot_t;

void H3Tune_Init(uint32_t now_ms);
const H3TuneProfile_t *H3Tune_Active(void);
bool H3Tune_ProfileValid(const H3TuneProfile_t *profile);
H3TuneResult_t H3Tune_StageField(H3TuneParameter_t parameter,
                                uint16_t value,
                                uint32_t expected_active_revision,
                                uint32_t expected_stage_revision);
H3TuneResult_t H3Tune_Apply(uint32_t expected_active_revision,
                            uint32_t expected_stage_revision);
H3TuneResult_t H3Tune_Revert(uint32_t expected_active_revision,
                             uint32_t expected_stage_revision);
H3TuneResult_t H3Tune_Authorize(uint32_t token, uint8_t runs,
                                uint32_t ttl_ms, uint32_t run_timeout_ms,
                                uint32_t now_ms);
H3TuneResult_t H3Tune_Start(uint32_t token, uint32_t now_ms);
H3TuneResult_t H3Tune_StartCenterHold(uint32_t token, uint32_t now_ms);
H3TuneResult_t H3Tune_Stop(uint32_t token, uint32_t now_ms);
H3TuneResult_t H3Tune_Heartbeat(uint32_t token, uint32_t now_ms);
H3TuneResult_t H3Tune_Revoke(uint32_t token, H3TuneStopReason_t reason);
void H3Tune_OnDisconnect(uint32_t now_ms);
void H3Tune_Task20ms(uint32_t now_ms);
void H3Tune_RecordResult(H3TuneResult_t result);
void H3Tune_GetSnapshot(H3TuneSnapshot_t *snapshot);

#endif
