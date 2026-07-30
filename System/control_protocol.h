#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include "control_params.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef M2_FAULT_INJECTION_TEST
#define CONTROL_PROTOCOL_FIRMWARE_ID "M6-runtime-20260731-r56-fi"
#else
#define CONTROL_PROTOCOL_FIRMWARE_ID "M6-runtime-20260731-r56"
#endif
#define CONTROL_PROTOCOL_SCHEMA_VERSION CONTROL_PARAMS_SCHEMA_VERSION
/* Payload only; CR/LF keeps every complete wire frame within 60 bytes. */
#define CONTROL_PROTOCOL_MAX_FRAME 58U

typedef void (*ControlProtocolCommitCallback_t)(
    const ControlParamsCommit_t *commit);

void ControlProtocol_Init(ControlProtocolCommitCallback_t commit_callback);

/* Returns true when the byte belongs to transaction framing (including EOL). */
bool ControlProtocol_ProcessByte(char byte, uint32_t now_ms);

/* Complete a diagnostic apply at the start of a 5 ms wheel boundary. */
void ControlProtocol_WheelBoundary(void);
bool ControlProtocol_ApplyLegacyStaged(void);

/* Retry bounded replies and discard partial frames after ISR/drop events. */
void ControlProtocol_Service(void);
bool ControlProtocol_IsFrameActive(void);

#endif
