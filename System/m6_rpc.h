#ifndef M6_RPC_H
#define M6_RPC_H

#include "control_protocol.h"
#include "m6_link.h"

#include <stdint.h>

void M6Rpc_Init(ControlProtocolCommitCallback_t commit_callback);
M6Result_t M6Rpc_Execute(uint16_t opcode, const uint8_t *body,
                         uint16_t body_length, uint8_t *success_body,
                         uint16_t *success_length, uint32_t *detail,
                         void *context);
void M6Rpc_Service(uint32_t now_ms);
void M6Rpc_Telemetry20ms(uint32_t now_ms);
void M6Rpc_ParameterBoundaryCompleted(ControlParamsResult_t result,
                                      const ControlParamsCommit_t *commit);
void M6Rpc_CancelPendingApply(M6Result_t result, uint32_t detail);

#endif
