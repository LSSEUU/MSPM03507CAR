#ifndef M6_ACTION_VALIDATE_H
#define M6_ACTION_VALIDATE_H

#include "m6_link.h"

#include <stdint.h>

M6Result_t M6Action_Validate(const uint8_t *body, uint16_t body_length,
                             uint64_t enabled_capabilities);

#endif
