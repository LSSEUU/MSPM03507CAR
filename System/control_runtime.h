#ifndef CONTROL_RUNTIME_H
#define CONTROL_RUNTIME_H

#include "control_params.h"

#include <stdbool.h>

/* Maps the active parameter snapshot into the control/application modules. */
bool ControlRuntime_ApplyActive(const ControlParamsCommit_t *commit);

#endif
