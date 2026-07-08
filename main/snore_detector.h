#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void snore_detector_start(void);
void snore_detector_set_policy(bool enabled,
                               bool sleep_active,
                               float target_kpa,
                               int cooldown_sec);
void snore_detector_set_interaction_active(bool active);
bool snore_detector_is_enabled(void);

#ifdef __cplusplus
}
#endif
