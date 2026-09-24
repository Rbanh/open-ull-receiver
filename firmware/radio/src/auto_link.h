#pragma once
#include <stdbool.h>
#include <stdint.h>
void ull_auto_init(void);
bool ull_auto_receive(const uint8_t *data, uint16_t length);
bool ull_auto_enabled(void);
void ull_auto_enable(bool enabled);
const char *ull_auto_state(void);
void ull_feature_set(bool enabled);
