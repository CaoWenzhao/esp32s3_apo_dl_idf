#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define A02YYUW_MIN_DISTANCE_MM 30
#define A02YYUW_MAX_DISTANCE_MM 4500

typedef struct {
    int distance_mm;
    bool valid;
} a02yyuw_reading_t;

bool a02yyuw_parse_frame(const uint8_t *frame, size_t len,
                         a02yyuw_reading_t *out);
bool a02yyuw_parse_latest(const uint8_t *buffer, size_t len,
                          a02yyuw_reading_t *out);
