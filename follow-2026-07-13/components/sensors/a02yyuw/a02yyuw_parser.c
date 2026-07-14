#include "a02yyuw_parser.h"

bool a02yyuw_parse_frame(const uint8_t *frame, size_t len,
                         a02yyuw_reading_t *out)
{
    if (out != NULL) {
        out->distance_mm = 0;
        out->valid = false;
    }
    if (frame == NULL || out == NULL || len < 4 || frame[0] != 0xFF) return false;
    const uint8_t checksum = (uint8_t)(frame[0] + frame[1] + frame[2]);
    if (checksum != frame[3]) return false;
    const uint16_t distance = ((uint16_t)frame[1] << 8) | frame[2];
    if (distance < A02YYUW_MIN_DISTANCE_MM || distance > A02YYUW_MAX_DISTANCE_MM) return false;
    out->distance_mm = (int)distance;
    out->valid = true;
    return true;
}

bool a02yyuw_parse_latest(const uint8_t *buffer, size_t len,
                          a02yyuw_reading_t *out)
{
    if (out == NULL) return false;
    *out = (a02yyuw_reading_t){0};
    if (buffer == NULL || len < 4) return false;
    bool found = false;
    for (size_t index = 0; index + 4 <= len; ++index) {
        a02yyuw_reading_t current = {0};
        if (a02yyuw_parse_frame(&buffer[index], 4, &current)) {
            *out = current;
            found = true;
        }
    }
    return found;
}
