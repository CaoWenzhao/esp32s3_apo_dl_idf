#include "bu_uwb_parser.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool bu_uwb_parse_distance_line(const char *line, bu_uwb_distance_t *out)
{
    if (out != NULL) *out = (bu_uwb_distance_t){0};
    if (line == NULL || out == NULL) return false;
    const char *position = strstr(line, "distance:");
    if (position == NULL) position = strstr(line, "distance=");
    if (position == NULL) position = strstr(line, "dist:");
    if (position == NULL) position = strstr(line, "dist=");
    if (position == NULL) return false;
    const char *colon = strchr(position, ':');
    const char *equals = strchr(position, '=');
    if (colon == NULL) position = equals;
    else if (equals == NULL) position = colon;
    else position = colon < equals ? colon : equals;
    if (position == NULL) return false;
    char *end = NULL;
    const float distance = strtof(position + 1, &end);
    if (end == position + 1 || distance < 0.0f) return false;
    out->distance_m = distance;
    out->distance_mm = (int)(distance * 1000.0f + 0.5f);
    out->valid = true;
    return true;
}

static bool json_string(const char *line, const char *key, char *out,
                        size_t out_size)
{
    char pattern[24];
    if (line == NULL || key == NULL || out == NULL || out_size == 0) return false;
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *position = strstr(line, pattern);
    if (position == NULL) return false;
    position += strlen(pattern);
    const char *end = strchr(position, '"');
    if (end == NULL || end == position) return false;
    size_t length = (size_t)(end - position);
    if (length >= out_size) length = out_size - 1;
    memcpy(out, position, length);
    out[length] = '\0';
    return true;
}

static bool json_int(const char *line, const char *key, int *out)
{
    char pattern[24];
    if (line == NULL || key == NULL || out == NULL) return false;
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *position = strstr(line, pattern);
    if (position == NULL) return false;
    position += strlen(pattern);
    char *end = NULL;
    const long value = strtol(position, &end, 10);
    if (end == position) return false;
    while (*end == ' ') ++end;
    if (*end != ',' && *end != '}') return false;
    *out = (int)value;
    return true;
}

bool bu_uwb_parse_twr_line(const char *line, bu_uwb_twr_reading_t *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (line == NULL || out == NULL || strncmp(line, "JS", 2) != 0 ||
        strstr(line, "\"TWR\"") == NULL) return false;
    const char *json = strchr(line, '{');
    if (json == NULL || json <= line + 2) return false;
    size_t frame_length = (size_t)(json - line);
    if (frame_length >= sizeof(out->frame_id)) frame_length = sizeof(out->frame_id) - 1;
    memcpy(out->frame_id, line, frame_length);
    out->frame_id[frame_length] = '\0';
    if (!json_string(line, "a16", out->anchor_id, sizeof(out->anchor_id)) ||
        !json_int(line, "R", &out->r) || !json_int(line, "T", &out->timestamp) ||
        !json_int(line, "D", &out->distance_cm) || !json_int(line, "P", &out->p) ||
        !json_int(line, "Xcm", &out->x_cm) || !json_int(line, "Ycm", &out->y_cm) ||
        !json_int(line, "O", &out->orientation) || !json_int(line, "V", &out->validity) ||
        !json_int(line, "X", &out->x) || !json_int(line, "Y", &out->y) ||
        !json_int(line, "Z", &out->z)) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    out->valid = true;
    return true;
}

bu_uwb_line_type_t bu_uwb_classify_line(const char *line)
{
    if (line == NULL) return BU_UWB_LINE_UNKNOWN;
    if (strncmp(line, "DATA,", 5) == 0) return BU_UWB_LINE_DATA;
    if (strncmp(line, "ERR,", 4) == 0) return BU_UWB_LINE_ERROR;
    if (strncmp(line, "JS", 2) == 0 && strstr(line, "\"TWR\"") != NULL)
        return BU_UWB_LINE_TWR;
    return BU_UWB_LINE_UNKNOWN;
}

const char *bu_uwb_line_payload(const char *line)
{
    if (line == NULL) return NULL;
    const bu_uwb_line_type_t type = bu_uwb_classify_line(line);
    if (type == BU_UWB_LINE_DATA) return line + 5;
    if (type == BU_UWB_LINE_ERROR) return line + 4;
    return line;
}
