#pragma once

#include <stdbool.h>

#define BU_UWB_LINE_MAX 160

typedef struct { float distance_m; int distance_mm; bool valid; } bu_uwb_distance_t;
typedef struct {
    char frame_id[8]; char anchor_id[8]; int r; int timestamp; int distance_cm;
    int p; int x_cm; int y_cm; int orientation; int validity; int x; int y;
    int z; bool valid;
} bu_uwb_twr_reading_t;
typedef enum {
    BU_UWB_LINE_UNKNOWN = 0, BU_UWB_LINE_DATA, BU_UWB_LINE_ERROR,
    BU_UWB_LINE_TWR,
} bu_uwb_line_type_t;

bool bu_uwb_parse_distance_line(const char *line, bu_uwb_distance_t *out);
bool bu_uwb_parse_twr_line(const char *line, bu_uwb_twr_reading_t *out);
bu_uwb_line_type_t bu_uwb_classify_line(const char *line);
const char *bu_uwb_line_payload(const char *line);
