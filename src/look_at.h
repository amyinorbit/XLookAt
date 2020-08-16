//===--------------------------------------------------------------------------------------------===
// look_at - the actual implementation of the look-at mechanism
//
// Created by Amy Parent <amy@amyparent.com>
// Copyright (c) 2020 Amy Parent
// Licensed under the MIT License
// =^•.•^=
//===--------------------------------------------------------------------------------------------===
#pragma once

#define XLA_COMMAND_LOOK "amyinorbit/xla/look"
#define XLA_COMMAND_REM "amyinorbit/xla/remember"

typedef struct {
    union {
        struct { double x, y, z; };
        struct { double lat, lon, elev; };
        struct { double pit, hdg, rll; };
        double data[3];
    };
} vec3_t;

void xla_setup();
void xla_start();
void xla_stop();
void xla_cleanup();

void xla_frame();
