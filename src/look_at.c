//===--------------------------------------------------------------------------------------------===
// look_at - XLookAt implementation
//
// Created by Amy Parent <amy@amyparent.com>
// Copyright (c) 2020 Amy Parent
// Licensed under the MIT License
// =^•.•^=
//===--------------------------------------------------------------------------------------------===
#include "look_at.h"
#include <XPLMUtilities.h>
#include <XPLMDataAccess.h>
#include <XPLMGraphics.h>
#include <ccore/log.h>
#include <stdbool.h>
#include <stdlib.h>
#include <tgmath.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static struct {
    bool is_looking;
    bool is_failed;
    vec3_t target;
    vec3_t saved_head;

    struct {
        vec3_t target;
        vec3_t plane_loc;
        vec3_t plane_rot;
    } ogl;

    struct {
        XPLMDataRef plane_lat;
        XPLMDataRef plane_lon;
        XPLMDataRef plane_elev;

        XPLMDataRef plane_pit;
        XPLMDataRef plane_hdg;
        XPLMDataRef plane_rll;

        XPLMDataRef plane_x;
        XPLMDataRef plane_y;
        XPLMDataRef plane_z;

        XPLMDataRef head_pit;
        XPLMDataRef head_hdg;
        XPLMDataRef head_rll;
    } dr;
    XPLMCommandRef look_cmd;
    XPLMCommandRef remember_cmd;
} state;


void xla_setup() {
    state.is_looking = false;
    state.is_failed = false;
    state.look_cmd = XPLMCreateCommand(XLA_COMMAND_LOOK, "Look at the current target");
    state.remember_cmd = XPLMCreateCommand(XLA_COMMAND_REM, "Set the current position as new target");
    CCASSERT(state.look_cmd);

    state.dr.plane_lat = NULL;
    state.dr.plane_lon = NULL;
    state.dr.plane_elev = NULL;

    state.dr.plane_pit = NULL;
    state.dr.plane_hdg = NULL;
    state.dr.plane_rll = NULL;

    state.dr.plane_x = NULL;
    state.dr.plane_y = NULL;
    state.dr.plane_z = NULL;

    state.dr.head_pit = NULL;
    state.dr.head_hdg = NULL;
    state.dr.head_rll = NULL;
}

static XPLMDataRef find_dref_checked(const char *path, bool strict) {
    XPLMDataRef dr = XPLMFindDataRef(path);
    if(!dr) {
        CCERROR("dataref `%s` not found", path);
        state.is_failed = true;
        if(strict) abort();
    } else {
        CCINFO("dataref `%s` found", path);
    }
    return dr;
}

static int bool_cmd(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon) {
    CCUNUSED(cmd);
    bool *val = refcon;
    switch(phase) {
    case xplm_CommandBegin:
        *val = true;
        state.saved_head.pit = XPLMGetDataf(state.dr.head_pit);
        state.saved_head.rll = XPLMGetDataf(state.dr.head_rll);
        state.saved_head.hdg = XPLMGetDataf(state.dr.head_hdg);
        break;
    case xplm_CommandEnd:
        *val = false;
        XPLMSetDataf(state.dr.head_pit, state.saved_head.pit);
        XPLMSetDataf(state.dr.head_rll, state.saved_head.rll);
        XPLMSetDataf(state.dr.head_hdg, state.saved_head.hdg);
        break;
    default:
        break;
    }
    return 1;
}

static int remember_place(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon) {
    CCUNUSED(cmd);
    CCUNUSED(refcon);
    if(phase != xplm_CommandBegin) return 1;

    state.target.lat = XPLMGetDatad(state.dr.plane_lat);
    state.target.lon = XPLMGetDatad(state.dr.plane_lon);
    state.target.elev = XPLMGetDatad(state.dr.plane_elev);

    CCINFO("lat: %f", state.target.lat);
    CCINFO("lon: %f", state.target.lon);
    CCINFO("elev: %f", state.target.elev);
    return 1;
}


void xla_start() {
    CCINFO("finding plane rotation datarefs");

    state.dr.plane_lat = find_dref_checked("sim/flightmodel/position/latitude", true);
    state.dr.plane_lon = find_dref_checked("sim/flightmodel/position/longitude", true);
    state.dr.plane_elev = find_dref_checked("sim/flightmodel/position/elevation", true);

    state.dr.plane_pit = find_dref_checked("sim/flightmodel/position/theta", true);
    state.dr.plane_hdg = find_dref_checked("sim/flightmodel/position/psi", true);
    state.dr.plane_rll = find_dref_checked("sim/flightmodel/position/phi", true);

    CCINFO("finding plane position datarefs");
    state.dr.plane_x = find_dref_checked("sim/flightmodel/position/local_x", true);
    state.dr.plane_y = find_dref_checked("sim/flightmodel/position/local_y", true);
    state.dr.plane_z = find_dref_checked("sim/flightmodel/position/local_z", true);

    CCINFO("finding pilot's head rotation datarefs");
    state.dr.head_pit = find_dref_checked("sim/graphics/view/pilots_head_the", true);
    state.dr.head_hdg = find_dref_checked("sim/graphics/view/pilots_head_psi", true);
    state.dr.head_rll = find_dref_checked("sim/graphics/view/pilots_head_phi", true);

    CCINFO("installing command handler");
    XPLMRegisterCommandHandler(state.look_cmd, bool_cmd, 0, &state.is_looking);
    XPLMRegisterCommandHandler(state.remember_cmd, remember_place, 0, NULL);
}

void xla_stop() {
    XPLMUnregisterCommandHandler(state.look_cmd, bool_cmd, 0, &state.is_looking);
    XPLMUnregisterCommandHandler(state.remember_cmd, remember_place, 0, NULL);
}

void xla_cleanup() {

}

static inline double vec3_square_mag(const vec3_t *v) {
    return (v->x*v->x) + (v->y*v->y) + (v->z*v->z);
}

static inline double vec3_mag(const vec3_t *v) {
    return sqrt(vec3_square_mag(v));
}

static inline void vec3_normalize(vec3_t *v) {
    double m = vec3_mag(v);
    if(m == 0) return;
    v->x /= m;
    v->y /= m;
    v->z /= m;
}

static inline double deg_to_rad(double v) {
    return v * (M_PI / 180.0);
}

static inline double rad_to_deg(double v) {
    return v * (180.0 / M_PI);
}

static inline double dot4(const double *a, const double *m, int col) {
    return a[0]*m[col] + a[1]*m[col+4] + a[2]*m[col+8] + a[3]*m[col+12];
}

static void vec3_transform(vec3_t *out, double m[16]) {
    double v[4] = {out->x, out->y, out->z, 1.0};
    out->x = dot4(v, m, 0);
    out->y = dot4(v, m, 1);
    out->z = dot4(v, m, 2);
}

void dbg_row(const double *row) {
    CCINFO("\t%f\t%f\t%f\t%f", row[0], row[1], row[2], row[3]);
}

static inline double normalize_hdg(double hdg) {
    if(hdg < 0.0) hdg += 360.0;
    if(hdg >= 360.0) hdg -= 360.0;
    return hdg;
}

void xla_frame() {
    if(!state.is_looking) return;

    state.ogl.plane_loc.x = XPLMGetDatad(state.dr.plane_x);
    state.ogl.plane_loc.y = XPLMGetDatad(state.dr.plane_y);
    state.ogl.plane_loc.z = XPLMGetDatad(state.dr.plane_z);

    state.ogl.plane_rot.hdg = XPLMGetDataf(state.dr.plane_hdg);
    state.ogl.plane_rot.pit = XPLMGetDataf(state.dr.plane_pit);
    state.ogl.plane_rot.rll = XPLMGetDataf(state.dr.plane_rll);

    XPLMWorldToLocal(
        state.target.lat, state.target.lon, state.target.elev,
        &state.ogl.target.x, &state.ogl.target.y, &state.ogl.target.z
    );


    vec3_t look;
    look.x = state.ogl.target.x - state.ogl.plane_loc.x;
    look.y = state.ogl.target.y - state.ogl.plane_loc.y;
    look.z = state.ogl.target.z - state.ogl.plane_loc.z;

    // So now we have a vector pointing from the plane to the target.
    // We need to change that from X-Plane's OpenGL coordinate system to the plane's frame.
    // Matrix maths ahoy!
    // This is really just euler angles converted to a rotation matrix
    double c1 = cos(deg_to_rad(-state.ogl.plane_rot.rll));
    double s1 = sin(deg_to_rad(-state.ogl.plane_rot.rll));

    double c2 = cos(deg_to_rad(-state.ogl.plane_rot.pit));
    double s2 = sin(deg_to_rad(-state.ogl.plane_rot.pit));

    double c3 = cos(deg_to_rad(-state.ogl.plane_rot.hdg));
    double s3 = sin(deg_to_rad(-state.ogl.plane_rot.hdg));


    double rot_mat[16] = {
        c1*c3 - s1*s2*s3,   -c2*s1,         c1*s3 + c3*s1*s2,   0.0,
        c3*s1 + c1*s2*s3,   c1*c2,          s1*s3 - c1*c3*s2,   0.0,
        -c2*s3,             s2,             c2*c3,              0.0,
        0.0,                0.0,            0.0,                1.0,
    };

    vec3_transform(&look, rot_mat);
    vec3_normalize(&look);

    // Now that we have a proper vector, we can do things to it!
    CCINFO("Transformed vector: %f, %f, %f", look.x, look.y, look.z);
    // double hdg = atan2(look.x, look.z);
    double elev = rad_to_deg(asin(look.y));
    double hdg = rad_to_deg(atan2(look.x, -look.z));

    CCINFO("targets: %.1f, %.1f", elev, normalize_hdg(hdg));
    XPLMSetDataf(state.dr.head_hdg, normalize_hdg(hdg));
    XPLMSetDataf(state.dr.head_pit, elev);
}
