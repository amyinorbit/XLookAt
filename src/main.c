#include "look_at.h"
#include <XPLMPlugin.h>
#include <XPLMUtilities.h>
#include <XPLMDisplay.h>
#include <XPLMProcessing.h>
#include <XPLMDataAccess.h>
#include <string.h>
#include <stdio.h>
#include <ccore/log.h>

/*
Datarefs that interest us:

sim/flightmodel/position/local_x
sim/flightmodel/position/local_y
sim/flightmodel/position/local_z

sim/flightmodel/position/theta      pitch
sim/flightmodel/position/phi        roll
sim/flightmodel/position/psi        yaw
*/

#if IBM==1
#include <windows.h>
BOOL APIENTRY DllMain( HANDLE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    CCUNUSED(hModule);
    CCUNUSED(ul_reason_for_call);
    CCUNUSED(lpReserved);
    return TRUE;
}
#endif

PLUGIN_API float flight_loop(float since_last, float since_last_fl, int count, void *refcon) {
    CCUNUSED(since_last);
    CCUNUSED(since_last_fl);
    CCUNUSED(count);
    CCUNUSED(refcon);

    xla_frame();
    return -1;
}


static FILE* file_out = NULL;
void log_printer(const char *msg) {
    if(!file_out) {
        file_out = fopen("xlookat_log.txt", "w");
    }
    fprintf(file_out, "%s", msg);
    fflush(file_out);
    XPLMDebugString(msg);
}

PLUGIN_API int XPluginStart(char * outName, char * outSig, char *outDesc) {
    // Plugin details
	strcpy(outName, "XLookAt");
	strcpy(outSig, "com.amyinorbit.xlookat");
	strcpy(outDesc, "Focus view on runways, locations and airports");
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    cc_set_printer(log_printer);
    CCINFO("XLookAt - version 2008.1");

    xla_setup();
	return 1;
}

PLUGIN_API void	XPluginStop(void) {
    xla_cleanup();
}

PLUGIN_API int XPluginEnable(void) {
    CCINFO("starting...");
    XPLMRegisterFlightLoopCallback(flight_loop, 0.5f, NULL);
    xla_start();
	return 1;
}

PLUGIN_API void XPluginDisable(void) {
    CCINFO("stopping");
    XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
    xla_stop();
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID id, intptr_t inMessage, void * inParam) {
    CCUNUSED(id);
    CCUNUSED(inParam);

    if(inMessage != XPLM_MSG_PLANE_LOADED) return;
}
