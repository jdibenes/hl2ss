
#include <mfapi.h>
#include "../hl2ss/server.h"
#include "../hl2ss/locator.h"
#include "../hl2ss/research_mode.h"
#include "../hl2ss/spatial_input.h"
#include "../hl2ss/stream_rm.h"
#include "../hl2ss/stream_mc.h"
#include "../hl2ss/stream_pv.h"
#include "../hl2ss/stream_si.h"
#include "../hl2ss/stream_rc.h"
#include "../hl2ss/utilities.h"
#include "../hl2ss/timestamps.h"
#include "ipc.h"
#include "plugin.h"

#define ENABLE_RM 1
#define ENABLE_MC 2
#define ENABLE_PV 4
#define ENABLE_SI 8

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
UNITY_EXPORT
void InitializeStreams(uint32_t enable)
{
    InitializeSockets();
    MFStartup(MF_VERSION);

    Locator_Initialize();

    if (enable & ENABLE_RM) { ResearchMode_Initialize(); }
    if (enable & ENABLE_SI) { SpatialInput_Initialize(); }

    if (enable & ENABLE_RM) { RM_Initialize(); }
    if (enable & ENABLE_MC) { MC_Initialize(); }
    if (enable & ENABLE_PV) { PV_Initialize(); }
    if (enable & ENABLE_SI) { SI_Initialize(); }

    if (enable & ENABLE_PV) { RC_Initialize(); }

    MQ_Initialize();
}

// OK
UNITY_EXPORT
void DebugMessage(char const* str)
{
    ShowMessage("%s", str);
}

// OK
UNITY_EXPORT
void SI_Update()
{
    SI_NotifyNextFrame(QPCTimestampToPerceptionTimestamp(GetCurrentQPCTimestamp()));
}
