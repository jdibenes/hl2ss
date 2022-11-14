
#include <mfapi.h>
#include "ipc.h"
#include "plugin.h"

#include "../hl2ss/server.h"
#include "../hl2ss/locator.h"
#include "../hl2ss/research_mode.h"
#include "../hl2ss/spatial_input.h"
#include "../hl2ss/stream_rm.h"
#include "../hl2ss/stream_mc.h"
#include "../hl2ss/stream_pv.h"
#include "../hl2ss/stream_si.h"
#include "../hl2ss/ipc_rc.h"
#include "../hl2ss/timestamps.h"
#include "../hl2ss/log.h"

#define HL2SS_ENABLE_RM  1
#define HL2SS_ENABLE_MC  2
#define HL2SS_ENABLE_PV  4
#define HL2SS_ENABLE_SI  8
#define HL2SS_ENABLE_RC 16

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

    if (enable & HL2SS_ENABLE_RM) { ResearchMode_Initialize(); }
    if (enable & HL2SS_ENABLE_SI) { SpatialInput_Initialize(); }

    if (enable & HL2SS_ENABLE_RM) { RM_Initialize(); }
    if (enable & HL2SS_ENABLE_MC) { MC_Initialize(); }
    if (enable & HL2SS_ENABLE_PV) { PV_Initialize(); }
    if (enable & HL2SS_ENABLE_SI) { SI_Initialize(); }

    if (enable & HL2SS_ENABLE_RC) { RC_Initialize(); }

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
