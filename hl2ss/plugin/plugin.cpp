
#include <mfapi.h>
#include "../hl2ss/server.h"
#include "../hl2ss/locator.h"
#include "../hl2ss/research_mode.h"
#include "../hl2ss/spatial_input.h"
#include "../hl2ss/stream_rm.h"
#include "../hl2ss/stream_mc.h"
#include "../hl2ss/stream_pv.h"
#include "../hl2ss/stream_si.h"
#include "../hl2ss/utilities.h"
#include "ipc_message_queue.h"
#include "plugin.h"

UNITY_API
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

    MQ_Initialize();
}

UNITY_API
void DebugMessage(char const* str)
{
    ShowMessage("%s", str);
}

UNITY_API
void SI_Update()
{
    SI_NotifyNextFrame(QPCTimestampToPerceptionTimestamp(GetCurrentQPCTimestamp()));
}
