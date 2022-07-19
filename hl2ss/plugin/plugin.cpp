
#include <mfapi.h>
#include "../hl2ss/server.h"
#include "../hl2ss/research_mode.h"
#include "../hl2ss/locator.h"
#include "../hl2ss/stream_rm.h"
#include "../hl2ss/stream_mc.h"
#include "../hl2ss/stream_pv.h"
#include "../hl2ss/utilities.h"
#include "ipc_message_queue.h"
#include "plugin.h"

UNITY_API
void InitializeStreams()
{
    InitializeSockets();
    MFStartup(MF_VERSION);

    Locator_Initialize();
    ResearchMode_Initialize();

    RM_Initialize();
    //PV_Initialize();
    MC_Initialize();
    MQ_Initialize();
}

UNITY_API
void DebugMessage(char const* str)
{
    ShowMessage("%s", str);
}
