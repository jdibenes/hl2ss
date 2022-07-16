
#include <mfapi.h>
#include "../hl2ss/server.h"
#include "../hl2ss/research_mode.h"
#include "../hl2ss/locator.h"
#include "../hl2ss/stream_rm.h"
#include "../hl2ss/stream_mc.h"
#include "../hl2ss/stream_pv.h"

extern "C" __declspec(dllexport)
void __stdcall InitializeStreams()
{
    InitializeSockets();
    MFStartup(MF_VERSION);

    Locator_Initialize();
    ResearchMode_Initialize();

    RM_Initialize();
    PV_Initialize();
    MC_Initialize();
}
