
#include <mfapi.h>
#include "configuration.h"
#include "ipc.h"
#include "plugin.h"

#include "../hl2ss/server.h"
#include "../hl2ss/timestamps.h"
#include "../hl2ss/log.h"
#include "../hl2ss/nfo.h"
#include "../hl2ss/types.h"
#include "../hl2ss/locator.h"
#include "../hl2ss/research_mode.h"
#include "../hl2ss/spatial_input.h"
#include "../hl2ss/personal_video.h"
#include "../hl2ss/spatial_mapping.h"
#include "../hl2ss/scene_understanding.h"
#include "../hl2ss/voice_input.h"
#include "../hl2ss/stream_rm.h"
#include "../hl2ss/stream_mc.h"
#include "../hl2ss/stream_pv.h"
#include "../hl2ss/stream_si.h"
#include "../hl2ss/ipc_rc.h"
#include "../hl2ss/ipc_sm.h"
#include "../hl2ss/ipc_su.h"
#include "../hl2ss/ipc_vi.h"
#include "../hl2ss/stream_eet.h"
#include "../hl2ss/stream_ea.h"
#include "../hl2ss/stream_ev.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.UI.Core.h>
#include <winrt/Windows.ApplicationModel.Core.h>

using namespace winrt::Windows::UI::Core;
using namespace winrt::Windows::ApplicationModel::Core;

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
    if (enable & HL2SS_ENABLE_PV) { PersonalVideo_Initialize(); }
    if (enable & HL2SS_ENABLE_SI) { SpatialInput_Initialize(); }
    if (enable & HL2SS_ENABLE_SM) { SpatialMapping_Initialize(); }
    if (enable & HL2SS_ENABLE_SU) { SceneUnderstanding_Initialize(); }
    if (enable & HL2SS_ENABLE_VI) { VoiceInput_Initialize(); }

    if (enable & HL2SS_ENABLE_RM) { RM_Initialize(); }
    if (enable & HL2SS_ENABLE_PV) { PV_Initialize(); }
    if (enable & HL2SS_ENABLE_MC) { MC_Initialize(); }
    if (enable & HL2SS_ENABLE_SI) { SI_Initialize(); }
    if (enable & HL2SS_ENABLE_RC) { RC_Initialize(); }
    if (enable & HL2SS_ENABLE_SM) { SM_Initialize(); }
    if (enable & HL2SS_ENABLE_SU) { SU_Initialize(); }
    if (enable & HL2SS_ENABLE_VI) { VI_Initialize(); }
    if (enable & HL2SS_ENABLE_MQ) { MQ_Initialize(); }
    if (enable & HL2SS_ENABLE_EET) { EET_Initialize(); }
    if (enable & HL2SS_ENABLE_EA) { EA_Initialize(); }
    if (enable & HL2SS_ENABLE_EV) { EV_Initialize(); }
}

// OK
UNITY_EXPORT
void InitializeStreamsOnUI(uint32_t enable)
{
    HANDLE event_done = CreateEvent(NULL, TRUE, FALSE, NULL);
    CoreApplication::Views().GetAt(0).Dispatcher().RunAsync(CoreDispatcherPriority::High, [=]() { InitializeStreams(enable); SetEvent(event_done); }).get();
    WaitForSingleObject(event_done, INFINITE);
    CloseHandle(event_done);
}

// OK
UNITY_EXPORT
void DebugMessage(char const* str)
{
    ShowMessage("%s", str);
}

// OK
UNITY_EXPORT
void GetLocalIPv4Address(wchar_t *buffer, int size)
{
    std::vector<wchar_t> address;
    GetLocalIPv4Address(address);
    wcscpy_s(buffer, size / sizeof(wchar_t), address.data());
}

// OK
UNITY_EXPORT
int OverrideWorldCoordinateSystem(void* scs_ptr)
{
    winrt::Windows::Perception::Spatial::SpatialCoordinateSystem scs = nullptr;
    if (scs_ptr)
    {
    winrt::copy_from_abi(scs, scs_ptr);
    scs = Locator_SanitizeSpatialCoordinateSystem(scs);
    if (!scs) { return false; }
    }
    Locator_OverrideWorldCoordinateSystem(scs);
    return true;
}
