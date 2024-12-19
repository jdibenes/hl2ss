
#include "timestamp.h"
#include "holographic_space.h"
#include "extended_execution.h"
#include "personal_video.h"
#include "research_mode.h"
#include "server_channel.h"
#include "types.h"

class Channel_RC : public Channel
{
private:
    bool Startup();
    void Run();
    void Cleanup();

    bool Dispatch();

    bool MSG_EE_GetApplicationVersion();
    bool MSG_TS_GetUTCOffset();
    bool MSG_HS_SetMarkerState();
    bool MSG_PV_GetSubsystemStatus();
    bool MSG_PV_SetFocus();
    bool MSG_PV_SetVideoTemporalDenoising();
    bool MSG_PV_SetWhiteBalancePreset();
    bool MSG_PV_SetWhiteBalanceValue();
    bool MSG_PV_SetExposure();
    bool MSG_PV_SetExposurePriorityVideo();
    bool MSG_PV_SetIsoSpeed();
    bool MSG_PV_SetBacklightCompensation();
    bool MSG_PV_SetSceneMode();
    bool MSG_EE_SetFlatMode();
    bool MSG_RM_SetEyeSelection();
    bool MSG_PV_SetDesiredOptimization();
    bool MSG_PV_SetPrimaryUse();
    bool MSG_PV_SetOpticalImageStabilization();
    bool MSG_PV_SetHdrVideo();
    bool MSG_PV_SetRegionsOfInterest();
    bool MSG_EE_SetInterfacePriority();
    bool MSG_EE_SetQuietMode();

public:
    Channel_RC(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_RC> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
bool Channel_RC::MSG_EE_GetApplicationVersion()
{
    uint16_t data[4];
    WSABUF wsaBuf[1];
    bool ok;

    ExtendedExecution_GetApplicationVersion(data);

    pack_buffer(wsaBuf, 0, data, sizeof(data));

    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }

    return true;
}

// OK
bool Channel_RC::MSG_TS_GetUTCOffset()
{
    bool ok;
    UINT64 offset;
    WSABUF wsaBuf[1];

    offset = Timestamp_GetQPCToUTCOffset();

    pack_buffer(wsaBuf, 0, &offset, sizeof(offset));

    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }

    return true;
}

// OK
bool Channel_RC::MSG_HS_SetMarkerState()
{
    bool ok;
    uint32_t state;

    ok = recv_u32(m_socket_client, m_event_client, state);
    if (!ok) { return false; }

    HolographicSpace_EnableMarker(state != 0);

    return true;
}

// OK
bool Channel_RC::MSG_PV_GetSubsystemStatus()
{
    bool status = PersonalVideo_Status();
    WSABUF wsaBuf[1];
    bool ok;

    pack_buffer(wsaBuf, 0, &status, sizeof(status));

    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetFocus()
{
    bool ok;
    uint32_t focusmode;
    uint32_t autofocusrange;
    uint32_t distance;
    uint32_t value;
    uint32_t disabledriverfallback;

    ok = recv_u32(m_socket_client, m_event_client, focusmode);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, autofocusrange);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, distance);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, value);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, disabledriverfallback);
    if (!ok) { return false; }

    PersonalVideo_SetFocus(focusmode, autofocusrange, distance, value, disabledriverfallback);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetVideoTemporalDenoising()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }

    PersonalVideo_SetVideoTemporalDenoising(mode);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetWhiteBalancePreset()
{
    bool ok;
    uint32_t preset;

    ok = recv_u32(m_socket_client, m_event_client, preset);
    if (!ok) { return false; }

    PersonalVideo_SetWhiteBalance_Preset(preset);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetWhiteBalanceValue()
{
    bool ok;
    uint32_t value;

    ok = recv_u32(m_socket_client, m_event_client, value);
    if (!ok) { return false; }

    PersonalVideo_SetWhiteBalance_Value(value);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetExposure()
{
    bool ok;
    uint32_t mode;
    uint32_t value;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, value);
    if (!ok) { return false; }

    PersonalVideo_SetExposure(mode, value);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetExposurePriorityVideo()
{
    bool ok;
    uint32_t enabled;

    ok = recv_u32(m_socket_client, m_event_client, enabled);
    if (!ok) { return false; }

    PersonalVideo_SetExposurePriorityVideo(enabled);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetIsoSpeed()
{
    bool ok;
    uint32_t setauto;
    uint32_t value;

    ok = recv_u32(m_socket_client, m_event_client, setauto);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, value);
    if (!ok) { return false; }

    PersonalVideo_SetIsoSpeed(setauto, value);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetBacklightCompensation()
{
    bool ok;
    uint32_t state;

    ok = recv_u32(m_socket_client, m_event_client, state);
    if (!ok) { return false; }

    PersonalVideo_SetBacklightCompensation(state != 0);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetSceneMode()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }

    PersonalVideo_SetSceneMode(mode);

    return true;
}

// OK
bool Channel_RC::MSG_EE_SetFlatMode()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }

    ExtendedExecution_SetFlatMode(mode != 0);

    return true;
}

// OK
bool Channel_RC::MSG_RM_SetEyeSelection()
{
    bool ok;
    uint32_t enable;

    ok = recv_u32(m_socket_client, m_event_client, enable);
    if (!ok) { return false; }

    ResearchMode_SetEyeSelection(enable != 0);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetDesiredOptimization()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }
    
    PersonalVideo_SetDesiredOptimization(mode);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetPrimaryUse()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }

    PersonalVideo_SetPrimaryUse(mode);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetOpticalImageStabilization()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }

    PersonalVideo_SetOpticalImageStabilization(mode);

    return true;
}

// OK 
bool Channel_RC::MSG_PV_SetHdrVideo()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }

    PersonalVideo_SetHdrVideo(mode);

    return true;
}

// OK
bool Channel_RC::MSG_PV_SetRegionsOfInterest()
{
    bool ok;
    uint32_t mode;
    float x;
    float y;
    float w;
    float h;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, reinterpret_cast<uint32_t&>(x));
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, reinterpret_cast<uint32_t&>(y));
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, reinterpret_cast<uint32_t&>(w));
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, reinterpret_cast<uint32_t&>(h));
    if (!ok) { return false; }

    bool     clear             = bit_test( mode, 12);
    bool     set               = bit_test( mode, 11);
    bool     auto_exposure     = bit_test( mode, 10);
    bool     auto_focus        = bit_test( mode,  9);
    bool     bounds_normalized = bit_test( mode,  8);
    uint32_t type              = bit_field(mode,  7, 0x01);
    uint32_t weight            = bit_field(mode,  0, 0x7F);

    PersonalVideo_SetRegionsOfInterest(clear, set, auto_exposure, auto_focus, bounds_normalized, x, y, w, h, type, weight);

    return true;
}

// OK
bool Channel_RC::MSG_EE_SetInterfacePriority()
{
    bool ok;
    uint32_t id;
    uint32_t priority;

    ok = recv_u32(m_socket_client, m_event_client, id);
    if (!ok) { return false; }
    ok = recv_u32(m_socket_client, m_event_client, priority);
    if (!ok) { return false; }

    ExtendedExecution_SetInterfacePriority(id - PORT_NUMBER_BASE, priority);

    return true;
}

// OK
bool Channel_RC::MSG_EE_SetQuietMode()
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(m_socket_client, m_event_client, mode);
    if (!ok) { return false; }

    ExtendedExecution_SetQuietMode(mode != 0);

    return true;
}

// OK
bool Channel_RC::Dispatch()
{
    uint8_t state;
    bool ok;

    ok = recv_u8(m_socket_client, m_event_client, state);
    if (!ok) { return false; }

    switch (state)
    {
    case 0x00: return MSG_EE_GetApplicationVersion();
    case 0x01: return MSG_TS_GetUTCOffset();
    case 0x02: return MSG_HS_SetMarkerState();
    case 0x03: return MSG_PV_GetSubsystemStatus();
    case 0x04: return MSG_PV_SetFocus();
    case 0x05: return MSG_PV_SetVideoTemporalDenoising();
    case 0x06: return MSG_PV_SetWhiteBalancePreset();
    case 0x07: return MSG_PV_SetWhiteBalanceValue();
    case 0x08: return MSG_PV_SetExposure();
    case 0x09: return MSG_PV_SetExposurePriorityVideo();
    case 0x0A: return MSG_PV_SetIsoSpeed();
    case 0x0B: return MSG_PV_SetBacklightCompensation();
    case 0x0C: return MSG_PV_SetSceneMode();
    case 0x0D: return MSG_EE_SetFlatMode();
    case 0x0E: return MSG_RM_SetEyeSelection();
    case 0x0F: return MSG_PV_SetDesiredOptimization();
    case 0x10: return MSG_PV_SetPrimaryUse();
    case 0x11: return MSG_PV_SetOpticalImageStabilization();
    case 0x12: return MSG_PV_SetHdrVideo();
    case 0x13: return MSG_PV_SetRegionsOfInterest();
    case 0x14: return MSG_EE_SetInterfacePriority();
    case 0x15: return MSG_EE_SetQuietMode();
    default:   return false;
    }
}

// OK
Channel_RC::Channel_RC(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_RC::Startup()
{
    SetNoDelay(true);
    return true;
}

// OK
void Channel_RC::Run()
{
    while (Dispatch());
}

// OK
void Channel_RC::Cleanup()
{
}

// OK
void RC_Startup()
{
    g_channel = std::make_unique<Channel_RC>("RC", PORT_NAME_RC, PORT_ID_RC);
}

// OK
void RC_Cleanup()
{
    g_channel.reset();
}
