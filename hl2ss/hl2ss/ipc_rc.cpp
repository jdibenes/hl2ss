
#include "server.h"
#include "ports.h"
#include "log.h"
#include "holographic_space.h"
#include "personal_video.h"
#include "extended_execution.h"
#include "timestamps.h"
#include "nfo.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle
static HANDLE g_event_client = NULL; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void RC_TransferError()
{
    SetEvent(g_event_client);
}

// OK
static void RC_MSG_GetApplicationVersion(SOCKET clientsocket)
{
    uint16_t data[4];
    WSABUF wsaBuf[1];
    bool ok;

    GetApplicationVersion(data);

    pack_buffer(wsaBuf, 0, data, sizeof(data));

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok)
    {
        RC_TransferError();
        return;
    }
}

// OK
static void RC_MSG_GetUTCOffset(SOCKET clientsocket)
{
    bool ok;
    uint32_t samples;
    UINT64 offset;
    WSABUF wsaBuf[1];

    ok = recv_u32(clientsocket, samples);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    offset = GetQPCToUTCOffset(samples);

    pack_buffer(wsaBuf, 0, &offset, sizeof(offset));

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok)
    {
        RC_TransferError();
        return;
    }
}

// OK
static void RC_MSG_SetHSMarkerState(SOCKET clientsocket)
{
    bool ok;
    uint32_t state;

    ok = recv_u32(clientsocket, state);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    HolographicSpace_EnableMarker(state != 0);
}

// OK
static void RC_MSG_GetPVSubsystemStatus(SOCKET clientsocket)
{
    bool status = PersonalVideo_Status();
    WSABUF wsaBuf[1];
    bool ok;

    pack_buffer(wsaBuf, 0, &status, sizeof(status));

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok)
    {
        RC_TransferError();
        return;
    }
}

// OK
static void RC_MSG_SetPVFocus(SOCKET clientsocket)
{
    bool ok;
    uint32_t focusmode;
    uint32_t autofocusrange;
    uint32_t distance;
    uint32_t value;
    uint32_t disabledriverfallback;

    ok = recv_u32(clientsocket, focusmode);
    if (!ok)
    {
        RC_TransferError();
        return;
    }
    ok = recv_u32(clientsocket, autofocusrange);
    if (!ok)
    {
        RC_TransferError();
        return;
    }
    ok = recv_u32(clientsocket, distance);
    if (!ok)
    {
        RC_TransferError();
        return;
    }
    ok = recv_u32(clientsocket, value);
    if (!ok)
    {
        RC_TransferError();
        return;
    }
    ok = recv_u32(clientsocket, disabledriverfallback);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetFocus(focusmode, autofocusrange, distance, value, disabledriverfallback);
}

// OK
static void RC_MSG_SetPVVideoTemporalDenoising(SOCKET clientsocket)
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(clientsocket, mode);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetVideoTemporalDenoising(mode);
}

// OK
static void RC_MSG_SetPVWhiteBalancePreset(SOCKET clientsocket)
{
    bool ok;
    uint32_t preset;

    ok = recv_u32(clientsocket, preset);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetWhiteBalance_Preset(preset);
}

// OK
static void RC_MSG_SetPVWhiteBalanceValue(SOCKET clientsocket)
{
    bool ok;
    uint32_t value;

    ok = recv_u32(clientsocket, value);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetWhiteBalance_Value(value);
}

// OK
static void RC_MSG_SetPVExposure(SOCKET clientsocket)
{
    bool ok;
    uint32_t mode;
    uint32_t value;

    ok = recv_u32(clientsocket, mode);
    if (!ok)
    {
        RC_TransferError();
        return;
    }
    ok = recv_u32(clientsocket, value);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetExposure(mode, value);
}

// OK
static void RC_MSG_SetPVExposurePriorityVideo(SOCKET clientsocket)
{
    bool ok;
    uint32_t enabled;

    ok = recv_u32(clientsocket, enabled);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetExposurePriorityVideo(enabled);
}

// OK
static void RC_MSG_SetPVIsoSpeed(SOCKET clientsocket)
{
    bool ok;
    uint32_t setauto;
    uint32_t value;

    ok = recv_u32(clientsocket, setauto);
    if (!ok)
    {
        RC_TransferError();
        return;
    }
    ok = recv_u32(clientsocket, value);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetIsoSpeed(setauto, value);
}

// OK
static void RC_MSG_SetPVBacklightCompensation(SOCKET clientsocket)
{
    bool ok;
    uint32_t state;

    ok = recv_u32(clientsocket, state);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetBacklightCompensation(state != 0);
}

// OK
static void RC_MSG_SetPVSceneMode(SOCKET clientsocket)
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(clientsocket, mode);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    PersonalVideo_SetSceneMode(mode);
}

// OK
static void RC_MSG_SetFlatMode(SOCKET clientsocket)
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(clientsocket, mode);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    ExtendedExecution_SetFlatMode(mode != 0);
}

// OK
static void RC_Dispatch(SOCKET clientsocket)
{
    uint8_t state;
    bool ok;

    ok = recv_u8(clientsocket, state);
    if (!ok)
    {
        RC_TransferError();
        return;
    }

    switch (state)
    {
    case 0x00: RC_MSG_GetApplicationVersion(clientsocket);       break;
    case 0x01: RC_MSG_GetUTCOffset(clientsocket);                break;
    case 0x02: RC_MSG_SetHSMarkerState(clientsocket);            break;
    case 0x03: RC_MSG_GetPVSubsystemStatus(clientsocket);        break;
    case 0x04: RC_MSG_SetPVFocus(clientsocket);                  break;
    case 0x05: RC_MSG_SetPVVideoTemporalDenoising(clientsocket); break;
    case 0x06: RC_MSG_SetPVWhiteBalancePreset(clientsocket);     break;
    case 0x07: RC_MSG_SetPVWhiteBalanceValue(clientsocket);      break;
    case 0x08: RC_MSG_SetPVExposure(clientsocket);               break;
    case 0x09: RC_MSG_SetPVExposurePriorityVideo(clientsocket);  break;
    case 0x0A: RC_MSG_SetPVIsoSpeed(clientsocket);               break;
    case 0x0B: RC_MSG_SetPVBacklightCompensation(clientsocket);  break;
    case 0x0C: RC_MSG_SetPVSceneMode(clientsocket);              break;
    case 0x0D: RC_MSG_SetFlatMode(clientsocket);                 break;
    default:
        RC_TransferError();
        return;
    }
}

// OK
static void RC_Translate(SOCKET clientsocket)
{
    ResetEvent(g_event_client);
    do { RC_Dispatch(clientsocket); } while (WaitForSingleObject(g_event_client, 0) == WAIT_TIMEOUT);
}

// OK
static DWORD WINAPI RC_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket;
    SOCKET clientsocket;

    listensocket = CreateSocket(PORT_NAME_RC);

    ShowMessage("RC: Listening at port %s", PORT_NAME_RC);

    do
    {
    ShowMessage("RC: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("RC: Client connected");

    RC_Translate(clientsocket);

    closesocket(clientsocket);

    ShowMessage("RC: Client disconnected");
    }
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("RC: Closed");

    return 0;
}

// OK
void RC_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, RC_EntryPoint, NULL, 0, NULL);
}

// OK
void RC_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void RC_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);

    g_thread = NULL;
    g_event_client = NULL;
    g_event_quit = NULL;
}
