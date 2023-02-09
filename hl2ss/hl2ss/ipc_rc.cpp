
#include "server.h"
#include "ports.h"
#include "log.h"
#include "holographic_space.h"
#include "personal_video.h"
#include "timestamps.h"
#include "nfo.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_quitevent = NULL; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void RC_EnableMarker(SOCKET clientsocket)
{
    bool ok;
    uint32_t state;
    
    ok = recv_u32(clientsocket, state);
    if (!ok) { return; }

    HolographicSpace_EnableMarker(state != 0);
}

// OK
static void RC_SetFocus(SOCKET clientsocket)
{
    bool ok;
    uint32_t focusmode;
    uint32_t autofocusrange;
    uint32_t distance;
    uint32_t value;
    uint32_t disabledriverfallback;

    ok = recv_u32(clientsocket, focusmode);
    if (!ok) { return; }
    ok = recv_u32(clientsocket, autofocusrange);
    if (!ok) { return; }
    ok = recv_u32(clientsocket, distance);
    if (!ok) { return; }
    ok = recv_u32(clientsocket, value);
    if (!ok) { return; }
    ok = recv_u32(clientsocket, disabledriverfallback);
    if (!ok) { return; }

    PersonalVideo_SetFocus(focusmode, autofocusrange, distance, value, disabledriverfallback);
}

// OK
static void RC_SetVideoTemporalDenoising(SOCKET clientsocket)
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(clientsocket, mode);
    if (!ok) { return; }

    PersonalVideo_SetVideoTemporalDenoising(mode);
}

// OK
static void RC_SetWhiteBalance_Preset(SOCKET clientsocket)
{
    bool ok;
    uint32_t preset;

    ok = recv_u32(clientsocket, preset);
    if (!ok) { return; }

    PersonalVideo_SetWhiteBalance_Preset(preset);
}

// OK
static void RC_SetWhiteBalance_Value(SOCKET clientsocket)
{
    bool ok;
    uint32_t value;

    ok = recv_u32(clientsocket, value);
    if (!ok) { return; }

    PersonalVideo_SetWhiteBalance_Value(value);
}

// OK
static void RC_SetExposure(SOCKET clientsocket)
{
    bool ok;
    uint32_t mode;
    uint32_t value;

    ok = recv_u32(clientsocket, mode);
    if (!ok) { return; }
    ok = recv_u32(clientsocket, value);
    if (!ok) { return; }

    PersonalVideo_SetExposure(mode, value);
}

// OK
static void RC_GetApplicationVersion(SOCKET clientsocket)
{
    uint16_t data[4];
    WSABUF wsaBuf;

    GetApplicationVersion(data);    

    wsaBuf.buf = (char*)data;
    wsaBuf.len = sizeof(data);
  
    send_multiple(clientsocket, &wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
static void RC_GetUTCOffset(SOCKET clientsocket)
{
    bool ok;
    uint32_t samples;
    UINT64 offset;
    WSABUF wsaBuf;
    
    ok = recv_u32(clientsocket, samples);
    if (!ok) { return; }

    offset = GetQPCToUTCOffset(samples);

    wsaBuf.buf = (char*)&offset;
    wsaBuf.len = sizeof(offset);

    send_multiple(clientsocket, &wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
static void RC_SetExposurePriorityVideo(SOCKET clientsocket)
{
    bool ok;
    uint32_t enabled;

    ok = recv_u32(clientsocket, enabled);
    if (!ok) { return; }

    PersonalVideo_SetExposurePriorityVideo(enabled);
}

// OK
static void RC_SetSceneMode(SOCKET clientsocket)
{
    bool ok;
    uint32_t mode;

    ok = recv_u32(clientsocket, mode);
    if (!ok) { return; }

    PersonalVideo_SetSceneMode(mode);
}

// OK
static void RC_SetIsoSpeed(SOCKET clientsocket)
{
    bool ok;
    uint32_t setauto;
    uint32_t value;

    ok = recv_u32(clientsocket, setauto);
    if (!ok) { return; }

    ok = recv_u32(clientsocket, value);
    if (!ok) { return; }

    PersonalVideo_SetIsoSpeed(setauto, value);
}

// OK
static void RC_GetSubsystemStatus(SOCKET clientsocket)
{
    bool status = PersonalVideo_Status();
    WSABUF wsaBuf;

    wsaBuf.len = sizeof(status);
    wsaBuf.buf = (char*)&status;

    send_multiple(clientsocket, &wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
static void RC_Translate(SOCKET clientsocket)
{
    uint8_t state;
    bool ok;

    ok = recv_u8(clientsocket, state);
    if (!ok) { return; }

    switch (state)
    {
    case 0x00: RC_EnableMarker(clientsocket);              break;
    case 0x01: RC_SetFocus(clientsocket);                  break;
    case 0x02: RC_SetVideoTemporalDenoising(clientsocket); break;
    case 0x03: RC_SetWhiteBalance_Preset(clientsocket);    break;
    case 0x04: RC_SetWhiteBalance_Value(clientsocket);     break;
    case 0x05: RC_SetExposure(clientsocket);               break;
    case 0x06: RC_GetApplicationVersion(clientsocket);     break;
    case 0x07: RC_GetUTCOffset(clientsocket);              break;
    case 0x08: RC_SetExposurePriorityVideo(clientsocket);  break;
    case 0x09: RC_SetSceneMode(clientsocket);              break;
    case 0x0A: RC_SetIsoSpeed(clientsocket);               break;
    case 0x0B: RC_GetSubsystemStatus(clientsocket);        break;
    }
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
    while (WaitForSingleObject(g_quitevent, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("RC: Closed");

    return 0;
}

// OK
void RC_Initialize()
{
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, RC_EntryPoint, NULL, 0, NULL);
}

// OK
void RC_Quit()
{
    SetEvent(g_quitevent);
}

// OK
void RC_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_quitevent);

    g_thread = NULL;
    g_quitevent = NULL;
}
