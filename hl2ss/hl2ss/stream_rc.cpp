
#include "server.h"
#include "ports.h"
#include "utilities.h"
#include "holographic_space.h"
#include "personal_video.h"

#include <winrt/Windows.ApplicationModel.h>

using namespace winrt::Windows::ApplicationModel;

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
    uint8_t state;
    
    ok = recv_u8(clientsocket, state);
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
static void RC_GetVersion(SOCKET clientsocket)
{
    PackageVersion version = Package::Current().Id().Version();
    uint16_t data[4] = { version.Major, version.Minor, version.Build, version.Revision };
    WSABUF wsaBuf;

    wsaBuf.buf = (char*)data;
    wsaBuf.len = sizeof(data);
  
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
    case 0: RC_EnableMarker(clientsocket);              break;
    case 1: RC_SetFocus(clientsocket);                  break;
    case 2: RC_SetVideoTemporalDenoising(clientsocket); break;
    case 3: RC_SetWhiteBalance_Preset(clientsocket);    break;
    case 4: RC_SetWhiteBalance_Value(clientsocket);     break;
    case 5: RC_SetExposure(clientsocket);               break;
    case 6: RC_GetVersion(clientsocket);                break;
    }
}

// OK
static DWORD WINAPI RC_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket;
    SOCKET clientsocket;

    listensocket = CreateSocket(PORT_RC);

    ShowMessage("RC: Listening at port %s", PORT_RC);

    do
    {
    ShowMessage("RC: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("RC: Client connected");

    RC_Translate(clientsocket);

    closesocket(clientsocket);

    ShowMessage("RC: Client Disconnected");
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
