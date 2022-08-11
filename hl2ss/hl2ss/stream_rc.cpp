
#include "server.h"
#include "ports.h"
#include "utilities.h"
#include "holographic_space.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_quitevent = NULL; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void RC_Translate(SOCKET clientsocket)
{
    uint8_t state;
    bool ok = recv_u8(clientsocket, state);
    if (ok) { HolographicSpace_EnableMarker(state != 0); }
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
