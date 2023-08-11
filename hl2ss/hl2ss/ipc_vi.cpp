
#include "server.h"
#include "ports.h"
#include "voice_input.h"
#include "log.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

HANDLE g_thread = NULL;
HANDLE g_event_quit = NULL;
HANDLE g_event_client = NULL;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void VI_TransferError()
{
    SetEvent(g_event_client);
}

// OK
static void VI_MSG_CreateRecognizer(SOCKET clientsocket)
{
    (void)clientsocket;
    VoiceInput_CreateRecognizer();
}

// OK
static void VI_MSG_RegisterCommands(SOCKET clientsocket)
{
    uint8_t clear;
    uint8_t count;
    std::vector<winrt::hstring> commands;
    std::vector<uint8_t> buffer;
    uint16_t stlen;
    uint8_t result;
    WSABUF wsaBuf[1];
    bool ok;

    ok = recv_u8(clientsocket, clear);
    if (!ok)
    {
        VI_TransferError();
        return;
    }
    ok = recv_u8(clientsocket, count);
    if (!ok)
    {
        VI_TransferError();
        return;
    }

    for (int i = 0; i < count; ++i)
    {
    ok = recv_u16(clientsocket, stlen);
    if (!ok)
    {
        VI_TransferError();
        return;
    }
    if (stlen <= 0) { continue; }

    buffer.resize(stlen);

    ok = recv(clientsocket, (char*)buffer.data(), (int)stlen);
    if (!ok)
    {
        VI_TransferError();
        return;
    }
    
    commands.push_back(winrt::hstring((wchar_t*)buffer.data(), stlen / sizeof(wchar_t)));
    }

    result = VoiceInput_RegisterCommands(commands, clear != 0);

    pack_buffer(wsaBuf, 0, &result, sizeof(result));
    
    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok)
    {
        VI_TransferError();
        return;
    }
}

// OK
static void VI_MSG_Start(SOCKET clientsocket)
{
    (void)clientsocket;
    VoiceInput_Start();
}

// OK
static void VI_MSG_Pop(SOCKET clientsocket)
{
    uint32_t count;
    VoiceInput_Result result;
    WSABUF wsaBuf[1];
    bool ok;

    count = (uint32_t)VoiceInput_GetCount();

    pack_buffer(wsaBuf, 0, &count, sizeof(count));

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok)
    {
        VI_TransferError();
        return;
    }

    pack_buffer(wsaBuf, 0, &result, sizeof(result));

    for (uint32_t i = 0; i < count; ++i)
    {
    result = VoiceInput_Pop();
    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok)
    {
        VI_TransferError();
        return;
    }
    }
}

// OK
static void VI_MSG_Clear(SOCKET clientsocket)
{
    (void)clientsocket;
    VoiceInput_Clear();
}

// OK
static void VI_MSG_Stop(SOCKET clientsocket)
{
    (void)clientsocket;
    VoiceInput_Stop();
}

// OK
static void VI_Dispatch(SOCKET clientsocket)
{
    uint8_t state;
    bool ok;

    ok = recv_u8(clientsocket, state);
    if (!ok)
    {
        VI_TransferError();
        return;
    }

    switch (state)
    {
    case 0x00: VI_MSG_CreateRecognizer(clientsocket); break;
    case 0x01: VI_MSG_RegisterCommands(clientsocket); break;
    case 0x02: VI_MSG_Start(clientsocket);            break;
    case 0x03: VI_MSG_Pop(clientsocket);              break;
    case 0x04: VI_MSG_Clear(clientsocket);            break;
    case 0x05: VI_MSG_Stop(clientsocket);             break;
    default:
        VI_TransferError();
        return;
    }
}

// OK
static void VI_Translate(SOCKET clientsocket)
{
    ResetEvent(g_event_client);
    do { VI_Dispatch(clientsocket); } while (WaitForSingleObject(g_event_client, 0) == WAIT_TIMEOUT);
    if (!VoiceInput_IsRunning()) { return; }
    VoiceInput_Stop();
    VoiceInput_Clear();
}

// OK
static DWORD WINAPI VI_EntryPoint(void *param)
{
    (void)param;

    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    listensocket = CreateSocket(PORT_NAME_VI);

    ShowMessage("VI: Listening at port %s", PORT_NAME_VI);

    do
    {
    ShowMessage("VI: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("VI: Client connected");

    VI_Translate(clientsocket);

    closesocket(clientsocket);

    ShowMessage("VI: Client disconnected");
    }
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("VI: Closed");

    return 0;
}

// OK
void VI_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, VI_EntryPoint, NULL, 0, NULL);
}

// OK
void VI_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void VI_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);
    CloseHandle(g_thread);
    CloseHandle(g_event_client);
    CloseHandle(g_event_quit);
}
