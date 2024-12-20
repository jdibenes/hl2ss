
#include "server_channel.h"
#include "extended_execution.h"
#include "lock.h"
#include "log.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
DWORD Channel::Thunk_Entry(void* self)
{
    static_cast<Channel*>(self)->Entry();
    return 0;
}

// OK
Channel::Channel(char const* name, char const* port, uint32_t id) : 
m_name(name), 
m_port(port), 
m_id(id)
{
    m_no_delay   = FALSE;

    m_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    m_thread     = CreateThread(NULL, 0, Thunk_Entry, this, 0, NULL);
}

// OK
Channel::~Channel()
{
    SetEvent(m_event_quit);
    WaitForSingleObject(m_thread, INFINITE);
    CloseHandle(m_thread);
    CloseHandle(m_event_quit);
}

// OK
void Channel::Entry()
{
    m_socket_listen = Server_CreateSocket(m_port);
    if (m_socket_listen == INVALID_SOCKET) { return; }

    bool ok = Startup();
    if (ok) { Loop(); }
    Cleanup();

    closesocket(m_socket_listen);
}

// OK
void Channel::Loop()
{
    int base_priority;

    ShowMessage("%s: Listening at port %s", m_name, m_port);

    m_event_client = CreateEvent(NULL, TRUE, FALSE, NULL);

    base_priority = GetThreadPriority(GetCurrentThread());

    do
    {
    ShowMessage("%s: Waiting for client", m_name);

    m_socket_client = Server_AcceptClient(m_socket_listen, m_no_delay);
    if (m_socket_client == INVALID_SOCKET) { break; }

    ShowMessage("%s: Client connected", m_name);

    SetThreadPriority(GetCurrentThread(), ExtendedExecution_GetInterfacePriority(m_id));

    Run();

    ResetEvent(m_event_client);

    SetThreadPriority(GetCurrentThread(), base_priority);

    closesocket(m_socket_client);

    ShowMessage("%s: Client disconnected", m_name);
    }
    while (WaitForSingleObject(m_event_quit, 0) == WAIT_TIMEOUT);

    CloseHandle(m_event_client);

    ShowMessage("%s: Closed", m_name);
}

void Channel::SetNoDelay(bool no_delay)
{
    m_no_delay = no_delay * TRUE;
}
