
#pragma once

#include "server.h"
#include "server_ports.h"

class Channel
{
private:
    HANDLE m_event_quit; // CloseHandle
    HANDLE m_thread; // CloseHandle
    SOCKET m_socket_listen; // closesocket
    DWORD  m_no_delay;

    void Entry();
    void Loop();

    static DWORD WINAPI Thunk_Entry(void* self);

protected:    
    SOCKET m_socket_client; // closesocket
    HANDLE m_event_client; // CloseHandle

    virtual bool Startup() = 0;
    virtual void Run()     = 0;
    virtual void Cleanup() = 0;

    void SetNoDelay(bool no_delay);

public:
    char const* const m_name;
    char const* const m_port;
    uint32_t const m_id;

    Channel(char const* name, char const* port, uint32_t id);
    virtual ~Channel();
};
