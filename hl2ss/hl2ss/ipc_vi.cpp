
#include "voice_input.h"
#include "server_channel.h"

class Channel_VI : public Channel
{
private:
    bool Startup();
    void Run();
    void Cleanup();

    bool Dispatch();

    bool RX_Commands();
    bool TX_Commands();

public:
    Channel_VI(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_VI> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
bool Channel_VI::RX_Commands()
{
    std::vector<winrt::hstring> commands;
    uint16_t count;   
    std::vector<uint8_t> buffer;
    uint16_t stlen;
    bool ok;

    ok = recv_u16(m_socket_client, m_event_client, count);
    if (!ok) { return false; }

    for (int i = 0; i < count; ++i)
    {
    ok = recv_u16(m_socket_client, m_event_client, stlen);
    if (!ok) { return false; }
    if (stlen <= 0) { continue; }

    buffer.resize(stlen);

    ok = recv(m_socket_client, m_event_client, buffer.data(), stlen);
    if (!ok) { return false; }
    
    commands.push_back(winrt::hstring(reinterpret_cast<wchar_t*>(buffer.data()), stlen / sizeof(wchar_t)));
    }

    if (commands.size() <= 0) { return false; }

    return VoiceInput_RegisterCommands(commands);
}

// OK
bool Channel_VI::TX_Commands()
{
    uint32_t count;
    VoiceInput_Result result;
    WSABUF wsaBuf[1];
    bool ok;

    count = VoiceInput_GetCount();

    pack_buffer(wsaBuf, 0, &count, sizeof(count));

    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }

    pack_buffer(wsaBuf, 0, &result, sizeof(result));

    for (uint32_t i = 0; i < count; ++i)
    {
    result = VoiceInput_Pop();
    ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { return false; }
    }

    return true;
}

// OK
bool Channel_VI::Dispatch()
{
    uint8_t state;
    bool ok;

    ok = RX_Commands();
    if (!ok) { return false; }

    VoiceInput_Start();

    if (!VoiceInput_Status()) { return false; }

    do
    {
    ok = recv_u8(m_socket_client, m_event_client, state);
    if (!ok) { break; }
    if (state == 0) { break; }
    }
    while (TX_Commands());

    VoiceInput_Stop();

    return ok;
}

// OK
Channel_VI::Channel_VI(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_VI::Startup()
{
    SetNoDelay(true);
    return true;
}

// OK
void Channel_VI::Run()
{
    VoiceInput_Open();
    while (Dispatch());
    VoiceInput_Close();
}

// OK
void Channel_VI::Cleanup()
{
}

// OK
void VI_Startup()
{
    g_channel = std::make_unique<Channel_VI>("VI", PORT_NAME_VI, PORT_ID_VI);
}

// OK
void VI_Cleanup()
{
    g_channel.reset();
}
