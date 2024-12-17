
#include <memory>
#include "message_queue.h"
#include "server_channel.h"

class Channel_MQX : public Channel
{
private:
	bool Startup();
	void Run();
	void Cleanup();

	bool Dispatch();

public:
	Channel_MQX(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_MQX> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
bool Channel_MQX::Dispatch()
{
	MQ_Item* item; // MessageQueue_Item_Delete
	uint32_t id;
	WSABUF wsabuf[1];
	bool ok;

	ok = recv_u32(m_socket_client, m_event_client, id);
	if (!ok) { return false; }

	if (id != MQ_MARKER)
	{
	MessageQueue_Client_RX_Push(id);
	return true;
	}

	item = MessageQueue_Client_TX_Pull();
	pack_buffer(wsabuf, 0, item, sizeof(MQ_Item::command) + sizeof(MQ_Item::size) + item->size);
	ok = send_multiple(m_socket_client, m_event_client, wsabuf, sizeof(wsabuf) / sizeof(WSABUF));
	MessageQueue_Item_Delete(item);
	if (!ok) { return false; }

	return true;
}

// OK
Channel_MQX::Channel_MQX(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_MQX::Startup()
{
	SetNoDelay(true);
	return true;
}

// OK
void Channel_MQX::Run()
{
	while (Dispatch());
	MessageQueue_Client_WaitForRestart();
}

// OK
void Channel_MQX::Cleanup()
{
}

// OK
void MQX_Startup()
{
	g_channel = std::make_unique<Channel_MQX>("MQX", PORT_NAME_MQX, PORT_ID_MQX);
}

// OK
void MQX_Cleanup()
{
	g_channel.reset();
}
