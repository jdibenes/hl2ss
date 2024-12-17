
#include <memory>
#include "message_queue.h"
#include "server_channel.h"
#include "lock.h"

class Channel_MQ : public Channel
{
private:
	bool Startup();
	void Run();
	void Cleanup();

	void Loop_RX();
	void Loop_TX();

	static DWORD WINAPI Thunk_RX(void* self);
	static DWORD WINAPI Thunk_TX(void* self);

public:
	Channel_MQ(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_MQ> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
DWORD Channel_MQ::Thunk_RX(void* self)
{
	static_cast<Channel_MQ*>(self)->Loop_RX();
	return 0;
}

// OK
DWORD Channel_MQ::Thunk_TX(void* self)
{
	static_cast<Channel_MQ*>(self)->Loop_TX();
	return 0;
}

// OK
void Channel_MQ::Loop_RX()
{
	MQ_Item* item; // MessageQueue_Item_Delete
	uint32_t command;
	uint32_t size;
	bool ok;	

	do
	{
	ok = recv_u32(m_socket_client, m_event_client, command);
	if (!ok) { break; }
	ok = recv_u32(m_socket_client, m_event_client, size);
	if (!ok) { break; }

	item = MessageQueue_Item_Create(command, size);

	Cleaner free_item([=]() { MessageQueue_Item_Delete(item); });

	if (size > 0)
	{
	ok = recv(m_socket_client, m_event_client, item->data, size);
	if (!ok) { break; }
	}

	free_item.Set(false);

	MessageQueue_Server_RX_Push(item);
	}
	while (WaitForSingleObject(m_event_client, 0) == WAIT_TIMEOUT);

	SetEvent(m_event_client);
}

// OK
void Channel_MQ::Loop_TX()
{
	uint32_t id;
	WSABUF wsaBuf[1];
	bool ok;

	do
	{
	ok = MessageQueue_Server_TX_Pull(m_event_client, id);
	if (!ok) { break; }

	pack_buffer(wsaBuf, 0, &id, sizeof(id));

	ok = send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
	if (!ok) { break; }
	}
	while (WaitForSingleObject(m_event_client, 0) == WAIT_TIMEOUT);

	SetEvent(m_event_client);
}

// OK
Channel_MQ::Channel_MQ(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_MQ::Startup()
{
	SetNoDelay(true);
	return true;
}

// OK
void Channel_MQ::Run()
{
	HANDLE threads[2];

	threads[0] = CreateThread(NULL, 0, Thunk_RX, this, 0, NULL);
	threads[1] = CreateThread(NULL, 0, Thunk_TX, this, 0, NULL);

	WaitForMultipleObjects(sizeof(threads) / sizeof(HANDLE), threads, TRUE, INFINITE);

	CloseHandle(threads[0]);
	CloseHandle(threads[1]);

	MessageQueue_Server_WaitForRestart();
}

// OK
void Channel_MQ::Cleanup()
{
}

// OK
void MQ_Startup()
{
	g_channel = std::make_unique<Channel_MQ>("MQ", PORT_NAME_MQ, PORT_ID_MQ);
}

// OK
void MQ_Cleanup()
{
	g_channel.reset();
}
