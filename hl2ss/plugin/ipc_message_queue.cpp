
#include <queue>
#include <malloc.h>
#include "../hl2ss/server.h"
#include "../hl2ss/utilities.h"
#include "plugin.h"

struct MQ_MSG
{
	uint32_t command;
	uint32_t size;
	void* data;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

HANDLE g_event_restart;
HANDLE g_event_so;
HANDLE g_event_error;
HANDLE g_event_quit;
CRITICAL_SECTION g_lock_si;
CRITICAL_SECTION g_lock_so;
std::queue<MQ_MSG> g_queue_si;
std::queue<uint32_t> g_queue_so;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

static DWORD WINAPI MQ_EntryPoint_Receive(void *param)
{
	SOCKET clientsocket = *((SOCKET*)param);
	bool ok;
	MQ_MSG msg;

	do
	{
	ok = recv_u32(clientsocket, msg.command);
	if (!ok) { break; }
	ok = recv_u32(clientsocket, msg.size);
	if (!ok) { break; }
	if (msg.size > 0)
	{
		msg.data = malloc(msg.size);
		ok = recv(clientsocket, (char*)msg.data, msg.size);
		if (!ok)
		{
			free(msg.data);
			break;
		}
	}
	else
	{
		msg.data = NULL;
	}

	{
	CriticalSection cs(&g_lock_si);
	g_queue_si.push(msg);
	}
	}
	while (WaitForSingleObject(g_event_error, 0) == WAIT_TIMEOUT);

	SetEvent(g_event_error);

	return 0;
}

UNITY_API
uint32_t MQ_SI_Peek()
{
	CriticalSection cs(&g_lock_si);
	return g_queue_si.empty() ? ~0UL : g_queue_si.front().size;
}

UNITY_API
void MQ_SI_Pop(uint32_t& command, uint8_t* data)
{
	MQ_MSG msg;

	{
	CriticalSection cs(&g_lock_si);
	msg = g_queue_si.front();
	g_queue_si.pop();
	}

	command = msg.command;
	if (msg.size <= 0) { return; }
	memcpy(data, msg.data, msg.size);
	free(msg.data);
}

static bool MQ_SO_Wait()
{
	HANDLE events[2];

	events[0] = g_event_so;
	events[1] = g_event_error;

	return WaitForMultipleObjects(sizeof(events) / sizeof(HANDLE), events, FALSE, INFINITE) == WAIT_OBJECT_0;
}

static DWORD WINAPI MQ_EntryPoint_Send(void *param)
{
	SOCKET clientsocket = *((SOCKET*)param);
	int status;
	bool empty;
	uint32_t id;

	do
	{
	{
	CriticalSection cs(&g_lock_so);
	empty = g_queue_so.empty();
	}

	if (empty && !MQ_SO_Wait()) { break; }

	{
	CriticalSection cs(&g_lock_so);
	id = g_queue_so.front();
	g_queue_so.pop();
	}

	status = send(clientsocket, (char*)&id, sizeof(id), 0);
	if (status != sizeof(id)) { break; }
	}
	while (WaitForSingleObject(g_event_error, 0) == WAIT_TIMEOUT);

	SetEvent(g_event_error);

	return 0;
}

UNITY_API
void MQ_SO_Push(uint32_t id)
{
	{
	CriticalSection cs(&g_lock_so);
	g_queue_so.push(id);
	}
	SetEvent(g_event_so);
}

UNITY_API
void MQ_Restart()
{
	SetEvent(g_event_restart);
}

static void MQ_Procedure(SOCKET clientsocket)
{
	HANDLE threads[2];
	MQ_MSG msg;
	
	threads[0] = CreateThread(NULL, 0, MQ_EntryPoint_Receive, &clientsocket, 0, NULL);
	threads[1] = CreateThread(NULL, 0, MQ_EntryPoint_Send,    &clientsocket, 0, NULL);

	WaitForMultipleObjects(sizeof(threads) / sizeof(HANDLE), threads, TRUE, INFINITE);

	CloseHandle(threads[0]);
	CloseHandle(threads[1]);

	msg.command = ~0UL;
	msg.size = 0;
	msg.data = NULL;

	{
	CriticalSection cs(&g_lock_si);
	g_queue_si.push(msg);
	}

	ShowMessage("MQ: Waiting for restart signal");
	WaitForSingleObject(g_event_restart, INFINITE);
	
	{
	CriticalSection cs(&g_lock_so);
	while (!g_queue_so.empty())	{ g_queue_so.pop(); }
	}

	ResetEvent(g_event_error);
	ResetEvent(g_event_so);

	ShowMessage("MQ: Restart signal received");
}

static DWORD WINAPI MQ_EntryPoint(void* param)
{
	(void)param;

	SOCKET listensocket = CreateSocket(PORT_MQ);
	SOCKET clientsocket;

	ShowMessage("MQ: Listening at port %s", PORT_MQ);

	do
	{
	ShowMessage("MQ: Waiting for client");

	clientsocket = accept(listensocket, NULL, NULL);
	if (clientsocket == INVALID_SOCKET) { break; }

	ShowMessage("MQ: Client connected");

	MQ_Procedure(clientsocket);

	closesocket(clientsocket);

	ShowMessage("MQ: Client disconnected");
	}
	while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

	closesocket(listensocket);

	ShowMessage("MQ: Closed");

	return 0;
}

void MQ_Initialize()
{
	g_event_quit    = CreateEvent(NULL, TRUE,  FALSE, NULL);
	g_event_error   = CreateEvent(NULL, TRUE,  FALSE, NULL);
	g_event_so      = CreateEvent(NULL, FALSE, FALSE, NULL);
	g_event_restart = CreateEvent(NULL, FALSE, FALSE, NULL);

	InitializeCriticalSection(&g_lock_so);
	InitializeCriticalSection(&g_lock_si);

	CreateThread(NULL, 0, MQ_EntryPoint, NULL, 0, NULL);
}
