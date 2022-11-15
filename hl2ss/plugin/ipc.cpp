
#include <queue>
#include <malloc.h>
#include "plugin.h"
#include "ports.h"

#include "../hl2ss/server.h"
#include "../hl2ss/lock.h"
#include "../hl2ss/log.h"

struct MQ_MSG
{
	uint32_t command;
	uint32_t size;
	void* data;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread; // CloseHandle
static HANDLE g_event_restart; // CloseHandle
static HANDLE g_semaphore_so; // CloseHandle
static HANDLE g_event_error; // CloseHandle
static HANDLE g_event_quit; // CloseHandle
static CRITICAL_SECTION g_lock_si; // DeleteCriticalSection
static CRITICAL_SECTION g_lock_so; // DeleteCriticalSection
static std::queue<MQ_MSG> g_queue_si;
static std::queue<uint32_t> g_queue_so;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
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

// OK
UNITY_EXPORT
uint32_t MQ_SI_Peek()
{
	CriticalSection cs(&g_lock_si);
	return g_queue_si.empty() ? ~0UL : g_queue_si.front().size;
}

// OK
UNITY_EXPORT
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

// OK
static bool MQ_SO_Wait()
{
	HANDLE events[2];

	events[0] = g_semaphore_so;
	events[1] = g_event_error;

	return WaitForMultipleObjects(sizeof(events) / sizeof(HANDLE), events, FALSE, INFINITE) == WAIT_OBJECT_0;
}

// OK
static DWORD WINAPI MQ_EntryPoint_Send(void *param)
{
	SOCKET clientsocket = *((SOCKET*)param);
	int status;
	uint32_t id;

	do
	{
	if (!MQ_SO_Wait()) { break; }

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

// OK
UNITY_EXPORT
void MQ_SO_Push(uint32_t id)
{
	CriticalSection cs(&g_lock_so);
	g_queue_so.push(id);
	ReleaseSemaphore(g_semaphore_so, 1, NULL);
}

// OK
UNITY_EXPORT
void MQ_Restart()
{
	SetEvent(g_event_restart);
}

// OK
static void MQ_Procedure(SOCKET clientsocket)
{
	HANDLE threads[2];
	MQ_MSG msg;

	g_semaphore_so = CreateSemaphore(NULL, 0, LONG_MAX, NULL);
	
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

	g_queue_so = {};
	CloseHandle(g_semaphore_so);	

	ResetEvent(g_event_error);

	ShowMessage("MQ: Restart signal received");
}

// OK
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

// OK
void MQ_Initialize()
{
	InitializeCriticalSection(&g_lock_so);
	InitializeCriticalSection(&g_lock_si);

	g_event_quit    = CreateEvent(NULL, TRUE,  FALSE, NULL);
	g_event_error   = CreateEvent(NULL, TRUE,  FALSE, NULL);
	g_event_restart = CreateEvent(NULL, FALSE, FALSE, NULL);
	
	g_thread = CreateThread(NULL, 0, MQ_EntryPoint, NULL, 0, NULL);
}

// OK
void MQ_Quit()
{
	SetEvent(g_event_quit);
}

// OK
void MQ_Cleanup()
{
	WaitForSingleObject(g_thread, INFINITE);

	CloseHandle(g_thread);

	CloseHandle(g_event_restart);
	CloseHandle(g_event_error);
	CloseHandle(g_event_quit);

	DeleteCriticalSection(&g_lock_si);
	DeleteCriticalSection(&g_lock_so);
}
