
#include "message_queue.h"
#include "queue.h"
#include "lock.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

// MQ -------------------------------------------------------------------------
static CRITICAL_SECTION m_lock_si; // DeleteCriticalSection
static CRITICAL_SECTION m_lock_so; // DeleteCriticalSection
static std::queue<MQ_Item*> m_queue_si;
static std::queue<uint32_t> m_queue_so;
static HANDLE m_semaphore_so = NULL; // CloseHandle
static HANDLE m_event_restart_s = NULL; // CloseHandle

// MQX ------------------------------------------------------------------------
static CRITICAL_SECTION g_lock_ci; // DeleteCriticalSection
static CRITICAL_SECTION g_lock_co; // DeleteCriticalSection
static HANDLE g_event_restart_c = NULL; // CloseHandle
static std::queue<MQ_Item*> g_queue_ci;
static std::queue<uint32_t> g_queue_co;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
MQ_Item::MQ_Item()
{
}

// OK
MQ_Item::~MQ_Item()
{
}

// OK
MQ_Item* MessageQueue_Item_Create(uint32_t command, uint32_t size)
{
    MQ_Item* item = reinterpret_cast<MQ_Item*>(new uint32_t[(sizeof(MQ_Item::command) + sizeof(MQ_Item::size) + size + sizeof(uint32_t) - 1) / sizeof(uint32_t)]); // delete[]

    item->command = command;
    item->size    = size;

    return item;
}

// OK
void MessageQueue_Item_Delete(MQ_Item* item)
{
    delete[] reinterpret_cast<uint32_t*>(item);
}

//-----------------------------------------------------------------------------
// MQ
//-----------------------------------------------------------------------------

// OK
void MessageQueue_Server_Startup()
{
    InitializeCriticalSection(&m_lock_so);
    InitializeCriticalSection(&m_lock_si);

    m_semaphore_so    = CreateSemaphore(NULL, 0, LONG_MAX, NULL);
    m_event_restart_s = CreateEvent(NULL, FALSE, FALSE, NULL);
}

// OK
void MessageQueue_Server_Cleanup()
{
    CloseHandle(m_event_restart_s);
    CloseHandle(m_semaphore_so);
    m_event_restart_s = NULL;
    m_semaphore_so = NULL;
    DeleteCriticalSection(&m_lock_si);
    DeleteCriticalSection(&m_lock_so);
}

// OK
void MessageQueue_Server_RX_Push(MQ_Item* item)
{
    CriticalSection cs(&m_lock_si);
    m_queue_si.push(item);
}

// OK (USER)
uint32_t MessageQueue_Server_RX_Peek()
{
    CriticalSection cs(&m_lock_si);
    return m_queue_si.empty() ? MQ_MARKER : m_queue_si.front()->size;
}

// OK (USER)
void MessageQueue_Server_RX_Pull(uint32_t& command, void* data)
{
    MQ_Item* item;

    {
    CriticalSection cs(&m_lock_si);
    item = pull(m_queue_si);
    }

    command = item->command;
    memcpy(data, item->data, item->size);

    MessageQueue_Item_Delete(item);
}

// OK (USER)
void MessageQueue_Server_TX_Push(uint32_t id)
{
    CriticalSection cs(&m_lock_so);
    m_queue_so.push(id);
    ReleaseSemaphore(m_semaphore_so, 1, NULL);
}

// OK
bool MessageQueue_Server_TX_Pull(HANDLE event_cancel, uint32_t& id)
{
    HANDLE events[2] = { event_cancel, m_semaphore_so };

    if (WaitForMultipleObjects(sizeof(events) / sizeof(HANDLE), events, FALSE, INFINITE) != (WAIT_OBJECT_0 + 1)) { return false; }

    {
    CriticalSection cs(&m_lock_so);
    id = pull(m_queue_so);
    }

    return true;
}

// OK (USER)
void MessageQueue_Server_Restart()
{
    {
    CriticalSection cs(&m_lock_so);
    while (WaitForSingleObject(m_semaphore_so, 0) == WAIT_OBJECT_0) { m_queue_so.pop(); }
    }

    SetEvent(m_event_restart_s);
}

// OK
void MessageQueue_Server_WaitForRestart()
{
    MessageQueue_Server_RX_Push(MessageQueue_Item_Create(MQ_MARKER, 0));
    WaitForSingleObject(m_event_restart_s, INFINITE);
}

//-----------------------------------------------------------------------------
// MQX
//-----------------------------------------------------------------------------

// OK
void MessageQueue_Client_Startup()
{
    InitializeCriticalSection(&g_lock_co);
    InitializeCriticalSection(&g_lock_ci);
    g_event_restart_c = CreateEvent(NULL, FALSE, FALSE, NULL);
}

// OK
void MessageQueue_Client_Cleanup()
{
    CloseHandle(g_event_restart_c);
    g_event_restart_c = NULL;
    DeleteCriticalSection(&g_lock_ci);
    DeleteCriticalSection(&g_lock_co);
}

// OK
void MessageQueue_Client_RX_Push(uint32_t id)
{
    CriticalSection cs(&g_lock_co);
    g_queue_co.push(id);
}

// OK (USER)
uint32_t MessageQueue_Client_RX_Peek()
{
    CriticalSection cs(&g_lock_co);
    return g_queue_co.empty() ? MQ_MARKER : sizeof(uint32_t);
}

// OK (USER)
void MessageQueue_Client_RX_Pull(uint32_t& id)
{
    CriticalSection cs(&g_lock_co);
    id = pull(g_queue_co);
}

// OK (USER)
void MessageQueue_Client_TX_Push(uint32_t command, uint32_t size, void const* data)
{
    MQ_Item* item = MessageQueue_Item_Create(command, size);

    memcpy(item->data, data, size);

    CriticalSection cs(&g_lock_ci);
    g_queue_ci.push(item);
}

// OK
MQ_Item* MessageQueue_Client_TX_Pull()
{
    CriticalSection cs(&g_lock_ci);
    return (g_queue_ci.size() > 0) ? pull(g_queue_ci) : MessageQueue_Item_Create(MQ_MARKER, 0);
}

// OK (USER)
void MessageQueue_Client_Restart()
{
    {
    CriticalSection cs(&g_lock_ci);
    while (g_queue_ci.size() > 0) { MessageQueue_Item_Delete(pull(g_queue_ci)); }
    }

    SetEvent(g_event_restart_c);
}

// OK
void MessageQueue_Client_WaitForRestart()
{
    MessageQueue_Client_RX_Push(MQ_MARKER);
    WaitForSingleObject(g_event_restart_c, INFINITE);
}
