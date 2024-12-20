
#include <Windows.h>

#include "../hl2ss/ipl.h"
#include "../hl2ss/extended_execution.h"
#include "../hl2ss/locator.h"
#include "../hl2ss/server.h"
#include "../hl2ss/personal_video.h"
#include "../hl2ss/extended_video.h"
#include "../hl2ss/message_queue.h"
#include "../hl2ss/lock.h"
#include "../hl2ss/log.h"

#define HL2SS_PLUGIN_EXPORT extern "C" __declspec(dllexport)

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

// OK
HL2SS_PLUGIN_EXPORT
void InitializeStreams(uint32_t enable)
{
    HL2SS_Load(false); 
}

// OK
HL2SS_PLUGIN_EXPORT
void InitializeStreamsOnUI(uint32_t enable)
{
    HANDLE event_done = CreateEvent(NULL, TRUE, FALSE, NULL);
    ExtendedExecution_RunOnMainThread([=]() { InitializeStreams(enable); SetEvent(event_done); });
    WaitForSingleObject(event_done, INFINITE);
    CloseHandle(event_done);
}

// OK
HL2SS_PLUGIN_EXPORT
void DebugMessage(char const* str)
{
    ShowMessage("%s", str);
}

// OK
HL2SS_PLUGIN_EXPORT
void GetLocalIPv4Address(wchar_t *buffer, int size)
{
    winrt::hstring address = Server_GetLocalIPv4Address();
    wcscpy_s(buffer, size / sizeof(wchar_t), address.c_str());
}

// OK
HL2SS_PLUGIN_EXPORT
int OverrideWorldCoordinateSystem(void* scs_ptr)
{
    winrt::Windows::Perception::Spatial::SpatialCoordinateSystem scs = nullptr;
    if (scs_ptr)
    {
    winrt::copy_from_abi(scs, scs_ptr);
    scs = Locator_SanitizeSpatialCoordinateSystem(scs);
    if (!scs) { return false; }
    }
    Locator_OverrideWorldCoordinateSystem(scs);
    return true;
}

// OK
HL2SS_PLUGIN_EXPORT
void CheckExceptions()
{
    HL2SS_Process_EE();
}

//-----------------------------------------------------------------------------
// Message Queue
//-----------------------------------------------------------------------------

// OK
HL2SS_PLUGIN_EXPORT
uint32_t MQ_SI_Peek()
{
    return MessageQueue_Server_RX_Peek();
}

// OK
HL2SS_PLUGIN_EXPORT
void MQ_SI_Pop(uint32_t& command, uint8_t* data)
{
    MessageQueue_Server_RX_Pull(command, data);
}

// OK
HL2SS_PLUGIN_EXPORT
void MQ_SO_Push(uint32_t id)
{
    MessageQueue_Server_TX_Push(id);
}

// OK
HL2SS_PLUGIN_EXPORT
void MQ_Restart()
{
    MessageQueue_Server_Restart();
}

// OK
HL2SS_PLUGIN_EXPORT
uint32_t MQX_CO_Peek()
{
    return MessageQueue_Client_RX_Peek();
}

// OK
HL2SS_PLUGIN_EXPORT 
void MQX_CO_Pop(uint32_t& id)
{
    MessageQueue_Client_RX_Pull(id);
}

// OK
HL2SS_PLUGIN_EXPORT 
void MQX_CI_Push(uint32_t command, uint32_t size, uint8_t const* data)
{
    MessageQueue_Client_TX_Push(command, size, data);
}

// OK
HL2SS_PLUGIN_EXPORT 
void MQX_Restart()
{
    MessageQueue_Client_Restart();
}

//-----------------------------------------------------------------------------
// Lock
//-----------------------------------------------------------------------------

// OK
HL2SS_PLUGIN_EXPORT
void NamedMutex_Destroy(void* p)
{
    delete static_cast<NamedMutex*>(p);
}

// OK
HL2SS_PLUGIN_EXPORT
void* NamedMutex_Create(wchar_t const* name)
{
    NamedMutex* mutex = new (std::nothrow) NamedMutex(); // delete
    if (mutex == nullptr) { return NULL; }
    if (mutex->Create(name)) { return mutex; }
    NamedMutex_Destroy(mutex);
    return NULL;
}

// OK
HL2SS_PLUGIN_EXPORT
int NamedMutex_Acquire(void* p, uint32_t timeout)
{
    return static_cast<NamedMutex*>(p)->Acquire(timeout);
}

// OK
HL2SS_PLUGIN_EXPORT
int NamedMutex_Release(void* p)
{
    return static_cast<NamedMutex*>(p)->Release();
}

//-----------------------------------------------------------------------------
// PV
//-----------------------------------------------------------------------------

// OK
HL2SS_PLUGIN_EXPORT
void PersonalVideo_RegisterNamedMutex(wchar_t const* name)
{
    PersonalVideo_CreateNamedMutex(name);
}

//-----------------------------------------------------------------------------
// EV
//-----------------------------------------------------------------------------

// OK
HL2SS_PLUGIN_EXPORT
void ExtendedVideo_RegisterNamedMutex(wchar_t const* name)
{
    ExtendedVideo_CreateNamedMutex(name);
}
