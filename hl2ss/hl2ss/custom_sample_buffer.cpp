
#include "custom_sample_buffer.h"
#include "lock.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void SampleBuffer::Startup(HOOK_BUFFER_PROC hook, void* param, bool enable)
{
    m_hook = hook;
    m_param = param;
    m_enable = enable;
    if (!enable) { return; }
    InitializeCriticalSection(&m_lock);
    m_semaphore = CreateSemaphore(NULL, 0, LONG_MAX, NULL);
    m_thread = CreateThread(NULL, 0, Thunk_Loop, this, 0, NULL);
}

// OK
void SampleBuffer::Push(IMFSample* pSample)
{
    if (!m_enable)
    {
    m_hook(pSample, m_param);
    }
    else
    {
    pSample->AddRef();
    {
    CriticalSection cs(&m_lock);
    m_buffer.push(pSample);
    }
    ReleaseSemaphore(m_semaphore, 1, NULL);
    }
}

// OK
void SampleBuffer::Loop()
{
    IMFSample* pSample; // Release

    while (true)
    {
    WaitForSingleObject(m_semaphore, INFINITE);
    {
    CriticalSection cs(&m_lock);
    if (m_buffer.empty()) { return; }
    pSample = pull(m_buffer);
    }
    m_hook(pSample, m_param);
    pSample->Release();
    }
}

// OK
DWORD WINAPI SampleBuffer::Thunk_Loop(void* self)
{
    static_cast<SampleBuffer*>(self)->Loop();
    return 0;
}

// OK
void SampleBuffer::Cleanup()
{
    if (!m_enable) { return; }
    ReleaseSemaphore(m_semaphore, 1, NULL);
    WaitForSingleObject(m_thread, INFINITE);
    CloseHandle(m_thread);
    CloseHandle(m_semaphore);
    DeleteCriticalSection(&m_lock);
}
