
#pragma once

#include "queue.h"
#include <mfidl.h>

typedef void(*HOOK_BUFFER_PROC)(IMFSample*, void*);

class SampleBuffer
{
private:
    HOOK_BUFFER_PROC m_hook;
    void* m_param;
    bool m_enable;
    std::queue<IMFSample*> m_buffer;
    CRITICAL_SECTION m_lock;
    HANDLE m_semaphore;
    HANDLE m_thread;

    void Loop();

    static DWORD WINAPI Thunk_Loop(void* self);

public:
    void Startup(HOOK_BUFFER_PROC hook, void* param, bool enable);
    void Push(IMFSample* pSample);
    void Cleanup();
};
