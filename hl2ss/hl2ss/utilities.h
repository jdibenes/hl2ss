
#pragma once

#include "server.h"
#include "custom_hook_callback.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"

#include <winrt/Windows.Graphics.Imaging.h>

//-----------------------------------------------------------------------------
// COM
//-----------------------------------------------------------------------------

template <class T>
void SafeRelease(T** ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = 0;
    }
}

//-----------------------------------------------------------------------------
// Logging 
//-----------------------------------------------------------------------------

void ShowMessage(const char* format, ...);
void ShowMessage(const wchar_t* format, ...);

//-----------------------------------------------------------------------------
// Critical Section 
//-----------------------------------------------------------------------------

class CriticalSection
{
private:
    void* m_pcs;

public:
    CriticalSection(void* pcs);
    ~CriticalSection();
};
