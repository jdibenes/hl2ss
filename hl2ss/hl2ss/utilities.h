
#pragma once

#include "server.h"
#include "custom_hook_callback.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"

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
// Media Sink
//-----------------------------------------------------------------------------

struct HookCallbackSocket
{
    SOCKET clientsocket;
    HANDLE clientevent;
};

//-----------------------------------------------------------------------------
// Remote Configuration
//-----------------------------------------------------------------------------

bool ReceiveAudioFormatAAC(SOCKET clientsocket, AACBitrate& bitrate);
bool ReceiveVideoFormatH26x(SOCKET clientsocket, H26xFormat& format);
bool ReceiveVideoFormat(SOCKET clientsocket, H26xFormat& format);
bool ReceiveVideoH26x(SOCKET clientsocket, H26xFormat& format);


//-----------------------------------------------------------------------------
// Packing
//-----------------------------------------------------------------------------

void PackUINT16toUINT32(BYTE const* slo16, BYTE const* shi16, BYTE* dst32, int n32ByteVectors);

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
