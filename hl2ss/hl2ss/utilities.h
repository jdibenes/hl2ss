
#pragma once


#include "server.h"
#include "custom_hook_callback.h"
#include "custom_media_types.h"
#include "custom_sink_writers.h"



struct HookCallbackSocket
{
    SOCKET clientsocket;
    HANDLE clientevent;
};

bool ReceiveAudioFormatAAC(SOCKET clientsocket, AACBitrate& bitrate);
bool ReceiveVideoFormatH26x(SOCKET clientsocket, H26xFormat& format);

void PackUINT16toUINT32(BYTE const* slo16, BYTE const* shi16, BYTE* dst32, int n32ByteVectors);

template <class T> void SafeRelease(T** ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = 0;
    }
}

class CriticalSection
{
private:
    void* m_pcs;

public:
    CriticalSection(void* pcs);
    ~CriticalSection();
};

void ShowMessage(const char* format, ...);
void ShowMessage(const wchar_t* format, ...);

UINT64 GetCurrentQPCTimeHns();

//#define TRACE(format, params) ShowMessage("%s:%d %s>" format, __FILE__, __LINE__, __func__, params)
