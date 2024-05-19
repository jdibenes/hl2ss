
#include <mfapi.h>
#include "custom_media_sink.h"
#include "custom_media_buffers.h"
#include "extended_audio.h"
#include "neon.h"
#include "log.h"
#include "ports.h"
#include "timestamps.h"
#include "ipc_sc.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Media.Devices.Core.h>
#include <winrt/Windows.Foundation.Numerics.h>

using namespace winrt::Windows::Media;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Media::Devices::Core;
using namespace winrt::Windows::Foundation::Numerics;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle

static bool g_reader_status = false;
static IMFSinkWriter* g_pSinkWriter = nullptr; // Release
static DWORD g_dwAudioIndex = 0;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void EA_DeviceQuery(SOCKET clientsocket)
{
    winrt::hstring query;
    uint32_t bytes;
    WSABUF wsaBuf[2];

    ExtendedAudio_QueryDevices(query);

    bytes = query.size() * sizeof(wchar_t);

    pack_buffer(wsaBuf, 0, &bytes, sizeof(bytes));
    pack_buffer(wsaBuf, 1, query.c_str(), bytes);

    send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
static void EA_OnAudioFrameArrived(MediaFrameReader const& sender, MediaFrameArrivedEventArgs const& args)
{
    (void)args;

    MediaFrameReference frame = nullptr;
    IMFSample* pSample; // Release
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;
    int64_t timestamp;

    if (!g_reader_status) { return; }
    frame = sender.TryAcquireLatestFrame();
    if (!frame) { return; }

    timestamp = frame.SystemRelativeTime().Value().count();

    AudioMediaFrame amf       = frame.AudioMediaFrame();
    AudioFrame      audio     = amf.GetAudioFrame();
    AudioBuffer     buffer    = audio.LockBuffer(AudioBufferAccessMode::Read);
    auto            reference = buffer.CreateReference();

    auto aep = amf.AudioEncodingProperties();

    uint32_t bytes_per_sample = aep.BitsPerSample() / 8;
    uint32_t channels         = aep.ChannelCount();

    uint32_t input_bytes   = reference.Capacity();
    uint32_t input_samples = input_bytes / bytes_per_sample;
    uint32_t output_bytes  = input_samples * sizeof(int16_t);
    uint32_t fill_bytes    = (channels == 1) ? output_bytes : 0;
    uint32_t buffer_bytes  = output_bytes + fill_bytes;

    MFCreateMemoryBuffer(buffer_bytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);

    float*   src_addr  = (float*)reference.data();
    int16_t* base_addr = (int16_t*)pDst;
    int16_t* high_addr = (int16_t*)(pDst + fill_bytes);

    switch (bytes_per_sample)
    {
    case 4:  Neon_F32ToS16(src_addr, input_samples, high_addr); break;
    case 2:  memcpy(high_addr, src_addr, output_bytes);         break;
    default: memset(high_addr, 0,        output_bytes);
    }

    if (channels == 1) { Neon_S16MonoToStereo(high_addr, input_samples, base_addr); }

    pBuffer->Unlock();
    pBuffer->SetCurrentLength(buffer_bytes);

    reference.Close();
    buffer.Close();
    audio.Close();

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(timestamp);

    g_pSinkWriter->WriteSample(g_dwAudioIndex, pSample);

    pBuffer->Release();
    pSample->Release();
}

// OK
static void EA_SendSample(IMFSample* pSample, void* param)
{
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG sampletime;
    BYTE* pBytes;
    DWORD cbData;
    WSABUF wsaBuf[3];
    HookCallbackSocket* user;
    bool ok;

    user = (HookCallbackSocket*)param;

    pSample->GetSampleTime(&sampletime);
    pSample->ConvertToContiguousBuffer(&pBuffer);

    pBuffer->Lock(&pBytes, NULL, &cbData);

    pack_buffer(wsaBuf, 0, &sampletime, sizeof(sampletime));
    pack_buffer(wsaBuf, 1, &cbData, sizeof(cbData));
    pack_buffer(wsaBuf, 2, pBytes, cbData);

    ok = send_multiple(user->clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    if (!ok) { SetEvent(user->clientevent); }

    pBuffer->Unlock();
    pBuffer->Release();
}

// OK
static void EA_Shoutcast(SOCKET clientsocket)
{
    uint32_t const channels   = 2;
    uint32_t const samplerate = 48000;

    MediaFrameReader reader = nullptr;
    CustomMediaSink* pSink; // Release
    HANDLE event_client; // CloseHandle
    MRCAudioOptions options;
    AACFormat format;
    HookCallbackSocket user;
    bool ok;

    ok = ReceiveMRCAudioOptions(clientsocket, options);
    if (!ok) { return; }

    ok = ReceiveAACFormat_Profile(clientsocket, format);
    if (!ok) { return; }

    if ((options.mixer_mode & 0x80000000) != 0)
    {
    EA_DeviceQuery(clientsocket);
    return;
    }

    ok = ExtendedAudio_Open(options);
    if (!ok) { return; }
    
    format.channels   = channels;
    format.samplerate = samplerate;

    event_client = CreateEvent(NULL, TRUE, FALSE, NULL);

    ExtendedAudio_RegisterEvent(event_client);

    user.clientsocket = clientsocket;
    user.clientevent  = event_client;
    user.format       = &format;

    CreateSinkWriterAudio(&pSink, &g_pSinkWriter, &g_dwAudioIndex, AudioSubtype::AudioSubtype_S16, format, EA_SendSample, &user);

    reader = ExtendedAudio_CreateFrameReader(); 

    reader.AcquisitionMode(MediaFrameReaderAcquisitionMode::Buffered);
    reader.FrameArrived(EA_OnAudioFrameArrived);

    g_reader_status = true;
    reader.StartAsync().get();
    WaitForSingleObject(event_client, INFINITE);
    g_reader_status = false;
    reader.StopAsync().get();

    g_pSinkWriter->Flush(g_dwAudioIndex);
    g_pSinkWriter->Release();
    pSink->Shutdown();
    pSink->Release();

    ExtendedAudio_RegisterEvent(NULL);

    CloseHandle(event_client);

    ExtendedAudio_Close();
}

// OK
static DWORD WINAPI EA_EntryPoint(void*)
{
    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket

    listensocket = CreateSocket(PORT_NAME_EA);

    ShowMessage("EA: Listening at port %s", PORT_NAME_EA);

    do
    {
    ShowMessage("EA: Waiting for client");

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("EA: Client connected");

    EA_Shoutcast(clientsocket);

    closesocket(clientsocket);

    ShowMessage("EA: Client disconnected");
    }
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

    closesocket(listensocket);

    ShowMessage("EA: Closed");

    return 0;
}

// OK
void EA_Initialize()
{
    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread = CreateThread(NULL, 0, EA_EntryPoint, NULL, 0, NULL);
}

// OK
void EA_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void EA_Cleanup()
{
    WaitForSingleObject(g_thread, INFINITE);

    CloseHandle(g_thread);
    CloseHandle(g_event_quit);

    g_thread = NULL;
    g_event_quit = NULL;
}
