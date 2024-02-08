
#include <mfapi.h>
#include "custom_media_sink.h"
#include "ports.h"
#include "microphone_capture.h"
#include "ipc_sc.h"
#include "log.h"

#include <winrt/Windows.UI.Core.h>
#include <winrt/Windows.ApplicationModel.Core.h>

using namespace winrt::Windows::UI::Core;
using namespace winrt::Windows::ApplicationModel::Core;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle

static winrt::com_ptr<MicrophoneCapture> g_microphoneCapture = nullptr;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void MC_SendSample(IMFSample* pSample, void* param)
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
static void MC_Shoutcast(SOCKET clientsocket)
{
	uint32_t const samplerate = 48000;
	
	CustomMediaSink* pSink; // Release
	IMFSinkWriter* pSinkWriter; // Release
	HANDLE event_client; // CloseHandle
	bool raw;
	uint8_t channels;
	AudioSubtype subtype;
	AACFormat format;
	HookCallbackSocket user;
	DWORD dwAudioIndex;
	bool ok;

	ok = ReceiveAACFormat_Profile(clientsocket, format);
	if (!ok) { return; }

	raw = (format.profile == AACProfile::AACProfile_None) && (format.level == AACLevel::AACLevel_L5);

	if (raw) { channels = 5; subtype = AudioSubtype::AudioSubtype_F32; }
	else     { channels = 2; subtype = AudioSubtype::AudioSubtype_S16; }

	format.channels = channels;
	format.samplerate = samplerate;

	g_microphoneCapture = winrt::make_self<MicrophoneCapture>();
	g_microphoneCapture->Initialize(raw);

	CoreApplication::Views().GetAt(0).Dispatcher().RunAsync(CoreDispatcherPriority::High, [=]() { g_microphoneCapture->Activate(); }).get();
	
	g_microphoneCapture->WaitActivate(INFINITE);
	if (!g_microphoneCapture->Status()) { return; }

	event_client = CreateEvent(NULL, TRUE, FALSE, NULL);

	user.clientsocket = clientsocket;
	user.clientevent  = event_client;
	user.format       = &format;

	CreateSinkWriterAudio(&pSink, &pSinkWriter, &dwAudioIndex, subtype, format, MC_SendSample, &user);

	g_microphoneCapture->Start();
	do { g_microphoneCapture->WriteSample(pSinkWriter, dwAudioIndex); } while (WaitForSingleObject(event_client, 0) == WAIT_TIMEOUT);
	g_microphoneCapture->Stop();
	
	pSinkWriter->Flush(dwAudioIndex);
	pSinkWriter->Release();
	pSink->Shutdown();
	pSink->Release();

	CloseHandle(event_client);
}

// OK
static DWORD WINAPI MC_EntryPoint(void*)
{
	SOCKET listensocket; // closesocket
	SOCKET clientsocket; // closesocket

	listensocket = CreateSocket(PORT_NAME_MC);

	ShowMessage("MC: Listening at port %s", PORT_NAME_MC);

	do
	{
	ShowMessage("MC: Waiting for client");

	clientsocket = accept(listensocket, NULL, NULL); // block
	if (clientsocket == INVALID_SOCKET) { break; }

	ShowMessage("MC: Client connected");

	MC_Shoutcast(clientsocket);

	closesocket(clientsocket);

	ShowMessage("MC: Client disconnected");
	}
	while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

	closesocket(listensocket);

	ShowMessage("MC: Closed");

	return 0;
}

// OK
void MC_Initialize()
{
	g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
	g_thread = CreateThread(NULL, 0, MC_EntryPoint, NULL, 0, NULL);
}

// OK
void MC_Quit()
{
	SetEvent(g_event_quit);
}

// OK
void MC_Cleanup()
{
	WaitForSingleObject(g_thread, INFINITE);

	CloseHandle(g_thread);
	CloseHandle(g_event_quit);

	g_thread = NULL;
	g_event_quit = NULL;
	g_microphoneCapture = nullptr;
}
