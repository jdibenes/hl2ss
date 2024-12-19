
#include <mmdeviceapi.h>
#include <AudioClient.h>
#include "extended_execution.h"
#include "microphone_capture.h"
#include "lock.h"
#include "nfo.h"

class MicrophoneCapture : public winrt::implements<MicrophoneCapture, IActivateAudioInterfaceCompletionHandler>
{
private:
	winrt::com_ptr<IAudioClient3> m_audio_client;
	winrt::com_ptr<IAudioCaptureClient> m_audio_capture_client;
	HANDLE m_event_data; // CloseHandle
	HANDLE m_event_activate; // CloseHandle
	bool m_raw;

	HRESULT Configure(IActivateAudioInterfaceAsyncOperation* operation);
	void Activate(); // Call from the main thread

public:
	MicrophoneCapture();
	~MicrophoneCapture();

	void Open(bool raw);
	void Close();
	bool Status();

	void ExecuteSensorLoop(HOOK_MC_PROC pHookCallback, void* pHookParam, HANDLE event_stop);

	// IActivateAudioInterfaceCompletionHandler Methods
	STDMETHOD(ActivateCompleted)(IActivateAudioInterfaceAsyncOperation* operation);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static winrt::com_ptr<MicrophoneCapture> g_mc;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
MicrophoneCapture::MicrophoneCapture()
{
	m_event_activate = CreateEvent(NULL, FALSE, FALSE, NULL);
	m_event_data     = CreateEvent(NULL, FALSE, FALSE, NULL);
}

// OK
MicrophoneCapture::~MicrophoneCapture()
{
	CloseHandle(m_event_data);
	CloseHandle(m_event_activate);
}

// OK
HRESULT MicrophoneCapture::Configure(IActivateAudioInterfaceAsyncOperation* operation)
{
	winrt::com_ptr<IUnknown> punkAudioInterface;
	AudioClientProperties acp;
	HRESULT activateStatus;
	WAVEFORMATEX* m_wfx; // CoTaskMemFree
	WAVEFORMATEXTENSIBLE* wfe;
	UINT32 defaultPeriodInFrames;
	UINT32 fundamentalPeriodInFrames;
	UINT32 minPeriodInFrames;
	UINT32 maxPeriodInFrames;

	Cleaner log_error_microphone([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedMicrophone); });

	operation->GetActivateResult(&activateStatus, punkAudioInterface.put());

	m_audio_client = punkAudioInterface.as<IAudioClient3>();

	if (m_raw)
	{
	acp.cbSize     = sizeof(AudioClientProperties);
	acp.eCategory  = AUDIO_STREAM_CATEGORY::AudioCategory_Media;
	acp.Options    = AUDCLNT_STREAMOPTIONS::AUDCLNT_STREAMOPTIONS_RAW;
	acp.bIsOffload = FALSE;

	m_audio_client->SetClientProperties(&acp);
	}

	m_audio_client->GetMixFormat(&m_wfx);

	Cleaner free_wfx([=]() { CoTaskMemFree(m_wfx); });

	wfe = reinterpret_cast<WAVEFORMATEXTENSIBLE*>(m_wfx);

	if (!m_raw)
	{
	wfe->SubFormat                   = KSDATAFORMAT_SUBTYPE_PCM;
	wfe->Format.wBitsPerSample       = 16;
	wfe->Format.nBlockAlign          = wfe->Format.nChannels * (wfe->Format.wBitsPerSample / 8);
	wfe->Format.nAvgBytesPerSec      = wfe->Format.nBlockAlign * wfe->Format.nSamplesPerSec;
	wfe->Samples.wValidBitsPerSample = wfe->Format.wBitsPerSample;
	}

	m_audio_client->GetSharedModeEnginePeriod(m_wfx, &defaultPeriodInFrames, &fundamentalPeriodInFrames, &minPeriodInFrames, &maxPeriodInFrames);
	activateStatus = m_audio_client->InitializeSharedAudioStream(AUDCLNT_STREAMFLAGS_EVENTCALLBACK, minPeriodInFrames, m_wfx, NULL);
	if (FAILED(activateStatus)) { return activateStatus; }
	m_audio_client->SetEventHandle(m_event_data);
	
	m_audio_capture_client.capture(m_audio_client, &IAudioClient::GetService);

	log_error_microphone.Set(false);

	return S_OK;
}

// OK
HRESULT MicrophoneCapture::ActivateCompleted(IActivateAudioInterfaceAsyncOperation* operation)
{
	HRESULT hr = Configure(operation);
	SetEvent(m_event_activate);
	return hr;
}

// OK
void MicrophoneCapture::Activate()
{
	winrt::com_ptr<IActivateAudioInterfaceAsyncOperation> asyncOp;
	ActivateAudioInterfaceAsync(GetBuiltInAudioCaptureId().c_str(), __uuidof(IAudioClient3), nullptr, this, asyncOp.put());
}

// OK
void MicrophoneCapture::Open(bool raw)
{
	m_raw = raw;
	ExtendedExecution_RunOnMainThread([=]() { this->Activate(); });
	WaitForSingleObject(m_event_activate, INFINITE);
}

// OK
void MicrophoneCapture::Close()
{
	m_audio_capture_client = nullptr;
	m_audio_client         = nullptr;
}

// OK
bool MicrophoneCapture::Status()
{
	return !!m_audio_capture_client;
}

// OK
void MicrophoneCapture::ExecuteSensorLoop(HOOK_MC_PROC hook, void* param, HANDLE event_stop)
{
	UINT32 frames; // ReleaseBuffer
	BYTE* data;
	DWORD flags;
	UINT64 dps;
	UINT64 qpc;

	m_audio_client->Start();

	do
	{
	WaitForSingleObject(m_event_data, INFINITE);

	while (m_audio_capture_client->GetBuffer(&data, &frames, &flags, &dps, &qpc) == S_OK)
	{
    hook(data, frames, flags & AUDCLNT_BUFFERFLAGS_SILENT, qpc, param);
	m_audio_capture_client->ReleaseBuffer(frames);
	}
	}
	while(WaitForSingleObject(event_stop, 0) == WAIT_TIMEOUT);
	
	m_audio_client->Stop();
	m_audio_client->Reset();

	WaitForSingleObject(m_event_data, 0);
}

// OK
void MicrophoneCapture_Startup()
{
	g_mc = winrt::make_self<MicrophoneCapture>();
}

// OK
void MicrophoneCapture_Cleanup()
{
	g_mc = nullptr;
}

// OK
void MicrophoneCapture_Open(bool raw)
{	
	g_mc->Open(raw);
}

// OK
void MicrophoneCapture_Close()
{
	g_mc->Close();
}

// OK
bool MicrophoneCapture_Status()
{
	return g_mc->Status();
}

// OK
void MicrophoneCapture_ExecuteSensorLoop(HOOK_MC_PROC hook, void* param, HANDLE event_stop)
{
	g_mc->ExecuteSensorLoop(hook, param, event_stop);
}
