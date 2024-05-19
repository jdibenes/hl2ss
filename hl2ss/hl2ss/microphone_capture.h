
#pragma once

#include <mmdeviceapi.h>
#include <AudioClient.h>
#include <mfidl.h>
#include <mfreadwrite.h>

#include <winrt/Windows.Foundation.h>

// OK
class MicrophoneCapture : public winrt::implements<MicrophoneCapture, IActivateAudioInterfaceCompletionHandler>
{
private:
	winrt::com_ptr<IAudioClient3> m_audioClient;
	winrt::com_ptr<IAudioCaptureClient> m_audioCaptureClient;
	WAVEFORMATEX* m_wfx; // CoTaskMemFree
	HANDLE m_eventActivate; // CloseHandle
	HANDLE m_eventData; // CloseHandle
	bool m_raw;
	bool m_status;

	HRESULT Configure(IActivateAudioInterfaceAsyncOperation* operation);

public:
	MicrophoneCapture();
	~MicrophoneCapture();

	bool Initialize(bool raw);
	void Activate(); // Call from the main thread
	bool WaitActivate(DWORD milliseconds);
	void Start();
	bool Status();
	void Stop();
	void WriteSample(IMFSinkWriter* pSinkWriter, DWORD dwAudioIndex);

	// IActivateAudioInterfaceCompletionHandler Methods
	STDMETHOD(ActivateCompleted)(IActivateAudioInterfaceAsyncOperation* operation);
};
