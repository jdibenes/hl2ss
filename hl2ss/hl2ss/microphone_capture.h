
#pragma once

#include <mmdeviceapi.h>
#include <AudioClient.h>
#include <mfidl.h>
#include <mfreadwrite.h>

#include <winrt/Windows.Media.Devices.Core.h>

// OK
class MicrophoneCapture : public winrt::implements<MicrophoneCapture, IActivateAudioInterfaceCompletionHandler>
{
private:
	winrt::com_ptr<IAudioClient3> m_audioClient;
	winrt::com_ptr<IAudioCaptureClient> m_audioCaptureClient;
	WAVEFORMATEX* m_wfx; // CoTaskMemFree
	HANDLE m_eventActivate; // CloseHandle
	HANDLE m_eventData; // CloseHandle

public:
	MicrophoneCapture();
	~MicrophoneCapture();

	bool Initialize(); // Call from the main thread
	bool WaitActivate(DWORD milliseconds);
	void Start();
	void Stop();
	void WriteSample(IMFSinkWriter* pSinkWriter, DWORD dwAudioIndex);

	// IActivateAudioInterfaceCompletionHandler Methods
	STDMETHOD(ActivateCompleted)(IActivateAudioInterfaceAsyncOperation* operation);
};
