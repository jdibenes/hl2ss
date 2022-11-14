
#include <mfapi.h>
#include "microphone_capture.h"
#include "timestamps.h"
#include "types.h"

using namespace winrt::Windows::Media::Devices;

//-----------------------------------------------------------------------------
// MicrophoneCapture Methods
//-----------------------------------------------------------------------------

// OK
MicrophoneCapture::MicrophoneCapture() :
	m_audioClient(nullptr),
	m_audioCaptureClient(nullptr),
	m_wfx(nullptr),
	m_eventActivate(NULL),
	m_eventData(NULL)
{
}

// OK
MicrophoneCapture::~MicrophoneCapture()
{
	m_audioClient = nullptr;
	m_audioCaptureClient = nullptr;
	if (m_wfx) { CoTaskMemFree(m_wfx); }
	if (m_eventActivate) { CloseHandle(m_eventActivate); }
	if (m_eventData) { CloseHandle(m_eventData); }
}

// OK
bool MicrophoneCapture::Initialize()
{
	winrt::com_ptr<IActivateAudioInterfaceAsyncOperation> asyncOp;
	HRESULT hr;

	hr = ActivateAudioInterfaceAsync(MediaDevice::GetDefaultAudioCaptureId(AudioDeviceRole::Default).c_str(), __uuidof(IAudioClient3), nullptr, this, asyncOp.put());
	if (FAILED(hr)) { return false; }

	m_eventActivate = CreateEvent(nullptr, TRUE, FALSE, nullptr);
	if (!m_eventActivate) { return false; }

	return true;
}

// OK
HRESULT MicrophoneCapture::ActivateCompleted(IActivateAudioInterfaceAsyncOperation* operation)
{
	WORD const bitspersample = 16;
	DWORD const samplerate = 48000;
	WORD const channels = 2;
	REFERENCE_TIME const buffersizehns = (HNS_BASE * 16000ULL * 3ULL) / samplerate; // 1 second

	winrt::com_ptr<IUnknown> punkAudioInterface;
	HRESULT activateStatus;
	HRESULT hr;
	WAVEFORMATEXTENSIBLE* wfe;
	BOOL ok;

	hr = operation->GetActivateResult(&activateStatus, punkAudioInterface.put());
	if (FAILED(hr)) { return hr; }
	if (FAILED(activateStatus)) { return activateStatus; }

	m_audioClient = punkAudioInterface.as<IAudioClient3>();

	hr = m_audioClient->GetMixFormat(&m_wfx);
	if (FAILED(hr)) { return hr; }

	wfe = reinterpret_cast<WAVEFORMATEXTENSIBLE*>(m_wfx);

	wfe->SubFormat = KSDATAFORMAT_SUBTYPE_PCM;
	wfe->Format.wBitsPerSample = bitspersample;
	wfe->Format.nSamplesPerSec = samplerate;
	wfe->Format.nChannels = channels;
	wfe->Format.nBlockAlign = wfe->Format.nChannels * (wfe->Format.wBitsPerSample / 8);
	wfe->Format.nAvgBytesPerSec = wfe->Format.nBlockAlign * wfe->Format.nSamplesPerSec;
	wfe->Samples.wValidBitsPerSample = wfe->Format.wBitsPerSample;

	hr = m_audioClient->Initialize(AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_EVENTCALLBACK, buffersizehns, 0, m_wfx, NULL);
	if (FAILED(hr)) { return hr; }

	m_eventData = CreateEvent(NULL, FALSE, FALSE, NULL);
	if (!m_eventData) { return E_FAIL; }

	hr = m_audioClient->SetEventHandle(m_eventData);
	if (FAILED(hr)) { return hr; }

	m_audioCaptureClient.capture(m_audioClient, &IAudioClient::GetService);

	ok = SetEvent(m_eventActivate);
	if (!ok) { return E_FAIL; }

	return S_OK;
}

// OK
bool MicrophoneCapture::WaitActivate(DWORD milliseconds)
{
	return WaitForSingleObject(m_eventActivate, milliseconds) == WAIT_OBJECT_0;
}

// OK
void MicrophoneCapture::Start()
{
	m_audioClient->Start();
}

// OK
void MicrophoneCapture::Stop()
{
	m_audioClient->Stop();
	//m_audioClient->Reset();
}

// OK
void MicrophoneCapture::WriteSample(IMFSinkWriter* pSinkWriter, DWORD dwAudioIndex)
{
	int const frame_duration_num = 625; // 10,000,000
	int const frame_duration_den = 3;   //     48,000

	IMFSample* pSample; // Release
	IMFMediaBuffer* pBuffer; // Release
	UINT32 framesAvailable; // ReleaseBuffer
	BYTE* data;
	DWORD dwCaptureFlags;
	int bytes;
	BYTE* pDst;
	UINT64 qpc;

	WaitForSingleObject(m_eventData, INFINITE);

	while (m_audioCaptureClient->GetBuffer(&data, &framesAvailable, &dwCaptureFlags, NULL, &qpc) == S_OK)
	{
	bytes = framesAvailable * m_wfx->nBlockAlign;

	MFCreateMemoryBuffer(bytes, &pBuffer);

	pBuffer->Lock(&pDst, NULL, NULL);
	if (dwCaptureFlags & AUDCLNT_BUFFERFLAGS_SILENT) { memset(pDst, 0, bytes); } else { memcpy(pDst, data, bytes); }
	pBuffer->Unlock();
	pBuffer->SetCurrentLength(bytes);

	m_audioCaptureClient->ReleaseBuffer(framesAvailable);

	MFCreateSample(&pSample);

	pSample->AddBuffer(pBuffer);
	pSample->SetSampleDuration((framesAvailable * frame_duration_num) / frame_duration_den);
	pSample->SetSampleTime(qpc);

	pSinkWriter->WriteSample(dwAudioIndex, pSample);

	pBuffer->Release();
	pSample->Release();
	}
}
