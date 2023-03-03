
#include <mfapi.h>
#include "server.h"
#include "custom_media_sink.h"
#include "custom_media_types.h"
#include "custom_hook_callback.h"
#include "custom_sink_writers.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static uint32_t ComputeNV12Stride(uint32_t width)
{
	return width + ((64 - (width & 63)) & 63);
}

// OK
static void CreateSingleStreamSinkWriter(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwStreamIndex, IMFMediaType* pInputType, IMFMediaType* pOutputType, HOOK_SINK_PROC hookproc, void* hookparam)
{
	HookSinkCallback* pHook; // Release
	CustomMediaSink* pSink;
	IMFAttributes* pSinkAttr; // Release
	IMFSinkWriter* pSinkWriter;
	DWORD dwStreamIndex;

	HookSinkCallback::CreateInstance(&pHook, hookproc, hookparam);
	CustomMediaSink::CreateInstance(&pSink, MEDIASINK_RATELESS, pHook);

	MFCreateAttributes(&pSinkAttr, 3);

	pSinkAttr->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
	pSinkAttr->SetUINT32(MF_LOW_LATENCY, TRUE);
	pSinkAttr->SetUINT32(MF_SINK_WRITER_DISABLE_THROTTLING, TRUE);

	MFCreateSinkWriterFromMediaSink(pSink, pSinkAttr, &pSinkWriter);

	pSinkWriter->AddStream(pOutputType, &dwStreamIndex);
	pSinkWriter->SetInputMediaType(dwStreamIndex, pInputType, NULL);
	pSinkWriter->BeginWriting();

	*ppSink = pSink;
	*ppSinkWriter = pSinkWriter;
	*pdwStreamIndex = dwStreamIndex;

	pHook->Release();
	//pSink->Release();
	pSinkAttr->Release();
}

// OK
void CreateSinkWriterPCMToPCM(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, AACFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypePCM; // Release

	CreateTypePCMS16(&pTypePCM, format.channels, format.samplerate);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwAudioIndex, pTypePCM, pTypePCM, hookproc, hookparam);

	pTypePCM->Release();
}

// OK
void CreateSinkWriterPCMToAAC(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, AACFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypePCM; // Release
	IMFMediaType* pTypeAAC; // Release

	CreateTypePCMS16(&pTypePCM, format.channels, format.samplerate);
	CreateTypeAAC(&pTypeAAC, format.channels, format.samplerate, format.profile);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwAudioIndex, pTypePCM, pTypeAAC, hookproc, hookparam);

	pTypePCM->Release();
	pTypeAAC->Release();
}

// OK
void CreateSinkWriterL8ToL8(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypeL8; // Release

	CreateTypeL8(&pTypeL8, format.width, format.height, format.width, format.framerate);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwVideoIndex, pTypeL8, pTypeL8, hookproc, hookparam);

	pTypeL8->Release();
}

// OK
void CreateSinkWriterNV12ToNV12(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypeNV12; // Release

	CreateTypeNV12(&pTypeNV12, format.width, format.height, ComputeNV12Stride(format.width), format.framerate);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwVideoIndex, pTypeNV12, pTypeNV12, hookproc, hookparam);

	pTypeNV12->Release();
}

// OK
void CreateSinkWriterARGBToARGB(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypeARGB; // Release

	CreateTypeARGB(&pTypeARGB, format.width, format.height, format.width, format.framerate);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwVideoIndex, pTypeARGB, pTypeARGB, hookproc, hookparam);

	pTypeARGB->Release();
}

// OK
void CreateSinkWriterNV12ToH26x(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypeNV12; // Release
	IMFMediaType* pTypeH26x; // Release

	CreateTypeNV12(&pTypeNV12, format.width, format.height, ComputeNV12Stride(format.width), format.framerate);
	CreateTypeH26x(&pTypeH26x, format.width, format.height, format.framerate, format.profile, format.bitrate);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwVideoIndex, pTypeNV12, pTypeH26x, hookproc, hookparam);

	pTypeNV12->Release();
	pTypeH26x->Release();
}
