
#include <mfapi.h>
#include <codecapi.h>
#include "server.h"
#include "custom_media_sink.h"
#include "custom_media_types.h"
#include "custom_hook_callback.h"
#include "custom_sink_writers.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static uint32_t ComputeStride(uint32_t width)
{
	uint32_t const align = 64;
	uint32_t const mask = align - 1;

	return width + ((align - (width & mask)) & mask);
}

// OK
static void CreateSingleStreamSinkWriter(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwStreamIndex, IMFMediaType* pInputType, IMFMediaType* pOutputType, IMFAttributes* pEncoderAttr, HOOK_SINK_PROC hookproc, void* hookparam)
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
	pSinkWriter->SetInputMediaType(dwStreamIndex, pInputType, pEncoderAttr);
	pSinkWriter->BeginWriting();

	*ppSink = pSink;
	*ppSinkWriter = pSinkWriter;
	*pdwStreamIndex = dwStreamIndex;

	pHook->Release();
	pSinkAttr->Release();
}

// OK
void CreateSinkWriterAudio(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, AudioSubtype input_subtype, AACFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release

	CreateTypeAudio(&pTypeInput, format.channels, format.samplerate, input_subtype, AACProfile::AACProfile_None, AACLevel::AACLevel_Default);
	CreateTypeAudio(&pTypeOutput, format.channels, format.samplerate, input_subtype, format.profile, format.level);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwAudioIndex, pTypeInput, pTypeOutput, NULL, hookproc, hookparam);

	pTypeInput->Release();
	pTypeOutput->Release();
}

// OK
void CreateSinkWriterVideo(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, VideoSubtype input_subtype, H26xFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	uint32_t stride = ComputeStride(format.width);
	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release
	IMFAttributes* pEncoderAttr; // Release

	CreateTypeVideo(&pTypeInput, format.width, format.height, stride, format.framerate, format.divisor, input_subtype, H26xProfile::H26xProfile_None, 0);
	CreateTypeVideo(&pTypeOutput, format.width, format.height, stride, format.framerate, format.divisor, input_subtype, format.profile, format.bitrate);

	MFCreateAttributes(&pEncoderAttr, 1);

	pEncoderAttr->SetUINT32(CODECAPI_AVEncMPVGOPSize, format.gop_size);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwVideoIndex, pTypeInput, pTypeOutput, pEncoderAttr, hookproc, hookparam);

	pTypeInput->Release();
	pTypeOutput->Release();
	pEncoderAttr->Release();
}
