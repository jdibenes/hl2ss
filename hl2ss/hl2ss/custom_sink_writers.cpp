
#include <mfapi.h>
#include <codecapi.h>
#include "custom_sink_writers.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
CustomSinkWriter::CustomSinkWriter(HOOK_SINK_PROC hookproc, void* hookparam, IMFMediaType* pInputType, IMFMediaType* pOutputType, IMFAttributes* pEncoderAttr)
{
	IMFAttributes* pSinkAttr; // Release

	MFCreateAttributes(&pSinkAttr, 3);
	
	pSinkAttr->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
	pSinkAttr->SetUINT32(MF_LOW_LATENCY, TRUE);
	pSinkAttr->SetUINT32(MF_SINK_WRITER_DISABLE_THROTTLING, TRUE);

	CustomMediaSink::CreateInstance(&m_pSink, MEDIASINK_RATELESS, hookproc, hookparam);

	MFCreateSinkWriterFromMediaSink(m_pSink, pSinkAttr, &m_pSinkWriter);

	m_pSinkWriter->AddStream(pOutputType, &m_dwStreamIndex);
	m_pSinkWriter->SetInputMediaType(m_dwStreamIndex, pInputType, pEncoderAttr);
	m_pSinkWriter->BeginWriting();

	pSinkAttr->Release();
}

// OK
CustomSinkWriter::~CustomSinkWriter()
{
	m_pSinkWriter->Finalize();
	m_pSinkWriter->Release();
	m_pSink->Shutdown();
	m_pSink->Release();
}

// OK
void CustomSinkWriter::WriteSample(IMFSample* pSample)
{
	m_pSinkWriter->WriteSample(m_dwStreamIndex, pSample);
}

// OK
std::unique_ptr<CustomSinkWriter> CustomSinkWriter::CreateForAudio(HOOK_SINK_PROC hookproc, void* hookparam, AudioSubtype input_subtype, AACFormat const& format)
{
	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release

	CreateTypeAudio(&pTypeInput,  format.channels, format.samplerate, input_subtype, AACProfile::AACProfile_None, AACLevel::AACLevel_Default);
	CreateTypeAudio(&pTypeOutput, format.channels, format.samplerate, input_subtype, format.profile,              format.level);

	std::unique_ptr<CustomSinkWriter> pSinkWriter = std::make_unique<CustomSinkWriter>(hookproc, hookparam, pTypeInput, pTypeOutput, nullptr);

	pTypeInput->Release();
	pTypeOutput->Release();

	return pSinkWriter;
}

// OK
std::unique_ptr<CustomSinkWriter> CustomSinkWriter::CreateForVideo(HOOK_SINK_PROC hookproc, void* hookparam, VideoSubtype input_subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& encoder_options)
{
	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release
	IMFAttributes* pEncoderAttr; // Release

	CreateTypeVideo(&pTypeInput,  format.width, format.height, stride, format.framerate, format.divisor, input_subtype, H26xProfile::H26xProfile_None, H26xLevel_Default, 0);
	CreateTypeVideo(&pTypeOutput, format.width, format.height, stride, format.framerate, format.divisor, input_subtype, format.profile,                format.level,      format.bitrate);

	TranslateEncoderOptions(encoder_options, &pEncoderAttr);

	std::unique_ptr<CustomSinkWriter> pSinkWriter = std::make_unique<CustomSinkWriter>(hookproc, hookparam, pTypeInput, pTypeOutput, pEncoderAttr);

	pTypeInput->Release();
	pTypeOutput->Release();
	pEncoderAttr->Release();

	return pSinkWriter;
}
