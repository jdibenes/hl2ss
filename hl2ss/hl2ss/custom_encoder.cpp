
#include <mfapi.h>
#include "custom_encoder.h"
#include "extended_execution.h"
#include "lock.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
CustomEncoder::CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size)
{
    m_metadata      = std::make_unique<uint8_t[]>(metadata_size);
    m_metadata_size = metadata_size;
    m_pHookCallback = pHookCallback;
    m_pHookParam    = pHookParam;
    m_pMetadataFree = pMetadataFree;

    memset(m_metadata.get(), 0, metadata_size);

    m_buffer_i.Startup(CustomEncoder::Thunk_Write,   this, ExtendedExecution_GetReaderBuffering());
    m_buffer_o.Startup(CustomEncoder::Thunk_Process, this, ExtendedExecution_GetEncoderBuffering());
}

// OK
CustomEncoder::CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size, AudioSubtype input_subtype, AACFormat  const& format) :
CustomEncoder(pHookCallback, pHookParam, pMetadataFree, metadata_size)
{
    m_pSinkWriter = CustomSinkWriter::CreateForAudio(Thunk_Sink, this, input_subtype, format);
}

// OK
CustomEncoder::CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size, VideoSubtype input_subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& encoder_options) :
CustomEncoder(pHookCallback, pHookParam, pMetadataFree, metadata_size)
{
    m_pSinkWriter = CustomSinkWriter::CreateForVideo(Thunk_Sink, this, input_subtype, format, stride, encoder_options);
}

// OK
CustomEncoder::~CustomEncoder()
{
    m_buffer_i.Cleanup();
    m_pSinkWriter.reset();
    m_buffer_o.Cleanup();
}

// OK
void CustomEncoder::WriteSample(IMFSample* pSample)
{
    if (m_pSinkWriter) { m_pSinkWriter->WriteSample(pSample); } else { Thunk_Sink(pSample, this); }
}

// OK
void CustomEncoder::ReceiveSample(IMFSample* pSample)
{
    m_buffer_o.Push(pSample);
}

// OK
void CustomEncoder::ProcessSample(IMFSample* pSample)
{
    UINT32 bCleanPoint = TRUE;
    IMFMediaBuffer* pBuffer; // Release
    LONGLONG hnsSampleTime;
    BYTE* pFrame;
    DWORD cbFrameBytes;

    pSample->GetUINT32(MFSampleExtension_CleanPoint, &bCleanPoint);

    pSample->ConvertToContiguousBuffer(&pBuffer);
    pSample->GetSampleTime(&hnsSampleTime);

    pSample->GetBlob(MF_USER_DATA_PAYLOAD, m_metadata.get(), m_metadata_size, NULL);
    
    pBuffer->Lock(&pFrame, NULL, &cbFrameBytes);

    m_pHookCallback(pFrame, cbFrameBytes, bCleanPoint, hnsSampleTime, m_metadata.get(), m_metadata_size, m_pHookParam);

    pBuffer->Unlock();
    pBuffer->Release();

    if (m_pMetadataFree) { m_pMetadataFree(m_metadata.get(), m_metadata_size); }
}

// OK
void CustomEncoder::Thunk_Write(IMFSample* pSample, void* self)
{
    static_cast<CustomEncoder*>(self)->WriteSample(pSample);
}

// OK
void CustomEncoder::Thunk_Sink(IMFSample* pSample, void* self)
{
    static_cast<CustomEncoder*>(self)->ReceiveSample(pSample);
}

// OK
void CustomEncoder::Thunk_Process(IMFSample* pSample, void* self)
{
    static_cast<CustomEncoder*>(self)->ProcessSample(pSample);
}

// OK
void CustomEncoder::CreateBuffer(IMFMediaBuffer** ppBuffer, DWORD size)
{
    MFCreateMemoryBuffer(size, ppBuffer);

    (*ppBuffer)->SetCurrentLength(size);
}

// OK
void CustomEncoder::WriteBuffer(IMFMediaBuffer* pBuffer, LONGLONG timestamp, LONGLONG duration, void* metadata)
{
    IMFSample* pSample; // Release

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(duration);
    pSample->SetSampleTime(timestamp);
    pSample->SetBlob(MF_USER_DATA_PAYLOAD, static_cast<UINT8*>(metadata), m_metadata_size);

    m_buffer_i.Push(pSample);

    pSample->Release();
}

// OK
void CustomEncoder_Startup()
{
    MFStartup(MF_VERSION);
}

// OK
void CustomEncoder_Cleanup()
{
    MFShutdown();
}
