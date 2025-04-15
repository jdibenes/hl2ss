
#include "encoder_any.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_ANY::Encoder_ANY(HOOK_ENCODER_PROC pHookCallback, void* pHookParam) :
CustomEncoder(pHookCallback, pHookParam, NULL, sizeof(ANY_Metadata))
{
}

// OK
void Encoder_ANY::WriteSample(BYTE const* data, uint32_t data_size, LONGLONG timestamp, ANY_Metadata* metadata)
{
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

    CreateBuffer(&pBuffer, data_size);

    pBuffer->Lock(&pDst, NULL, NULL);
    memcpy(pDst, data, data_size);
    pBuffer->Unlock();

    WriteBuffer(pBuffer, timestamp, 0, metadata);

    pBuffer->Release();
}
