
#include "encoder_rm_imu.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_RM_IMU::Encoder_RM_IMU(HOOK_ENCODER_PROC pHookCallback, void* pHookParam) :
CustomEncoder(pHookCallback, pHookParam, NULL, sizeof(RM_IMU_Metadata))
{
}

// OK
void Encoder_RM_IMU::WriteSample(BYTE const* data, uint32_t data_size, LONGLONG timestamp, RM_IMU_Metadata* metadata)
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
