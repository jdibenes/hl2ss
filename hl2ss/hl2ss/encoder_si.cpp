
#include "encoder_si.h"

using namespace winrt::Windows::Perception::People;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_SI::Encoder_SI(HOOK_ENCODER_PROC pHookCallback, void* pHookParam) :
CustomEncoder(pHookCallback, pHookParam, NULL, 0)
{
}

// OK
void Encoder_SI::WriteSample(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, LONGLONG timestamp)
{
    int32_t const packet_size = sizeof(uint32_t) + sizeof(SpatialInput_Frame) + sizeof(SpatialInput_Ray) + (2 * HAND_SIZE);

    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

    CreateBuffer(&pBuffer, packet_size);

    pBuffer->Lock(&pDst, NULL, NULL);

    memcpy(pDst, &valid,     sizeof(valid));              pDst += sizeof(valid);
    memcpy(pDst, head_pose,  sizeof(SpatialInput_Frame)); pDst += sizeof(SpatialInput_Frame);
    memcpy(pDst, eye_ray,    sizeof(SpatialInput_Ray));   pDst += sizeof(SpatialInput_Ray);
    memcpy(pDst, left_hand,  HAND_SIZE);                  pDst += HAND_SIZE;
    memcpy(pDst, right_hand, HAND_SIZE);                  pDst += HAND_SIZE;

    pBuffer->Unlock();

    WriteBuffer(pBuffer, timestamp, 0, NULL);

    pBuffer->Release();
}
