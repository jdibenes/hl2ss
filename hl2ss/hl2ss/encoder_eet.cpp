
#include "encoder_eet.h"

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Microsoft::MixedReality::EyeTracking;

struct EET_Frame
{
    float3   c_origin;
    float3   c_direction;
    float3   l_origin;
    float3   l_direction;
    float3   r_origin;
    float3   r_direction;
    float    l_openness;
    float    r_openness;
    float    vergence_distance;
    uint32_t valid;
};

struct EET_Packet
{
    uint64_t  timestamp;
    uint32_t  size;
    uint32_t  _reserved;
    EET_Frame frame;
    float4x4  pose;
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_EET::Encoder_EET(HOOK_ENCODER_PROC pHookCallback, void* pHookParam) :
CustomEncoder(pHookCallback, pHookParam, NULL, 0)
{
}

// OK
void Encoder_EET::WriteSample(EyeGazeTrackerReading const& frame, float4x4 const& pose, LONGLONG timestamp)
{
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

    CreateBuffer(&pBuffer, sizeof(EET_Packet));

    pBuffer->Lock(&pDst, NULL, NULL);

    EET_Packet& eet_packet = *reinterpret_cast<EET_Packet*>(pDst);

    memset(&eet_packet, 0, sizeof(eet_packet));
    
    eet_packet.timestamp = timestamp;
    eet_packet.size      = sizeof(EET_Packet::_reserved) + sizeof(EET_Packet::frame);
    eet_packet._reserved = 0;

    if (frame)
    {
    bool cg_valid = frame.TryGetCombinedEyeGazeInTrackerSpace(eet_packet.frame.c_origin, eet_packet.frame.c_direction);
    bool lg_valid = frame.TryGetLeftEyeGazeInTrackerSpace(eet_packet.frame.l_origin, eet_packet.frame.l_direction);
    bool rg_valid = frame.TryGetRightEyeGazeInTrackerSpace(eet_packet.frame.r_origin, eet_packet.frame.r_direction);
    bool lo_valid = frame.TryGetLeftEyeOpenness(eet_packet.frame.l_openness);
    bool ro_valid = frame.TryGetRightEyeOpenness(eet_packet.frame.r_openness);
    bool vd_valid = frame.TryGetVergenceDistance(eet_packet.frame.vergence_distance);
    bool ec_valid = frame.IsCalibrationValid();

    eet_packet.frame.valid = (vd_valid << 6) | (ro_valid << 5) | (lo_valid << 4) | (rg_valid << 3) | (lg_valid << 2) | (cg_valid << 1) | (ec_valid << 0);
    }

    eet_packet.pose = pose;

    pBuffer->Unlock();

    WriteBuffer(pBuffer, timestamp, 0, NULL);

    pBuffer->Release();
}
