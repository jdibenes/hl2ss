
#pragma once

#include <researchmode/ResearchModeApi.h>
#include <vector>

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>

int const RM_VLC_WIDTH  = 640;
int const RM_VLC_HEIGHT = 480;
int const RM_VLC_FPS    = 30;
int const RM_VLC_PIXELS = RM_VLC_WIDTH * RM_VLC_HEIGHT;
int const RM_VLC_SIZE   = RM_VLC_PIXELS * sizeof(uint8_t);

int const RM_ZHT_WIDTH  = 512;
int const RM_ZHT_HEIGHT = 512;
int const RM_ZHT_FPS    = 45;
int const RM_ZHT_MASK   = 4090;
int const RM_ZHT_PIXELS = RM_ZHT_WIDTH * RM_ZHT_HEIGHT;
int const RM_ZHT_ZSIZE  = RM_ZHT_PIXELS * sizeof(uint16_t);
int const RM_ZHT_ABSIZE = RM_ZHT_PIXELS * sizeof(uint16_t);

int const RM_ZLT_WIDTH  = 320;
int const RM_ZLT_HEIGHT = 288;
int const RM_ZLT_FPS    = 5;
int const RM_ZLT_MASK   = 128;
int const RM_ZLT_PIXELS = RM_ZLT_HEIGHT * RM_ZLT_WIDTH;
int const RM_ZLT_ZSIZE  = RM_ZLT_PIXELS * sizeof(uint16_t);
int const RM_ZLT_ABSIZE = RM_ZLT_PIXELS * sizeof(uint16_t);

int const RM_ACC_BATCH = 93;
int const RM_GYR_BATCH = 315;
int const RM_MAG_BATCH = 11;

typedef void (*HOOK_RM_PROC)(IResearchModeSensorFrame*, void*);
typedef void (*HOOK_RM_VLC_PROC)(BYTE const*, UINT64, UINT64, UINT64, UINT32, void*);
typedef void (*HOOK_RM_ZHT_PROC)(UINT16 const*, UINT16 const*, UINT64, UINT64, void*);
typedef void (*HOOK_RM_ZLT_PROC)(BYTE const*, UINT16 const*, UINT16 const*, UINT64, UINT64, void*);
typedef void (*HOOK_RM_ACC_PROC)(AccelDataStruct const*, size_t, UINT64, UINT64, void*);
typedef void (*HOOK_RM_GYR_PROC)(GyroDataStruct const*, size_t, UINT64, UINT64, void*);
typedef void (*HOOK_RM_MAG_PROC)(MagDataStruct const*, size_t, UINT64, UINT64, void*);

void ResearchMode_Startup();
void ResearchMode_Cleanup();
bool ResearchMode_Status();
IResearchModeSensor* ResearchMode_GetSensor(ResearchModeSensorType type);
winrt::Windows::Foundation::Numerics::float4x4 ResearchMode_GetRigNodeWorldPose(UINT64 host_ticks);
bool ResearchMode_WaitForConsent(IResearchModeSensor* sensor);
bool ResearchMode_GetIntrinsics(IResearchModeSensor* sensor, std::vector<float>& uv2x, std::vector<float>& uv2y, std::vector<float>& mapx, std::vector<float>& mapy, float K[4]);
bool ResearchMode_GetExtrinsics(IResearchModeSensor* sensor, DirectX::XMFLOAT4X4& extrinsics);
void ResearchMode_ExecuteSensorLoop(IResearchModeSensor* sensor, HOOK_RM_PROC hook, void* param, HANDLE event_stop);
void ResearchMode_ProcessSample_VLC(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_VLC_PROC hook, void* param);
void ResearchMode_ProcessSample_ZHT(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ZHT_PROC hook, void* param);
void ResearchMode_ProcessSample_ZLT(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ZLT_PROC hook, void* param);
void ResearchMode_ProcessSample_ACC(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ACC_PROC hook, void* param);
void ResearchMode_ProcessSample_GYR(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_GYR_PROC hook, void* param);
void ResearchMode_ProcessSample_MAG(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_MAG_PROC hook, void* param);

void ResearchMode_SetEyeSelection(bool enable);
