
#pragma once

#include <vector>
#include <researchmode/ResearchModeApi.h>

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

uint8_t const NV12_ZERO_CHROMA = 0x80;

bool ResearchMode_WaitForConsent(IResearchModeSensor* sensor);

bool ResearchMode_Initialize();
void ResearchMode_Cleanup();

IResearchModeSensor* ResearchMode_GetSensor(ResearchModeSensorType type);
ResearchModeSensorType const* ResearchMode_GetSensorTypes();
int ResearchMode_GetSensorTypeCount();

bool ResearchMode_GetIntrinsics(IResearchModeSensor* sensor, std::vector<float>& uv2x, std::vector<float>& uv2y, std::vector<float>& mapx, std::vector<float>& mapy, float K[4]);
bool ResearchMode_GetExtrinsics(IResearchModeSensor* sensor, DirectX::XMFLOAT4X4& extrinsics);

winrt::Windows::Perception::Spatial::SpatialLocator ResearchMode_GetLocator();
