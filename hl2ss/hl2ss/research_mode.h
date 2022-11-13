
#pragma once

#include <vector>
#include "researchmode/ResearchModeApi.h"

int const RM_VLC_WIDTH  = 640;
int const RM_VLC_HEIGHT = 480;
int const RM_VLC_FPS    = 30;

int const RM_ZHT_WIDTH  = 512;
int const RM_ZHT_HEIGHT = 512;
int const RM_ZHT_FPS    = 45;
int const RM_ZHT_MASK   = 4090;

int const RM_ZLT_WIDTH  = 320;
int const RM_ZLT_HEIGHT = 288;
int const RM_ZLT_FPS    = 5;
int const RM_ZLT_MASK   = 128;

bool ResearchMode_WaitForCameraConsent();
bool ResearchMode_WaitForIMUConsent();
bool ResearchMode_WaitForConsent(IResearchModeSensor* sensor);

bool ResearchMode_Initialize();
void ResearchMode_Cleanup();

IResearchModeSensor* ResearchMode_GetSensor(ResearchModeSensorType type);
ResearchModeSensorType const* ResearchMode_GetSensorTypes();
int ResearchMode_GetSensorTypeCount();

bool ResearchMode_GetIntrinsics(IResearchModeSensor* sensor, std::vector<float>& uv2x, std::vector<float>& uv2y, std::vector<float>& mapx, std::vector<float>& mapy, float K[4]);
bool ResearchMode_GetExtrinsics(IResearchModeSensor* sensor, DirectX::XMFLOAT4X4& extrinsics);

GUID ResearchMode_GetRigNodeId();
