
#pragma once

#include "research_mode.h"

bool RM_GetCameraExtrinsics(ResearchModeSensorType type, DirectX::XMFLOAT4X4& extrinsics);
bool RM_GetCameraIntrinsics(ResearchModeSensorType type, std::vector<float>& uv2x, std::vector<float>& uv2y);
void RM_Initialize();
void RM_Quit();
void RM_Cleanup();
