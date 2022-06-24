
#pragma once

#include <vector>
#include "researchmode/ResearchModeApi.h"

bool InitializeResearchMode();
void CleanupResearchMode();
bool ResearchModeWaitForCameraConsent();
bool ResearchModeWaitForIMUConsent();
void GetRigNodeId(GUID& outGuid);
IResearchModeSensor* GetResearchModeSensor(ResearchModeSensorType type);
ResearchModeSensorType const* GetResearchModeSensorTypes();
int GetResearchModeSensorTypeCount();
