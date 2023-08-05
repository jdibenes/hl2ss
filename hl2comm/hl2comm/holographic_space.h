
#pragma once

#include <winrt/Windows.Perception.h>

void HolographicSpace_Initialize();
void HolographicSpace_Update();
winrt::Windows::Perception::PerceptionTimestamp HolographicSpace_GetTimestamp();
void HolographicSpace_Clear();
void HolographicSpace_Present();
void HolographicSpace_EnableMarker(bool state);
