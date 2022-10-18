
#pragma once

#include <winrt/Windows.Perception.h>

void SI_NotifyNextFrame(winrt::Windows::Perception::PerceptionTimestamp const& ts);
void SI_Initialize();
void SI_Quit();
void SI_Cleanup();
