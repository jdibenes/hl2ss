
#pragma once

#include <vector>

#include <winrt/Windows.Foundation.h>

struct VoiceInput_Result
{
    uint32_t Index;
    uint32_t Confidence;
    uint64_t PhraseDuration;
    uint64_t PhraseStartTime;
    double   RawConfidence;
};

void VoiceInput_Startup();
void VoiceInput_Cleanup();
void VoiceInput_Open();
void VoiceInput_Close();
bool VoiceInput_RegisterCommands(std::vector<winrt::hstring> const& strings);
void VoiceInput_Start();
void VoiceInput_Stop();
bool VoiceInput_Status();
uint32_t VoiceInput_GetCount();
VoiceInput_Result VoiceInput_Pop();
