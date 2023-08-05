
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

void VoiceInput_Initialize();
void VoiceInput_CreateRecognizer();
bool VoiceInput_RegisterCommands(std::vector<winrt::hstring> const& strings, bool clear);
void VoiceInput_Start();
void VoiceInput_Stop();
size_t VoiceInput_GetCount();
VoiceInput_Result VoiceInput_Pop();
void VoiceInput_Clear();
bool VoiceInput_IsRunning();
