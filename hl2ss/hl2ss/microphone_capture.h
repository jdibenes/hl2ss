
#pragma once

#include <Windows.h>

typedef void (*HOOK_MC_PROC)(BYTE*, UINT32, bool, UINT64, void*);

void MicrophoneCapture_Startup();
void MicrophoneCapture_Cleanup();
void MicrophoneCapture_Open(bool raw);
void MicrophoneCapture_Close();
bool MicrophoneCapture_Status();
void MicrophoneCapture_ExecuteSensorLoop(HOOK_MC_PROC hook, void* param, HANDLE event_stop);
