
#pragma once

#include <stdint.h>
#include <vector>

#include <winrt/Windows.Foundation.h>

void GetApplicationVersion(uint16_t data[4]);
void GetLocalIPv4Address(std::vector<wchar_t> &address);
void PrintSystemInfo();
winrt::hstring GetBuiltInVideoCaptureId();
winrt::hstring GetBuiltInAudioCaptureId();
