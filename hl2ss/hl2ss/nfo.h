
#pragma once

#include <stdint.h>
#include <vector>

#include <winrt/Windows.Foundation.h>

void GetApplicationVersion(uint16_t data[4]);
void GetLocalIPv4Address(std::vector<wchar_t> &address);
void PrintSystemInfo();
winrt::hstring GetBuiltInVideoCaptureId();
winrt::hstring GetBuiltInAudioCaptureId();
void GetVideoCaptureIds(std::vector<winrt::hstring>& ids);
void GetAudioCaptureIds(std::vector<winrt::hstring>& ids);
void GetAudioCaptureIdsAndNames(std::vector<winrt::hstring>& ids, std::vector<winrt::hstring>& names);
