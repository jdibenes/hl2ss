
#pragma once

#include <winrt/Windows.Media.Capture.h>
#include <winrt/Windows.Media.Capture.Frames.h>

void PersonalVideo_Initialize(winrt::Windows::Media::Capture::MediaCapture const& mediaCapture, winrt::Windows::Media::Capture::Frames::MediaFrameSource& videoSource);
bool PersonalVideo_SetFormat(winrt::Windows::Media::Capture::Frames::MediaFrameSource const& videoSource, uint32_t width, uint32_t height, uint32_t framerate);
