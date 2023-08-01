
#pragma once

#include<stdio.h>
#include <string>
#include <stdio.h>
#include <sstream>
#include <spdlog/spdlog.h>

// from https://stackoverflow.com/questions/43732825/use-debug-log-from-c
//Create a callback delegate
typedef void(*LoggingFuncCallBack)(const char* message, int color, int size);


void SetupCallbackLogSink(LoggingFuncCallBack cb);
void SetupDebugLogSink();
void SetupFileLogSink(const char* filename);


//void ShowMessage(const char* format, ...);
//void ShowMessage(const wchar_t* format, ...);


std::string wide_string_to_string(const std::wstring& wide_string);

template<typename... Args>
inline void ShowMessage(Args&&... args) {
    spdlog::info(std::forward<Args>(args)...);
}
//void ShowMessage(const char* format, ...);
//void ShowMessage(const wchar_t* format, ...);
