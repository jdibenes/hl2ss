// Fill out your copyright notice in the Description page of Project Settings.


#include "hl2ss_api.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

typedef void(*pfn_InitializeStreams)(uint32_t enable);
typedef void(*pfn_DebugMessage)(char const* str);
typedef int(*pfn_OverrideWorldCoordinateSystem)(void* scs_ptr);
typedef void(*pfn_GetLocalIPv4Address)(wchar_t* buffer, int size);

typedef uint32_t(*pfn_MQ_SI_Peek)();
typedef void(*pfn_MQ_SI_Pop)(uint32_t& command, uint8_t* data);
typedef void(*pfn_MQ_SO_Push)(uint32_t id);
typedef void(*pfn_MQ_Restart)();

#if PLATFORM_HOLOLENS
extern "C" { HMODULE LoadLibraryW(LPCWSTR lpLibFileName); }
#endif

void* hl2ss_api::hmod_mmrsu = NULL;
void* hl2ss_api::hmod_mmret = NULL;
void* hl2ss_api::hmod_hl2ss = NULL;

void* hl2ss_api::p_InitializeStreams = NULL;
void* hl2ss_api::p_DebugMessage = NULL;
void* hl2ss_api::p_OverrideWorldCoordinateSystem = NULL;
void* hl2ss_api::p_GetLocalIPv4Address = NULL;

void* hl2ss_api::p_MQ_SI_Peek = NULL;
void* hl2ss_api::p_MQ_SI_Pop = NULL;
void* hl2ss_api::p_MQ_SO_Push = NULL;
void* hl2ss_api::p_MQ_Restart = NULL;

int hl2ss_api::Load()
{
    FString path_base = FPaths::ProjectPluginsDir() + L"hl2ss/Binaries/hl2ss/";

    path_base.ReplaceCharInline(L'/', L'\\');

    FString path_dll_mmrsu = path_base + L"Microsoft.MixedReality.SceneUnderstanding.dll";
    FString path_dll_mmret = path_base + L"Microsoft.MixedReality.EyeTracking.dll";
    FString path_dll_hl2ss = path_base + L"hl2ss.dll";

    if (!FPaths::FileExists(path_dll_mmrsu)) { return -1; }
    if (!FPaths::FileExists(path_dll_mmret)) { return -2; }
    if (!FPaths::FileExists(path_dll_hl2ss)) { return -3; }

    hmod_mmrsu = LoadLibraryW(*path_dll_mmrsu);
    hmod_mmret = LoadLibraryW(*path_dll_mmret);
    hmod_hl2ss = LoadLibraryW(*path_dll_hl2ss);

    if (hmod_mmrsu == NULL) { return -4; }
    if (hmod_mmret == NULL) { return -5; }
    if (hmod_hl2ss == NULL) { return -6; }

    p_InitializeStreams = GetProcAddress((HMODULE)hmod_hl2ss, "InitializeStreams");
    p_DebugMessage = GetProcAddress((HMODULE)hmod_hl2ss, "DebugMessage");
    p_OverrideWorldCoordinateSystem = GetProcAddress((HMODULE)hmod_hl2ss, "OverrideWorldCoordinateSystem");
    p_GetLocalIPv4Address = GetProcAddress((HMODULE)hmod_hl2ss, "GetLocalIPv4Address");

    p_MQ_SI_Peek = GetProcAddress((HMODULE)hmod_hl2ss, "MQ_SI_Peek");
    p_MQ_SI_Pop = GetProcAddress((HMODULE)hmod_hl2ss, "MQ_SI_Pop");
    p_MQ_SO_Push = GetProcAddress((HMODULE)hmod_hl2ss, "MQ_SO_Push");
    p_MQ_Restart = GetProcAddress((HMODULE)hmod_hl2ss, "MQ_Restart");

    if (p_InitializeStreams == NULL) { return -7; }
    if (p_DebugMessage == NULL) { return -8; }
    if (p_OverrideWorldCoordinateSystem == NULL) { return -9; }
    if (p_GetLocalIPv4Address == NULL) { return -10; }

    if (p_MQ_SI_Peek == NULL) { return -11; }
    if (p_MQ_SI_Pop == NULL) { return -12; }
    if (p_MQ_SO_Push == NULL) { return -13; }
    if (p_MQ_Restart == NULL) { return -14; }

    return 1;
}

int hl2ss_api::Initialize()
{
    int initialize_result = Load();
    OutputDebugStringW(*(L"hl2ss::Load returned " + FString::FromInt(initialize_result) + L"\n"));
    return initialize_result;
}

void hl2ss_api::InitializeStreams(uint32_t enable)
{
    if (p_InitializeStreams == NULL) { return; }
    reinterpret_cast<pfn_InitializeStreams>(p_InitializeStreams)(enable);
}

void hl2ss_api::DebugMessage(char const* str)
{
    if (p_DebugMessage == NULL) { return; }
    reinterpret_cast<pfn_DebugMessage>(p_DebugMessage)(str);
}

int hl2ss_api::OverrideWorldCoordinateSystem(void* scs)
{
    if (p_OverrideWorldCoordinateSystem == NULL) { return false; }
    return reinterpret_cast<pfn_OverrideWorldCoordinateSystem>(p_OverrideWorldCoordinateSystem)(scs);
}

void hl2ss_api::GetLocalIPv4Address(wchar_t* buffer, int size)
{
    if (p_GetLocalIPv4Address == NULL) { return; }
    reinterpret_cast<pfn_GetLocalIPv4Address>(p_GetLocalIPv4Address)(buffer, size);
}

uint32_t hl2ss_api::MQ_SI_Peek()
{
    if (p_MQ_SI_Peek == NULL) { return 0xFFFFFFFF; }
    return reinterpret_cast<pfn_MQ_SI_Peek>(p_MQ_SI_Peek)();
}

void hl2ss_api::MQ_SI_Pop(uint32_t& command, uint8_t* data)
{
    if (p_MQ_SI_Pop == NULL) { return; }
    reinterpret_cast<pfn_MQ_SI_Pop>(p_MQ_SI_Pop)(command, data);
}

void hl2ss_api::MQ_SO_Push(uint32_t id)
{
    if (p_MQ_SO_Push == NULL) { return; }
    reinterpret_cast<pfn_MQ_SO_Push>(p_MQ_SO_Push)(id);
}

void hl2ss_api::MQ_Restart()
{
    if (p_MQ_Restart == NULL) { return; }
    reinterpret_cast<pfn_MQ_Restart>(p_MQ_Restart)();
}
