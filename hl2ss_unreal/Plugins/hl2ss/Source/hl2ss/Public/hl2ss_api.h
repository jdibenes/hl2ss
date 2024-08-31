// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 
 */
class HL2SS_API hl2ss_api
{
private:
    static void* hmod_mmrsu;
    static void* hmod_mmret;
    static void* hmod_hl2ss;

    static void* p_InitializeStreams;
    static void* p_DebugMessage;
    static void* p_OverrideWorldCoordinateSystem;
    static void* p_GetLocalIPv4Address;

    static void* p_MQ_SI_Peek;
    static void* p_MQ_SI_Pop;
    static void* p_MQ_SO_Push;
    static void* p_MQ_Restart;

    static void* p_MQX_CO_Peek;
    static void* p_MQX_CO_Pop;
    static void* p_MQX_CI_Push;
    static void* p_MQX_Restart;

    static void* p_PersonalVideo_RegisterNamedMutex;
    static void* p_ExtendedVideo_RegisterNamedMutex;

    static int Load();

public:
    enum STREAM_ENABLE
    {
        ENABLE_RM  =    1,
        ENABLE_PV  =    2,
        ENABLE_MC  =    4,
        ENABLE_SI  =    8,
        ENABLE_RC  =   16,
        ENABLE_SM  =   32,
        ENABLE_SU  =   64,
        ENABLE_VI  =  128,
        ENABLE_MQ  =  256,
        ENABLE_EET =  512,
        ENABLE_EA  = 1024,
        ENABLE_EV  = 2048,
        ENABLE_MQX = 4096,

        ENABLE_ALL = 0xFFFFFFFF,
    };

    static uint32_t const QUEUE_EMPTY = 0xFFFFFFFF;
    static uint32_t const CLIENT_DISCONNECTED = 0xFFFFFFFF;

    static int Initialize();
    static void InitializeStreams(uint32_t enable);
    static void DebugMessage(char const* str);
    static int OverrideWorldCoordinateSystem(void* scs);
    static void GetLocalIPv4Address(wchar_t* buffer, int size);
    static uint32_t MQ_SI_Peek();
    static void MQ_SI_Pop(uint32_t& command, uint8_t* data);
    static void MQ_SO_Push(uint32_t id);
    static void MQ_Restart();
    static uint32_t MQX_CO_Peek();
    static void MQX_CO_Pop(uint32_t& id);
    static void MQX_CI_Push(uint32_t command, uint32_t size, uint8_t const* data);
    static void MQX_Restart();
    static void PersonalVideo_RegisterNamedMutex(wchar_t const* name);
    static void ExtendedVideo_RegisterNamedMutex(wchar_t const* name);
};
