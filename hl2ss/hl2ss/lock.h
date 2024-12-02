
#pragma once

#include <Windows.h>
#include <functional>
#include "plugin.h"

class CriticalSection
{
private:
    CRITICAL_SECTION* m_pcs;

public:
    CriticalSection(CRITICAL_SECTION* pcs);
    ~CriticalSection();
};

class SRWLock
{
private:
    SRWLOCK* m_psrwl;
    bool m_exclusive;

public:
    SRWLock(SRWLOCK* psrwl, bool exclusive);
    ~SRWLock();
};

class Cleaner
{
private:
    std::function<void()> m_f;
    bool m_enable;

public:
    Cleaner(std::function<void()> f);
    ~Cleaner();
    void Set(bool enable);
};

class NamedMutex
{
private:
    HANDLE m_mutex;

public:
    NamedMutex();
    ~NamedMutex();
    bool Create(wchar_t const* name);
    void Close();
    bool Acquire(DWORD timeout) const;
    bool Release() const;
};

HL2SS_PLUGIN_EXPORT void* NamedMutex_Create(wchar_t const* name);
HL2SS_PLUGIN_EXPORT void NamedMutex_Destroy(void* p);
HL2SS_PLUGIN_EXPORT int NamedMutex_Acquire(void* p, uint32_t timeout);
HL2SS_PLUGIN_EXPORT int NamedMutex_Release(void* p);
