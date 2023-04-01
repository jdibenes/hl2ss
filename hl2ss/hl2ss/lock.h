
#pragma once

#include <Windows.h>

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
