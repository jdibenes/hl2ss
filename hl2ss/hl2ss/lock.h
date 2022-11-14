
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
