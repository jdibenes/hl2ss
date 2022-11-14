
#include <Windows.h>
#include "lock.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
CriticalSection::CriticalSection(CRITICAL_SECTION* pcs)
{
	m_pcs = pcs;
	if (m_pcs) { EnterCriticalSection(m_pcs); }
}

// OK
CriticalSection::~CriticalSection()
{
	if (m_pcs) { LeaveCriticalSection(m_pcs); }
}
