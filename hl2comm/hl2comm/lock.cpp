
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

// OK
SRWLock::SRWLock(SRWLOCK *psrwl, bool exclusive)
{
	m_psrwl = psrwl;
	m_exclusive = exclusive;
	if (exclusive) { AcquireSRWLockExclusive(psrwl); } else { AcquireSRWLockShared(psrwl); }
}

// OK
SRWLock::~SRWLock()
{
	if (m_exclusive) { ReleaseSRWLockExclusive(m_psrwl); } else { ReleaseSRWLockShared(m_psrwl); }
}
