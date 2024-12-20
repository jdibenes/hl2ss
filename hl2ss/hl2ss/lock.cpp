
#include "lock.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
CriticalSection::CriticalSection(CRITICAL_SECTION* pcs)
{
	m_pcs = pcs;
	EnterCriticalSection(m_pcs);
}

// OK
CriticalSection::~CriticalSection()
{
	LeaveCriticalSection(m_pcs);
}

// OK
SRWLock::SRWLock(SRWLOCK *psrwl, bool exclusive)
{
	m_psrwl = psrwl;
	m_exclusive = exclusive;
	if (m_exclusive) { AcquireSRWLockExclusive(m_psrwl); } else { AcquireSRWLockShared(m_psrwl); }
}

// OK
SRWLock::~SRWLock()
{
	if (m_exclusive) { ReleaseSRWLockExclusive(m_psrwl); } else { ReleaseSRWLockShared(m_psrwl); }
}

// OK
Cleaner::Cleaner(std::function<void()> f)
{
	m_f = f;
	m_enable = true;
}

// OK
Cleaner::~Cleaner()
{
	if (m_enable) { m_f(); }
}

// OK
void Cleaner::Set(bool enable)
{
	m_enable = enable;
}

// OK
NamedMutex::NamedMutex()
{
	m_mutex = NULL;
}

// OK
NamedMutex::~NamedMutex()
{
	Close();
}

// OK
bool NamedMutex::Create(wchar_t const* name)
{
	if (m_mutex != NULL) { Close(); }
	if ((name == NULL) || (wcslen(name) <= 0)) { return false; }
	m_mutex = CreateMutex(NULL, FALSE, name);
	return m_mutex != NULL;
}

// OK
void NamedMutex::Close()
{
	if (m_mutex == NULL) { return; }
	CloseHandle(m_mutex);
	m_mutex = NULL;
}

// OK
bool NamedMutex::Acquire(DWORD timeout) const
{
	if (m_mutex == NULL) { return true; }
	return WaitForSingleObject(m_mutex, timeout) == WAIT_OBJECT_0;
}

// OK
bool NamedMutex::Release() const
{
	if (m_mutex == NULL) { return true; }
	return ReleaseMutex(m_mutex) != 0;
}
