
#include <Windows.h>
#include <new>
#include "lock.h"
#include "log.h"

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

// OK
NamedMutex::NamedMutex() : m_mutex(NULL)
{
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

// OK
void* NamedMutex_Create(wchar_t const* name)
{
	NamedMutex* mutex = new (std::nothrow) NamedMutex(); // delete
	if (mutex == nullptr) { return NULL; }
	if (mutex->Create(name)) { return mutex; }
	NamedMutex_Destroy(mutex);
	return NULL;
}

// OK
void NamedMutex_Destroy(void* p)
{
	delete (NamedMutex*)p;
}

// OK
int NamedMutex_Acquire(void* p, uint32_t timeout)
{
	return ((NamedMutex*)p)->Acquire(timeout);
}

// OK
int NamedMutex_Release(void* p)
{
	return ((NamedMutex*)p)->Release();
}
