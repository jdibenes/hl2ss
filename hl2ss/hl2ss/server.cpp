
#include <ws2tcpip.h>
#include "extended_execution.h"
#include "server.h"
#include "lock.h"
#include "types.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Networking.h>
#include <winrt/Windows.Networking.Connectivity.h>

using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Networking;
using namespace winrt::Windows::Networking::Connectivity;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
bool Server_Startup()
{
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	return iResult == 0;
}

// OK
bool Server_Cleanup()
{
	int status = WSACleanup();
	return status == 0;
}

// OK
SOCKET Server_CreateSocket(char const* port)
{
	addrinfo hints;
	addrinfo* result;
	SOCKET listensocket;
	int status;

	ZeroMemory(&hints, sizeof(hints));

	hints.ai_family   = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags    = AI_PASSIVE;

	status = getaddrinfo(NULL, port, &hints, &result);
	if (status != 0) { return INVALID_SOCKET; }

	Cleaner free_info([=]() { freeaddrinfo(result); });

	listensocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (listensocket == INVALID_SOCKET) { return INVALID_SOCKET; }

	Cleaner free_socket([=]() { closesocket(listensocket); });

	status = bind(listensocket, result->ai_addr, static_cast<int>(result->ai_addrlen));
	if (status != 0) { return INVALID_SOCKET; }

	status = listen(listensocket, SOMAXCONN);
	if (status != 0) { return INVALID_SOCKET; }

	free_socket.Set(false);

	return listensocket;
}

// OK
SOCKET Server_AcceptClient(SOCKET socket, DWORD nodelay)
{
	SOCKET client = accept(socket, NULL, NULL);

	setsockopt(client, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&nodelay), sizeof(nodelay));

	return client;
}

// OK
winrt::hstring Server_GetLocalIPv4Address()
{
    for (auto const& hostname : NetworkInformation::GetHostNames())
    {
    if (hostname.Type() == HostNameType::Ipv4) { return hostname.ToString(); }
    }

	ExtendedExecution_EnterException(Exception::Exception_UnknownNetworkAddress);
    return L"0.0.0.0";
}

// OK
static bool recv_u8(SOCKET socket, uint8_t& byte)
{
	v8& buf = reinterpret_cast<v8&>(byte);
	int status;

	status = recv(socket, &buf.x, 1, 0);
	if (status != 1) { return false; }

	return true;
}

// OK
static bool recv_u16(SOCKET socket, uint16_t& word)
{
	v16& buf = reinterpret_cast<v16&>(word);
	bool ok;

	ok = recv_u8(socket, buf.b.b0.b);
	if (!ok) { return false; }
	ok = recv_u8(socket, buf.b.b1.b);
	if (!ok) { return false; }

	return true;
}

// OK
static bool recv_u32(SOCKET socket, uint32_t& dword)
{
	v32& buf = reinterpret_cast<v32&>(dword);
	bool ok;

	ok = recv_u16(socket, buf.w.w0.w);
	if (!ok) { return false; }
	ok = recv_u16(socket, buf.w.w1.w);
	if (!ok) { return false; }

	return true;
}

// OK
static bool recv(SOCKET socket, void* buffer, int bytes)
{
	char* dst = static_cast<char*>(buffer);

	while (bytes > 0)
	{
	int status = recv(socket, dst, bytes, 0);
	if ((status == SOCKET_ERROR) || (status == 0)) { return false; }
	dst   += status;
	bytes -= status;
	}

	return bytes == 0;
}

// OK
static bool send_multiple(SOCKET socket, LPWSABUF buffers, DWORD buffer_count)
{
	DWORD bytes_sent;
	int status = WSASend(socket, buffers, buffer_count, &bytes_sent, 0, NULL, NULL);
	return status != SOCKET_ERROR;
}

// OK
bool recv_u8(SOCKET socket, HANDLE event_error, uint8_t& byte)
{
	bool ok = recv_u8(socket, byte);
	if (!ok) { SetEvent(event_error); }
	return ok;
}

// OK
bool recv_u16(SOCKET socket, HANDLE event_error, uint16_t& word)
{
	bool ok = recv_u16(socket, word);
	if (!ok) { SetEvent(event_error); }
	return ok;
}

// OK
bool recv_u32(SOCKET socket, HANDLE event_error, uint32_t& dword)
{
	bool ok = recv_u32(socket, dword);
	if (!ok) { SetEvent(event_error); }
	return ok;
}

// OK
bool recv(SOCKET socket, HANDLE event_error, void* buffer, int buffer_size)
{
	bool ok = recv(socket, buffer, buffer_size);
	if (!ok) { SetEvent(event_error); }
	return ok;
}

// OK
void pack_buffer(LPWSABUF buffers, int index, void const* buffer, ULONG buffer_size)
{
	buffers[index].buf = static_cast<char*>(const_cast<void*>(buffer));
	buffers[index].len = buffer_size;
}

// OK
bool send_multiple(SOCKET socket, HANDLE event_error, LPWSABUF buffers, DWORD buffer_count)
{
	bool ok = send_multiple(socket, buffers, buffer_count);
	if (!ok) { SetEvent(event_error); }
	return ok;
}
