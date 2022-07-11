
#include <ws2tcpip.h>
#include "server.h"
#include "utilities.h"
#include "types.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
bool InitializeSockets()
{
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	return iResult == 0;
}

// OK
SOCKET CreateSocket(char const* port)
{
	addrinfo hints;
	addrinfo* result;
	int iResult;
	SOCKET listensocket;

	ZeroMemory(&hints, sizeof(hints));

	hints.ai_family   = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags    = AI_PASSIVE;

	iResult = getaddrinfo(NULL, port, &hints, &result);
	if (iResult != 0) { return INVALID_SOCKET; }

	listensocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (listensocket != INVALID_SOCKET) { iResult = bind(listensocket, result->ai_addr, (int)(result->ai_addrlen)); }
	freeaddrinfo(result);
	if (listensocket == INVALID_SOCKET) { return INVALID_SOCKET; }
	if (iResult == SOCKET_ERROR) { goto _socket_error; }

	iResult = listen(listensocket, SOMAXCONN);
	if (iResult == SOCKET_ERROR) { goto _socket_error; }

	return listensocket;

_socket_error:
	closesocket(listensocket);
	return INVALID_SOCKET;
}

// OK
void CleanupSockets()
{
	WSACleanup();
}

// OK
bool recv_u8(SOCKET socket, uint8_t& byte)
{
	v8 buf;

	int status;
	status = recv(socket, (char*)&buf.b, 1, 0);
	if (status != 1) { return false; }
	byte = buf.b;
	return true;
}

// OK
bool recv_u16(SOCKET socket, uint16_t& word)
{
	v16 buf;

	int status;
	status = recv(socket, (char*)&buf.b.b0.b, 1, 0);
	if (status != 1) { return false; }
	status = recv(socket, (char*)&buf.b.b1.b, 1, 0);
	if (status != 1) { return false; }
	word = buf.w;
	return true;
}

// OK
bool recv_u32(SOCKET socket, uint32_t& dword)
{
	v32 buf;

	int status;
	status = recv(socket, (char*)&buf.w.w0.b.b0.b, 1, 0);
	if (status != 1) { return false; }
	status = recv(socket, (char*)&buf.w.w0.b.b1.b, 1, 0);
	if (status != 1) { return false; }
	status = recv(socket, (char*)&buf.w.w1.b.b0.b, 1, 0);
	if (status != 1) { return false; }
	status = recv(socket, (char*)&buf.w.w1.b.b1.b, 1, 0);
	if (status != 1) { return false; }
	dword = buf.d;
	return true;
}

bool send_multiple(SOCKET s, LPWSABUF buffers, DWORD dwBufferCount)
{
	int status;
	DWORD value;

	value = FALSE;
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char*)&value, sizeof(value));
	for (DWORD i = 0; i < dwBufferCount; ++i)
	{
		status = send(s, buffers[i].buf, buffers[i].len, 0);
		if ((ULONG)status != buffers[i].len) { return false; }
	}
	value = TRUE;
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char*)&value, sizeof(value));
	send(s, NULL, 0, 0);

	return true;

	//code below does not work well for some reason
	//status = WSASend(s, buffers, dwBufferCount, &dwBytesSent, 0, NULL, NULL);
	//return status != SOCKET_ERROR;
}
