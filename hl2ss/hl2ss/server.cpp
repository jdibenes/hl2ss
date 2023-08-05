
#include <ws2tcpip.h>
#include "server.h"
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
	SOCKET listensocket;

	ZeroMemory(&hints, sizeof(hints));

	hints.ai_family   = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags    = AI_PASSIVE;

	getaddrinfo(NULL, port, &hints, &result);
	listensocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	bind(listensocket, result->ai_addr, (int)(result->ai_addrlen));
	freeaddrinfo(result);
	listen(listensocket, SOMAXCONN);

	return listensocket;
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

// OK
bool recv(SOCKET clientsocket, char* buf, int bytes)
{
	int status;

	while (bytes > 0)
	{
	status = recv(clientsocket, buf, bytes, 0);
	if ((status == SOCKET_ERROR) || (status == 0)) { return false; }
	buf   += status;
	bytes -= status;
	}

	return bytes == 0;
}

// OK
bool send_multiple(SOCKET s, LPWSABUF buffers, DWORD dwBufferCount)
{
	DWORD dwBytesSent;
	int status;

	status = WSASend(s, buffers, dwBufferCount, &dwBytesSent, 0, NULL, NULL);
	return status != SOCKET_ERROR;
}

// OK
void pack_buffer(WSABUF *dst, int index, void const *buffer, ULONG length)
{
	dst[index].buf = (char*)buffer;
	dst[index].len = length;
}
