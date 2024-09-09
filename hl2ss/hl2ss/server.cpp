
#include <vector>
#include <ws2tcpip.h>
#include "server.h"
#include "types.h"
#include "log.h"

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

// OK
SOCKET CreateSocket(char const* target, uint16_t port, uint32_t& max_msg_size, SOCKADDR_IN& mc_addr)
{
	int sizeofu32 = sizeof(uint32_t);
	DWORD loop = 0;
	DWORD ttl = 32;

	SOCKET s;
	IN_ADDR addr;

	s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL,  (char*)&ttl,           sizeof(ttl));
	setsockopt(s, IPPROTO_IP, IP_MULTICAST_LOOP, (char*)&loop,          sizeof(loop));
	getsockopt(s, SOL_SOCKET, SO_MAX_MSG_SIZE,   (char*)&max_msg_size, &sizeofu32);

	inet_pton(AF_INET, target, &addr);

	mc_addr.sin_family = AF_INET;
	mc_addr.sin_port = htons(port);
	mc_addr.sin_addr = addr;

	return s;
}

// OK
bool Select(SOCKET s)
{
	timeval tv{ 0, 0 };
	fd_set fs;

	fs.fd_count    = 1;
	fs.fd_array[0] = s;

	return select(1, &fs, NULL, NULL, &tv) == 1;
}

// OK
bool send_multiple(SOCKET s, uint32_t max_msg_size, SOCKADDR const* mc_addr, LPWSABUF buffers, DWORD dwBufferCount, BOOL clean_point)
{
	uint64_t header;

	max_msg_size -= sizeof(header);

	DWORD total_bytes = 0;
	for (DWORD i = 0; i < dwBufferCount; ++i) { total_bytes += buffers[i].len; }
	DWORD dgram_count = (total_bytes / max_msg_size) + ((total_bytes % max_msg_size) != 0);
	
	if (dgram_count <= 0) { return true; }

	max_msg_size = (total_bytes / dgram_count) + ((total_bytes % dgram_count) != 0);

	std::vector<WSABUF> block;
	DWORD dwBytesSent;
	int status;
	
	int32_t byte_count;
	WSABUF source = { 0, 0 };
	DWORD dgram_index = 0;
	DWORD dwBufferIndex = 0;
	int state = 0;
	DWORD total_sent = 0;

	while (dgram_index < dgram_count)
	{
		switch (state)
		{
		case 0:
			header = ((uint64_t)(clean_point != 0) << 32) | (uint64_t)(((dgram_count & 0xFFFF) << 16) | (dgram_index & 0xFFFF));
			byte_count = max_msg_size;
			block.push_back({ sizeof(header), (char*)&header });
			state = 1;
			break;
		case 1:
			state = (source.len <= 0) ? 2 : 3;
			break;
		case 2:
			if (dwBufferIndex < dwBufferCount)
			{ 
				source = buffers[dwBufferIndex++];
				state = 3;
			}
			else
			{
				state = 6;
			}
			break;
		case 3:
			state = (source.len <= byte_count) ? 4 : 5;
			break;
		case 4:
			block.push_back(source);
			byte_count -= source.len;
			source.len = 0;
			state = (byte_count <= 0) ? 6 : 2;
			break;
		case 5:
			block.push_back({ (ULONG)byte_count, source.buf });
			source.buf += byte_count;
			source.len -= byte_count;
			state = 6;
			break;
		case 6:
			status = WSASendTo(s, block.data(), block.size(), &dwBytesSent, 0, mc_addr, sizeof(SOCKADDR_IN), NULL, NULL);
			total_sent += dwBytesSent;
			if (status == SOCKET_ERROR) { return false; }
			block.clear();
			dgram_index++;			
			state = 0;
			break;
		}
	}

	return true;
}
