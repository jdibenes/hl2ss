
#pragma once

#include <winsock2.h>

bool   InitializeSockets();
SOCKET CreateSocket(char const* port);
void   CleanupSockets();

bool recv_u8(SOCKET socket, uint8_t& byte);
bool recv_u16(SOCKET socket, uint16_t& word);
bool recv_u32(SOCKET socket, uint32_t& dword);
bool recv(SOCKET clientsocket, char* buf, int bytes);

bool send_multiple(SOCKET s, LPWSABUF buffers, DWORD dwBufferCount);
void pack_buffer(WSABUF* dst, int index, void const* buffer, ULONG length);
