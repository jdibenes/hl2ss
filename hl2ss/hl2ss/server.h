
#pragma once

#include <winsock2.h>

bool   InitializeSockets();
SOCKET CreateSocket(char const* port);
SOCKET CreateSocket(char const* target, uint16_t port, uint32_t& max_msg_size, SOCKADDR_IN& mc_addr);
bool   Select(SOCKET s);
void   CleanupSockets();

bool recv_u8(SOCKET socket, uint8_t& byte);
bool recv_u16(SOCKET socket, uint16_t& word);
bool recv_u32(SOCKET socket, uint32_t& dword);
bool recv(SOCKET clientsocket, char* buf, int bytes);

bool send_multiple(SOCKET s, LPWSABUF buffers, DWORD dwBufferCount);
bool send_multiple(SOCKET s, uint32_t max_msg_size, SOCKADDR const* mc_addr, LPWSABUF buffers, DWORD dwBufferCount, BOOL clean_point);
void pack_buffer(WSABUF* dst, int index, void const* buffer, ULONG length);
