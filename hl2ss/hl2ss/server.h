
#pragma once

#include <winsock2.h>

#include <winrt/Windows.Foundation.h>

bool Server_Startup();
bool Server_Cleanup();

SOCKET Server_CreateSocket(char const* port);
SOCKET Server_AcceptClient(SOCKET socket, DWORD nodelay);

winrt::hstring Server_GetLocalIPv4Address();

bool recv_u8(SOCKET socket, HANDLE event_error, uint8_t& byte);
bool recv_u16(SOCKET socket, HANDLE event_error, uint16_t& word);
bool recv_u32(SOCKET socket, HANDLE event_error, uint32_t& dword);
bool recv(SOCKET socket, HANDLE event_error, void* buffer, int buffer_size);

void pack_buffer(LPWSABUF buffers, int index, void const* buffer, ULONG buffer_size);
bool send_multiple(SOCKET socket, HANDLE event_error, LPWSABUF buffers, DWORD buffer_count);
