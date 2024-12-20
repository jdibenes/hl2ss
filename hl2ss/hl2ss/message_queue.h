
#pragma once

#include <Windows.h>
#include <stdint.h>

struct MQ_Item
{
public:
	uint32_t command;
	uint32_t size;
	uint32_t data[1];

private:
	MQ_Item();
	~MQ_Item();
};

uint32_t const MQ_MARKER = ~0UL;

MQ_Item* MessageQueue_Item_Create(uint32_t command, uint32_t size);
void MessageQueue_Item_Delete(MQ_Item* item);

void MessageQueue_Server_Startup();
void MessageQueue_Server_Cleanup();
void MessageQueue_Server_RX_Push(MQ_Item* item);
uint32_t MessageQueue_Server_RX_Peek(); // USER
void MessageQueue_Server_RX_Pull(uint32_t& command, void* data); // USER
void MessageQueue_Server_TX_Push(uint32_t id); // USER
bool MessageQueue_Server_TX_Pull(HANDLE event_cancel, uint32_t& id);
void MessageQueue_Server_Restart(); // USER
void MessageQueue_Server_WaitForRestart();

void MessageQueue_Client_Startup();
void MessageQueue_Client_Cleanup();
void MessageQueue_Client_RX_Push(uint32_t id);
uint32_t MessageQueue_Client_RX_Peek(); // USER
void MessageQueue_Client_RX_Pull(uint32_t& id); // USER
void MessageQueue_Client_TX_Push(uint32_t command, uint32_t size, void const* data); // USER
MQ_Item* MessageQueue_Client_TX_Pull();
void MessageQueue_Client_Restart(); // USER
void MessageQueue_Client_WaitForRestart();
