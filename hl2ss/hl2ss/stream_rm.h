
#pragma once

#include "zenoh.h"

void RM_Initialize(const char* client_id, z_session_t session);
void RM_Quit();
void RM_Cleanup();
