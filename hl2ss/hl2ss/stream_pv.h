
#pragma once

#include "zenoh.h"
#include "custom_sink_writers.h"


void PV_Initialize(const char* client_id, z_session_t session, bool enable_location, H26xFormat format);
void PV_Quit();
void PV_Cleanup();
