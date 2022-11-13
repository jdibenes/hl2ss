
#pragma once

#include <stdint.h>
#include <vector>

void GetApplicationVersion(uint16_t data[4]);
void GetLocalIPv4Address(std::vector<wchar_t> &address);
