
#pragma once

#include <stdint.h>

#define HNS_BASE (10ULL * 1000ULL * 1000ULL)

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef int8_t  s8;
typedef int16_t s16;
typedef int32_t s32;

union v8  { u8 b; s8 c; };
union v16 { struct { v8  b0, b1; } b; u16 w; s16 s; };
union v32 { struct { v16 w0, w1; } w; u32 d; s32 i; };
