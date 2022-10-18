
#pragma once

#include <stdint.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t  s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

union v8  { u8 b; s8 c; };
union v16 { struct { v8  b0, b1; } b; u16 w; s16 s; };
union v32 { struct { v16 w0, w1; } w; u32 d; s32 i; };
union v64 { struct { v32 d0, d1; } d; u64 q; s64 l; };
