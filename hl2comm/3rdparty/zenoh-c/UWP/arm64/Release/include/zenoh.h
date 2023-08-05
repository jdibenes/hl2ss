#ifndef ZENOH_H
#define ZENOH_H

#include <assert.h>
#include <stdint.h>

#include "zenoh_configure.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZENOH_C "0.7.2.0"
#define ZENOH_C_MAJOR 0
#define ZENOH_C_MINOR 7
#define ZENOH_C_PATCH 2
#define ZENOH_C_TWEAK 0

#ifdef _MSC_VER
#define ALIGN(n) __declspec(align(n))
#else
#define ALIGN(n) __attribute__((aligned(n)))
#endif

#include "zenoh_configure.h"

// clang-format off
// include order is important
#include "zenoh_concrete.h"
#include "zenoh_commons.h"
// clang-format on

#ifdef __cplusplus
}
#endif
#include "zenoh_macros.h"
#endif
