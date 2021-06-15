#ifndef QUADROCOPTER_MSVC_FIXES_H
#define QUADROCOPTER_MSVC_FIXES_H

#ifdef _MSC_VER

#include <intrin.h>
#define __builtin_popcount __popcnt
#define __builtin_popcountll _mm_popcnt_u64

#define _USE_MATH_DEFINES
#include <cmath>
#endif

#endif //QUADROCOPTER_MSVC_FIXES_H
