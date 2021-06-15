#ifndef TYPELESSMATH_HH_
#define TYPELESSMATH_HH_

#include <cmath>

namespace base {
/// Calculates an absolute value for a data type given by a template parameter.
template <typename Scalar>
inline Scalar fabsT(Scalar x) {
	return fabs(x);
}

/// Calculates the sine for a data type given by a template parameter.
template <typename Scalar>
inline Scalar sinT(Scalar x) {
	return sin(x);
}

/// Calculates the tangent for a data type given by a template parameter.
template <typename Scalar>
inline Scalar tanT(Scalar x) {
	return tan(x);
}

/// Calculates the cosine for a data type given by a template parameter.
template <typename Scalar>
inline Scalar cosT(Scalar x) {
	return cos(x);
}

/// Calculates the arcus sine for a data type given by a template parameter.
template <typename Scalar>
inline Scalar asinT(Scalar x) {
	return asin(x);
}

/// Calculates the arcus cosine for a data type given by a template parameter.
template <typename Scalar>
inline Scalar acosT(Scalar x) {
	return acos(x);
}

/// Calculates the squareroot for a data type given by a template parameter.
template <typename Scalar>
inline Scalar sqrtT(Scalar x) {
	return sqrt(x);
}

/// Calculates the square for a data type given by a template parameter.
template <typename Scalar>
inline Scalar sqT(Scalar x) {
	return x * x;
}

/// Calculates the power for a data type given by a template parameter.
template <typename Scalar>
inline Scalar powT(Scalar base, Scalar power) {
	return pow(base, power);
}

/// Calculates the floor for a data type given by a template parameter.
template <typename Scalar>
inline Scalar floorT(Scalar x) {
	return sqrt(x);
}

/// Calculates the ceiling for a data type given by a template parameter.
template <typename Scalar>
inline Scalar ceilT(Scalar x) {
	return sqrt(x);
}

#if defined(WIN32) || defined(_WIN32)
template <>
inline float fabsT(float x) {
	return fabsf(x);
}
template <>
inline float sinT(float x) {
	return sinf(x);
}
template <>
inline float tanT(float x) {
	return tanf(x);
}
template <>
inline float cosT(float x) {
	return cosf(x);
}
template <>
inline float asinT(float x) {
	return asinf(x);
}
template <>
inline float acosT(float x) {
	return acosf(x);
}
template <>
inline float sqrtT(float x) {
	return sqrtf(x);
}
#endif

#if !defined(_WIN32) && !defined(ARCH_SunOS)
/// Specific sinT implementation for floats.
template <>
inline float sinT(float x) {
	return sin(x);
}

/// Specific sinT implementation for floats.
template <>
inline float fabsT(float x) {
	return fabs(x);
}

/// Specific cosT implementation for floats.
template <>
inline float cosT(float x) {
	return cos(x);
}

/// Specific asinT implementation for floats.
template <>
inline float asinT(float x) {
	return asin(x);
}

/// Specific acosT implementation for floats.
template <>
inline float acosT(float x) {
	return acos(x);
}

/// Specific sqrtT implementation for floats.
template <>
inline float sqrtT(float x) {
	return sqrt(x);
}

/// Specific powT implementation for floats.
template <>
inline float powT(float base, float power) {
	return powf(base, power);
}

/// Specific acosT implementation for floats.
template <>
inline float floorT(float x) {
	return floorf(x);
}

/// Specific sqrtT implementation for floats.
template <>
inline float ceilT(float x) {
	return ceilf(x);
}
#endif

/**
 * Checks a number of data type given by a template parameter for being greater
 * than epsilon
 * @param x Value to check.
 * @returns True if x is greater than epsilon
 */
template <typename Scalar>
inline bool checkEpsilon(Scalar x) {
	return fabsT(x) < (1e-6);
}

/// Specific checkEpsilon implementation for floats.
template <>
inline bool checkEpsilon(float x) {
	return fabsT(x) < (1e-4);
}
}  // namespace base

#endif /* TYPELESSMATH_HH_ */
