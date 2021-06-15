#ifndef VECTOR_HH
#define VECTOR_HH

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include "MSVC_Fixes.h"

#include <string.h>

#include <algorithm>
#include <iostream>

#include <assert.h>
#include <math.h>
#include <vector>

#include <typeinfo>

#if defined(ARCH_cygwin) || defined(ARCH_SUNOS) || defined(COMP_bc)
inline float sqrtf(float x) {
	return float(sqrt(double(x)));
}
inline float fabsf(float x) {
	return float(fabs(double(x)));
}
#endif

namespace base {

/**
 * A vector is an array of N values of type Scalar.
 */
template <typename Scalar, int N>
class Vector {
   public:
	/** the type of the scalar used in this template */
	typedef Scalar value_type;
	/** returns dimension N of the vector */
	static inline int dim() { return N; }

	/** default constructor creates uninitialized values. */
	inline Vector() {}

	/** constructor taking 1 explicit scalar value; only valid for
	 1D templates. */
	explicit inline Vector(const Scalar& v0) {
		assert(N == 1);
		values[0] = v0;
	}

	/** constructor taking 2 explicit scalar values; only valid for
	 2D templates. */
	inline Vector(const Scalar& v0, const Scalar& v1) {
		assert(N == 2);
		values[0] = v0;
		values[1] = v1;
	}

	/** constructor taking 3 explicit scalar values; only valid for
	 3D templates. */
	inline Vector(const Scalar& v0, const Scalar& v1, const Scalar& v2) {
		assert(N == 3);
		values[0] = v0;
		values[1] = v1;
		values[2] = v2;
	}

	/** constructor taking 4 explicit scalar values; only valid for
	 4D templates. */
	inline Vector(const Scalar& v0,
				  const Scalar& v1,
				  const Scalar& v2,
				  const Scalar& v3) {
		assert(N == 4);
		values[0] = v0;
		values[1] = v1;
		values[2] = v2;
		values[3] = v3;
	}

	/** constructor taking 5 explicit scalar values; only valid for
	 5D templates. */
	inline Vector(const Scalar& v0,
				  const Scalar& v1,
				  const Scalar& v2,
				  const Scalar& v3,
				  const Scalar& v4) {
		assert(N == 5);
		values[0] = v0;
		values[1] = v1;
		values[2] = v2;
		values[3] = v3;
		values[4] = v4;
	}

	/** constructor taking 6 explicit scalar values; only valid for
	 specialized 6D templates. */
	inline Vector(const Scalar& v0,
				  const Scalar& v1,
				  const Scalar& v2,
				  const Scalar& v3,
				  const Scalar& v4,
				  const Scalar& v5) {
		assert(N == 6);
		values[0] = v0;
		values[1] = v1;
		values[2] = v2;
		values[3] = v3;
		values[4] = v4;
		values[5] = v5;
	}

	/** constructor taking 8 explicit scalar values; only valid for
	 specialized 8D templates. */
	inline Vector(const Scalar& v0,
				  const Scalar& v1,
				  const Scalar& v2,
				  const Scalar& v3,
				  const Scalar& v4,
				  const Scalar& v5,
				  const Scalar& v6,
				  const Scalar& v7) {
		assert(N == 8);
		values[0] = v0;
		values[1] = v1;
		values[2] = v2;
		values[3] = v3;
		values[4] = v4;
		values[5] = v5;
		values[6] = v6;
		values[7] = v7;
	}

	/// Construct from a value array
	explicit inline Vector(const Scalar _values[N]) {
		memcpy(values, _values, N * sizeof(Scalar));
	}

	/// copy constructor (same kind of vector)
#ifndef COMP_VC
	inline Vector(const Vector<Scalar, N>& o) { operator=(o); }
#endif

	/// copy&cast constructor
	template <typename otherScalarType>
	inline Vector(const Vector<otherScalarType, N>& o) {
		for (int i = 0; i < N; ++i)
			values[i] = (Scalar)o[i];
	}

	/// assignment from a vector of the same kind
#ifndef COMP_VC
	inline Vector<Scalar, N>& operator=(const Vector<Scalar, N>& o) {
		for (int i = 0; i < N; i++)
			values[i] = o.values[i];
		return *this;
	}
#endif

	//   /// cast from vector with a different scalar type
	//   template<typename otherScalarType>
	//   inline Vector<Scalar,N> &operator=(const Vector<otherScalarType,N> &o)
	//   { for(int i=0; i<N; i++) values[i] = Scalar(o[i]); return *this; }

	/// cast to Scalar array
	inline operator Scalar*() { return values; }

	/// cast to const Scalar array
	inline operator const Scalar*() const { return values; }

	/// get i'th element read-write
	inline Scalar& operator[](int i) {
		assert(i >= 0 && i < N);
		return values[i];
	}

	/// get i'th element read-only
	inline const Scalar& operator[](int i) const {
		assert(i >= 0 && i < N);
		return values[i];
	}

	/// component-wise comparison
	inline bool operator==(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++)
			if (values[i] != other.values[i])
				return false;
		return true;
	}

	/// component-wise comparison
	inline bool operator!=(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++)
			if (values[i] != other.values[i])
				return true;
		return false;
	}

	/// component-wise greater
	inline bool gt(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++)
			if (values[i] <= other.values[i])
				return false;
		return true;
	}

	/// component-wise greater
	inline bool operator>(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++) {
			if (values[i] > other.values[i])
				return true;
			if (values[i] < other.values[i])
				return false;
		}
		return false;
	}

	/// component-wise geq
	inline bool geq(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++)
			if (values[i] < other.values[i])
				return false;
		return true;
	}

	inline bool operator>=(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++) {
			if (values[i] > other.values[i])
				return true;
			if (values[i] < other.values[i])
				return false;
		}
		return true;
	}

	/// component-wise lesser
	inline bool le(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++)
			if (values[i] >= other.values[i])
				return false;
		return true;
	}

	inline bool operator<(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++) {
			if (values[i] < other.values[i])
				return true;
			if (values[i] > other.values[i])
				return false;
		}
		return false;
	}

	/// component-wise leq
	inline bool leq(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++)
			if (values[i] > other.values[i])
				return false;
		return true;
	}

	inline bool operator<=(const Vector<Scalar, N>& other) const {
		for (int i = 0; i < N; i++) {
			if (values[i] < other.values[i])
				return true;
			if (values[i] > other.values[i])
				return false;
		}
		return true;
	}

	/// component-wise absolute value
	inline Vector<Scalar, N> abs() {
		for (int i = 0; i < N; i++)
			values[i] = values[i] < 0 ? -1 * values[i] : values[i];
		return *this;
	}

	// norm computations
	// Common vector types have faster, unrolled implementation.
	inline Scalar norm() const {
		Scalar sum = (Scalar)0.0;
		for (unsigned int d = 0; d < N; d++)
			sum += values[d] * values[d];
		return sqrt(sum);
	}

	/// compute squared norm; leave vector unchanged
	inline Scalar sqrnorm() const { return dot(*this); }

	/// normalize vector, return normalized vector
	inline Vector<Scalar, N>& normalize() {
		Scalar n = norm();
		if (n == 0.)
			assert(!"normalizing zero-vector");
		*this *= Scalar(1.0) / n;
		return *this;
	}

	/// return normalized vector
	inline Vector<Scalar, N> normalized() {
		Vector<Scalar, N> result = *this;
		result.normalize();
		return result;
	}

	/// normalize vector, return normalized vector, write norm to *n
	inline Vector<Scalar, N>& normalize(Scalar* n) {
		*n = norm();
		if (*n == 0.)
			assert(!"normalizing zero-vector");
		*this *= 1. / *n;
		return *this;
	}

	/// return the maximal component
#ifdef _WIN32
#undef max
#undef min
#endif
	inline Scalar max() const {
		Scalar m;
		bool first = true;
		for (int i = 0; i < N; i++)
			if (first) {
				m = values[i];
				first = false;
			} else if (values[i] > m)
				m = values[i];
		return m;
	}

	/// return the minimal component
	inline Scalar min() const {
		Scalar m;
		bool first = true;
		for (int i = 0; i < N; i++)
			if (first) {
				m = values[i];
				first = false;
			} else if (values[i] < m)
				m = values[i];
		return m;
	}

	/// return arithmetic mean
	inline Scalar arith_mean() const {
		Scalar m;
		bool first = true;
		for (int i = 0; i < N; i++)
			if (first) {
				m = values[i];
				first = false;
			} else
				m += values[i];
		return m / Scalar(N);
	}

	/// component-wise min
	inline Vector<Scalar, N> min(const Vector<Scalar, N>& other) const {
		Vector<Scalar, N> res;
		for (int i = 0; i < N; i++)
			res[i] = std::min(values[i], other[i]);
		return res;
	}

	/// component-wise max
	inline Vector<Scalar, N> max(const Vector<Scalar, N>& other) const {
		Vector<Scalar, N> res;
		for (int i = 0; i < N; i++)
			res[i] = std::max(values[i], other[i]);
		return res;
	}

	/// minimize values: same as *this = min(*this,other), but faster
	inline Vector<Scalar, N> minimize(const Vector<Scalar, N>& other) {
		for (int i = 0; i < N; i++)
			if (other[i] < values[i])
				values[i] = other[i];
		return *this;
	}

	/// maximize values: same as *this = max(*this,other), but faster
	inline Vector<Scalar, N> maximize(const Vector<Scalar, N>& other) {
		for (int i = 0; i < N; i++)
			if (other[i] > values[i])
				values[i] = other[i];
		return *this;
	}

	/// compute scalar product with another vector of same type
	inline Scalar operator|(const Vector<Scalar, N>& other) const {
		bool first = true;
		Scalar p;
		for (int i = 0; i < N; i++)
			if (first) {
				first = false;
				p = values[i] * other.values[i];
			} else
				p += values[i] * other.values[i];
		return p;
	}

	/// dot product: alias for operator| ; cf. base::dot(Vector,Vector)
	inline Scalar dot(const Vector<Scalar, N>& other) const {
		return operator|(other);
	}

	/// component-wise self-multiplication with scalar
	inline const Vector<Scalar, N>& operator*=(const Scalar& s) {
		for (int i = 0; i < N; i++)
			values[i] *= s;
		return *this;
	}

	/// component-wise multiplication with scalar
	inline Vector<Scalar, N> operator*(const Scalar& s) const {
		Vector<Scalar, N> v(*this);
		return v *= s;
	}

	/// component-wise self-multiplication
	inline const Vector<Scalar, N>& operator*=(const Vector<Scalar, N>& rhs) {
		for (int i = 0; i < N; i++)
			values[i] *= rhs[i];
		return *this;
	}

	/// component-wise multiplication
	inline Vector<Scalar, N> operator*(const Vector<Scalar, N>& rhs) const {
		Vector<Scalar, N> v(*this);
		return v *= rhs;
	}

	/// component-wise self-division by scalar
	inline const Vector<Scalar, N>& operator/=(const Scalar& s) {
		for (int i = 0; i < N; i++)
			values[i] /= s;
		return *this;
	}

	/// component-wise division by scalar
	inline Vector<Scalar, N> operator/(const Scalar& s) const {
		Vector<Scalar, N> v(*this);
		return v /= s;
	}

	/// component-wise self-division
	inline const Vector<Scalar, N>& operator/=(const Vector<Scalar, N>& rhs) {
		for (int i = 0; i < N; i++)
			values[i] /= rhs[i];
		return *this;
	}

	/// component-wise division
	inline Vector<Scalar, N> operator/(const Vector<Scalar, N>& rhs) const {
		Vector<Scalar, N> v(*this);
		return v /= rhs;
	}

	/// vector difference from this
	inline Vector<Scalar, N>& operator-=(const Vector<Scalar, N>& other) {
		for (int i = 0; i < N; i++)
			values[i] -= other.values[i];
		return *this;
	}

	/// vector difference
	inline Vector<Scalar, N> operator-(const Vector<Scalar, N>& other) const {
		Vector<Scalar, N> v(*this);
		v -= other;
		return v;
	}

	/// vector self-addition
	inline Vector<Scalar, N>& operator+=(const Vector<Scalar, N>& other) {
		for (int i = 0; i < N; i++)
			values[i] += other.values[i];
		return *this;
	}

	/// vector addition
	inline Vector<Scalar, N> operator+(const Vector<Scalar, N>& other) const {
		Vector<Scalar, N> v(*this);
		v += other;
		return v;
	}

	/// unary minus
	inline Vector<Scalar, N> operator-(void) const {
		Vector<Scalar, N> v(*this);
		for (int i = 0; i < N; i++)
			v.values[i] = -v.values[i];
		return v;
	}

	/** cross product: only defined for Vec3f/Vec3d as specialization */
	inline Vector<Scalar, N> operator%(const Vector<Scalar, N>& other) const {
		assert(!"cross product not defined for this type");
		return Vector<Scalar, N>();
	}

	/** cross product: only defined for Vec3f/Vec3d as specialization */
	inline Vector<Scalar, N> cross(const Vector<Scalar, N>& other) const {
		return operator%(other);
	}

	inline int hamming(const Vector<Scalar, N>& other) const {
		assert(!"Not implemted for general types.");
		return 0;
	}

	/** central projection 4D->3D (w=1). this is only defined for 4D. */
	inline Vector<Scalar, 3> centralProjection() const {
		assert(!"centralProjection not defined for this type");
		return Vector<Scalar, 3>();
	}

	/** projects the vector into a plane (3D) normal to the given vector, which
	 must have unit length. self is modified and the new vector is returned. */
	inline const Vector<Scalar, N>& projectNormalTo(
		const Vector<Scalar, N>& v) {
		Scalar sprod = (*this | v);
		for (int i = 0; i < N; i++)
			values[i] -= (v.values[i] * sprod);
		return *this;
	}

	/** component-wise apply function object with Scalar operator()(Scalar). */
	template <typename func>
	inline Vector<Scalar, N> apply(const func& f) const {
		Vector<Scalar, N> result;
		for (int i = 0; i < N; i++)
			result[i] = f(values[i]);
		return result;
	}

	/** compose vector containing the same value in each component */
	static inline Vector<Scalar, N> vectorize(Scalar value) {
		Vector<Scalar, N> result;
		for (int i = 0; i < N; i++)
			result[i] = value;
		return result;
	}

	/** set all values in vector to 0 */
	inline void clear() {
		for (int i = 0; i < N; i++)
			values[i] = (Scalar)0;
	}

	float tripleProduct(const Vector<Scalar, N>& B,
						const Vector<Scalar, N>& C) const {
		const Vector<Scalar, N> tmp = B.cross(C);
		return dot(tmp);
	}

	/** Remove component parallel to A. */
	void remove(const Vector<Scalar, N>& A) {
		const Scalar tmp = dot(A) / A.sqrnorm();
		*this -= tmp * A;
	}

	/**
	 * Converted to integer vector with rounding down also for negative numbers.
	 */
	Vector<int, N> ToVecI() const {
		Vector<int, N> result;

		for (unsigned int n = 0; n < N; n++)
			if (values[n] >= 0)
				result[n] = values[n];
			else
				result[n] = fmodf(values[n], 1.0f) == 0.0f ? values[n]
														   : values[n] - 1.0f;

		return result;
	}

	/**
	 * Returns true if all components are 0
	 */
	bool IsZero() const {
		for (unsigned int n = 0; n < N; n++)
			if (values[n] != 0)
				return false;
		return true;
	}

	static std::vector<Vector<Scalar, N> > Orthonormalize(
		const std::vector<Vector<Scalar, N> >& vectors);

   protected:
	/** The N values of the template Scalar type are the only data members
	 of this class. This guarantees 100% compatibility with arrays of type
	 Scalar and size N, allowing us to define the cast operators to and from
	 arrays and array pointers */
	//#if defined(__INTEL_COMPILER)
	//  __declspec( align(16) ) Scalar values[N];
	//#else
	Scalar values[N];
	//#endif
};

template <typename Scalar, int N>
inline std::vector<Vector<Scalar, N> > Vector<Scalar, N>::Orthonormalize(
	const std::vector<Vector<Scalar, N> >& vectors) {
	std::vector<Vector<Scalar, N> > result;
	unsigned int vectorCount = N < vectors.size() ? N : vectors.size();
	for (unsigned int v = 0; v < vectorCount; v++) {
		// Error case
		if (vectors[v].IsZero())
			return result;

		// Base case
		if (v == 0) {
			Vector<Scalar, N> e0 = vectors[0];
			e0.normalize();
			result.push_back(e0);
			continue;
		}

		// Iterative case
		Vector<Scalar, N> e = vectors[v];
		for (unsigned int k = 0; k < v; k++) {
			Scalar inSetDirection = e.dot(result[k]);
			e = e - result[k] * inSetDirection;
		}
		e.normalize();
		result.push_back(e);
	}

	return result;
}

/** output a vector by printing its space-separated compontens */
template <typename Scalar, int N>
inline std::ostream& operator<<(std::ostream& os,
								const Vector<Scalar, N>& vec) {
	for (int i = 0; i < N - 1; i++)
		os << vec[i] << " ";
	os << vec[N - 1];

	return os;
}

/** assignment from a vector of the same kind */
template <>
inline Vector<float, 3>& Vector<float, 3>::operator=(
	const Vector<float, 3>& o) {
	values[0] = o.values[0];
	values[1] = o.values[1];
	values[2] = o.values[2];
	return *this;
}

/** scalar * vector */
template <typename Scalar, int N>
inline Vector<Scalar, N> operator*(Scalar s, const Vector<Scalar, N>& v) {
	return v * s;
}

/** read the space-separated components of a vector from a stream */
template <typename Scalar, int N>
inline std::istream& operator>>(std::istream& is, Vector<Scalar, N>& vec) {
	for (int i = 0; i < N; i++)
		is >> vec[i];
	return is;
}

// specialized versions for Vec3f follow

/** vector self-addition */
template <>
inline Vector<float, 3>& Vector<float, 3>::operator+=(
	const Vector<float, 3>& other) {
	values[0] += other.values[0];
	values[1] += other.values[1];
	values[2] += other.values[2];
	return *this;
}

/** vector addition */
template <>
inline Vector<float, 3> Vector<float, 3>::operator+(
	const Vector<float, 3>& other) const {
	Vector<float, 3> v(*this);
	v += other;
	return v;
}

/** vector difference from this */
template <>
inline Vector<float, 3>& Vector<float, 3>::operator-=(
	const Vector<float, 3>& o) {
	values[0] -= o.values[0];
	values[1] -= o.values[1];
	values[2] -= o.values[2];
	return *this;
}

/** vector difference */
template <>
inline Vector<float, 3> Vector<float, 3>::operator-(
	const Vector<float, 3>& o) const {
	Vector<float, 3> v(*this);
	v -= o;
	return v;
}

/** component-wise self-division */
template <>
inline const Vector<float, 3>& Vector<float, 3>::operator/=(
	const Vector<float, 3>& rhs) {
	values[0] /= rhs[0];
	values[1] /= rhs[1];
	values[2] /= rhs[2];
	return *this;
}

/** component-wise division by scalar */
template <>
inline Vector<float, 3> Vector<float, 3>::operator/(const float& s) const {
	Vector<float, 3> v(*this);
	return v /= s;
}

/** component-wise self-multiplication with scalar */
template <>
inline const Vector<float, 3>& Vector<float, 3>::operator*=(const float& s) {
	values[0] *= s;
	values[1] *= s;
	values[2] *= s;
	return *this;
}

/** component-wise multiplication with scalar */
template <>
inline Vector<float, 3> Vector<float, 3>::operator*(const float& s) const {
	Vector<float, 3> v(*this);
	return v *= s;
}

/** norm. only for some types */
template <>
inline float Vector<float, 3>::norm() const {
	return sqrtf(values[0] * values[0] + values[1] * values[1] +
				 values[2] * values[2]);
}

/** norm. only for some types */
template <>
inline double Vector<double, 3>::norm() const {
	return sqrt(values[0] * values[0] + values[1] * values[1] +
				values[2] * values[2]);
}

/** norm. only for some types */
template <>
inline long double Vector<long double, 3>::norm() const {
	return sqrt(values[0] * values[0] + values[1] * values[1] +
				values[2] * values[2]);
}

template <>
inline float Vector<float, 2>::norm() const {
#ifdef WIN32
	return sqrtf(values[0] * values[0] + values[1] * values[1]);
#else
	return sqrt(values[0] * values[0] + values[1] * values[1]);
#endif
}

/** sqrnorm for Vec3f */
template <>
inline double Vector<double, 2>::norm() const {
	return sqrt(values[0] * values[0] + values[1] * values[1]);
}

/** norm. only for some types */
template <>
inline float Vector<float, 128>::norm() const {
	return sqrtf(sqrnorm());
}

/** norm. only for some types */
template <>
inline double Vector<double, 128>::norm() const {
	return sqrt(sqrnorm());
}

/** sqrnorm. only for some types */
template <>
inline float Vector<float, 3>::sqrnorm() const {
	return values[0] * values[0] + values[1] * values[1] +
		   values[2] * values[2];
}

/** sqrnorm for Vec3d */
template <>
inline double Vector<double, 3>::sqrnorm() const {
	return values[0] * values[0] + values[1] * values[1] +
		   values[2] * values[2];
}

/** cross product for Vec3f */
template <>
inline Vector<float, 3> Vector<float, 3>::operator%(
	const Vector<float, 3>& other) const {
	return Vector<float, 3>(
		values[1] * other.values[2] - values[2] * other.values[1],
		values[2] * other.values[0] - values[0] * other.values[2],
		values[0] * other.values[1] - values[1] * other.values[0]);
}

/** cross product for Vec3d */
template <>
inline Vector<double, 3> Vector<double, 3>::operator%(
	const Vector<double, 3>& other) const {
	return Vector<double, 3>(
		values[1] * other.values[2] - values[2] * other.values[1],
		values[2] * other.values[0] - values[0] * other.values[2],
		values[0] * other.values[1] - values[1] * other.values[0]);
}

template <>
inline int Vector<unsigned char, 4>::hamming(
	const Vector<unsigned char, 4>& other) const {
	unsigned int difference =
		*reinterpret_cast<const unsigned int*>(&values[0]) ^
		*reinterpret_cast<const unsigned int*>(&other.values[0]);
	int result = __builtin_popcount(difference);

	return result;
}

template <>
inline int Vector<unsigned char, 32>::hamming(
	const Vector<unsigned char, 32>& other) const {
	int result;

	{
		const uint64_t* op1 = reinterpret_cast<const uint64_t*>(&values[0]);
		const uint64_t* op2 =
			reinterpret_cast<const uint64_t*>(&other.values[0]);
		result = __builtin_popcountll(op1[0] ^ op2[0]);
		result += __builtin_popcountll(op1[1] ^ op2[1]);
		result += __builtin_popcountll(op1[2] ^ op2[2]);
		result += __builtin_popcountll(op1[3] ^ op2[3]);
	}

	return result;
}

/** central projection for Vec4f */
template <>
inline Vector<float, 3> Vector<float, 4>::centralProjection() const {
	return (fabsf(values[3]) > 1e-5)
			   ? Vector<float, 3>(values[0] / values[3], values[1] / values[3],
								  values[2] / values[3])
			   : Vector<float, 3>(0., 0., 0.);
}

/** central projection for Vec4d */
template <>
inline Vector<double, 3> Vector<double, 4>::centralProjection() const {
	return (fabs(values[3]) > 1e-5)
			   ? Vector<double, 3>(values[0] / values[3], values[1] / values[3],
								   values[2] / values[3])
			   : Vector<double, 3>(0., 0., 0.);
}

/// compute scalar product with another vector of same type
template <>
inline float Vector<float, 3>::operator|(const Vector<float, 3>& rhs) const {
	return values[0] * rhs[0] + values[1] * rhs[1] + values[2] * rhs[2];
}

/** symmetric version of the dot product */
template <typename Scalar, int N>
inline Scalar dot(const Vector<Scalar, N>& v1, const Vector<Scalar, N>& v2) {
	return v1.dot(v2);
}

/** symmetric version of the cross product */
template <typename Scalar, int N>
inline Vector<Scalar, N> cross(const Vector<Scalar, N>& v1,
							   const Vector<Scalar, N>& v2) {
	return v1.cross(v2);
}

/** add direction (N-1) to a N vector. ret = p + d*t. Only valid for
 homogenous points (4D) and 3D direction vector */
template <typename Scalar>
inline Vector<Scalar, 4> add_direction(const Vector<Scalar, 4>& p,
									   const Vector<Scalar, 3>& d,
									   Scalar t = 1.0) {
	Vector<Scalar, 4> p_tmp(p);
	p_tmp.centralProjection();
	p_tmp[0] += d[0] * t;
	p_tmp[1] += d[1] * t;
	p_tmp[2] += d[2] * t;
	return p_tmp;
}

/** 1-byte signed vector */
typedef Vector<signed char, 1> Vec1c;
/** 1-byte unsigned vector */
typedef Vector<unsigned char, 1> Vec1uc;
/** 1-short signed vector */
typedef Vector<signed short int, 1> Vec1s;
/** 1-short unsigned vector */
typedef Vector<unsigned short int, 1> Vec1us;
/** 1-int signed vector */
typedef Vector<signed int, 1> Vec1i;
/** 1-int unsigned vector */
typedef Vector<unsigned int, 1> Vec1ui;
/** 1-float vector */
typedef Vector<float, 1> Vec1f;
/** 1-double vector */
typedef Vector<double, 1> Vec1d;

/** 2-byte signed vector */
typedef Vector<signed char, 2> Vec2c;
/** 2-byte unsigned vector */
typedef Vector<unsigned char, 2> Vec2uc;
/** 2-short signed vector */
typedef Vector<signed short int, 2> Vec2s;
/** 2-short unsigned vector */
typedef Vector<unsigned short int, 2> Vec2us;
/** 2-int signed vector */
typedef Vector<signed int, 2> Vec2i;
/** 2-int unsigned vector */
typedef Vector<unsigned int, 2> Vec2ui;
/** 2-float vector */
typedef Vector<float, 2> Vec2f;
/** 2-double vector */
typedef Vector<double, 2> Vec2d;

/** 3-byte signed vector */
typedef Vector<signed char, 3> Vec3c;
/** 3-byte unsigned vector */
typedef Vector<unsigned char, 3> Vec3uc;
/** 3-short signed vector */
typedef Vector<signed short int, 3> Vec3s;
/** 3-short unsigned vector */
typedef Vector<unsigned short int, 3> Vec3us;
/** 3-int signed vector */
typedef Vector<signed int, 3> Vec3i;
/** 3-int unsigned vector */
typedef Vector<unsigned int, 3> Vec3ui;
/** 3-float vector */
typedef Vector<float, 3> Vec3f;
/** 3-double vector */
typedef Vector<double, 3> Vec3d;

/** 4-byte signed vector */
typedef Vector<signed char, 4> Vec4c;
/** 4-byte unsigned vector */
typedef Vector<unsigned char, 4> Vec4uc;
/** 4-short signed vector */
typedef Vector<signed short int, 4> Vec4s;
/** 4-short unsigned vector */
typedef Vector<unsigned short int, 4> Vec4us;
/** 4-int signed vector */
typedef Vector<signed int, 4> Vec4i;
/** 4-int unsigned vector */
typedef Vector<unsigned int, 4> Vec4ui;
/** 4-float vector */
typedef Vector<float, 4> Vec4f;
/** 4-double vector */
typedef Vector<double, 4> Vec4d;

/** 6-byte signed vector */
typedef Vector<signed char, 6> Vec6c;
/** 6-byte unsigned vector */
typedef Vector<unsigned char, 6> Vec6uc;
/** 6-short signed vector */
typedef Vector<signed short int, 6> Vec6s;
/** 6-short unsigned vector */
typedef Vector<unsigned short int, 6> Vec6us;
/** 6-int signed vector */
typedef Vector<signed int, 6> Vec6i;
/** 6-int unsigned vector */
typedef Vector<unsigned int, 6> Vec6ui;
/** 6-float vector */
typedef Vector<float, 6> Vec6f;
/** 6-double vector */
typedef Vector<double, 6> Vec6d;

/** 8-float vector */
typedef Vector<float, 8> Vec8f;

} /* end of namespace base */

typedef base::Vector<unsigned char, 32> DescriptorType;

#endif /* VECTOR_HH */
