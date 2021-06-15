#ifndef _MATRIX2X2_H_
#define _MATRIX2X2_H_

#include "Vector.hh"

#undef MAT
#define MAT(m, r, c) ((m)[((r)*2) + (c)])
#define M(r, c) ((&_00)[((r)*2) + (c)])

#define MATRIX_OPERATOR(src, dst, op) \
	{                                 \
		(dst)._00 op(src)._00;        \
		(dst)._01 op(src)._01;        \
		(dst)._10 op(src)._10;        \
		(dst)._11 op(src)._11;        \
	}

namespace base {

/**
 * Fixed size 2x2 matrix implementation.
 */
template <typename Scalar>
struct Matrix2x2 {
	/// Data type for the scalars of the matrix
	typedef Scalar value_type;

	/// Returns number of rows.
	int Rows() const { return 2; }

	/// Returns number of columns.
	int Cols() const { return 2; }

	/// Default constructor creates uninitialized values.
	Matrix2x2() {}

	Vector<Scalar, 2> operator*(const Vector<Scalar, 2>& in) {
		Vector<Scalar, 2> result;
		result[0] = _00 * in[0] + _01 * in[1];
		result[1] = _10 * in[0] + _11 * in[1];
		return result;
	}

	Matrix2x2 Inverse() const {
		float invDet = 1.0f / (_00 * _11 - _01 * _10);
		Matrix2x2 result;
		result._00 = _11 * invDet;
		result._01 = -_01 * invDet;
		result._10 = -_10 * invDet;
		result._11 = _00 * invDet;

		return result;
	}

	/// Matrix element of row 0, column 0.
	Scalar _00;
	/// Matrix element of row 0, column 1.
	Scalar _01;
	/// Matrix element of row 1, column 0.
	Scalar _10;
	/// Matrix element of row 1, column 1.
	Scalar _11;
};

/** ======================================================== */
/** typedefs                                                 */
/** ======================================================== */

/// Float 3x3 matrix.
typedef Matrix2x2<float> Matrix2x2f;
/// Double 3x3 matrix.
typedef Matrix2x2<double> Matrix2x2d;

}  // namespace base
#undef MATRIX_OPERATOR
#undef MAT
#undef M
#endif /* _MATRIX2X2_H_ */
