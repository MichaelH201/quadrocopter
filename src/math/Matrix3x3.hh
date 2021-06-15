#ifndef _MATRIX3X3_H_
#define _MATRIX3X3_H_

#include "MSVC_Fixes.h"

#include "Vector.hh"

#include "TypelessMath.hh"

#include <math.h>
#include <stdio.h>

#undef MAT
#define MAT(m, r, c) ((m)[((r)*3) + (c)])
#define M(r, c) ((&_00)[((r)*3) + (c)])

#define MATRIX_OPERATOR(src, dst, op) \
	{                                 \
		(dst)._00 op(src)._00;        \
		(dst)._01 op(src)._01;        \
		(dst)._02 op(src)._02;        \
		(dst)._10 op(src)._10;        \
		(dst)._11 op(src)._11;        \
		(dst)._12 op(src)._12;        \
		(dst)._20 op(src)._20;        \
		(dst)._21 op(src)._21;        \
		(dst)._22 op(src)._22;        \
	}

namespace base {

/**
 * Fixed size 3x3 matrix implementation.
 * @remarks Matrix was inplemented for a right hand coordinate system in which
 * x, y and z correspond to right, in and up. All rotations are CCW. This class
 * is intended to be used for multiplying its matrices from to left to
 * column vectors.
 */
template <typename Scalar>
struct Matrix3x3 {
	/// Data tyoe fir the scalars of the matrix
	typedef Scalar value_type;

	/// Returns number of rows.
	int Rows() const { return 3; }

	/// Returns number of columns.
	int Cols() const { return 3; }

	/// Default constructor creates uninitialized values.
	Matrix3x3() {}

	/// Copy constructor (copy all elements).
	inline Matrix3x3(const Matrix3x3<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, =);
	}

	/// Copy constructor (copy all elements).
	template <class otherScalar>
	inline Matrix3x3(const Matrix3x3<otherScalar>& inst) {
		MATRIX_OPERATOR(inst, *this, =);
	}

	/**
	 * Sets up the matrix by using an array of N*N scalar values.
	 * @param array Contains the scalar values to be used as initialization
	 * in 'row first' order.
	 */
	inline Matrix3x3(const Scalar array[]) {
		Scalar* dst = &_00;
		for (int i = 0; i < 3 * 3; i++)
			*dst++ = *array++;
	}

	/**
	 * Sets up the matrix with the data given in three vectors.
	 * @param x Contains the first row of the matrix.
	 * @param y Contains the second row of the matrix.
	 * @param z Contains the third row of the matrix.
	 */
	inline Matrix3x3(const Vector<Scalar, 3>& x,
					 const Vector<Scalar, 3>& y,
					 const Vector<Scalar, 3>& z);

	/**
	 * Assignment operator for float arrays.
	 * @param array Contains the scalar values to be used for the matrix
	 * in 'row first' order.
	 */
	inline const Matrix3x3& operator=(const Scalar array[]) {
		Scalar* dst = &_00;
		for (int i = 0; i < 3 * 3; i++)
			*dst++ = *array++;
		return *this;
	}

	/**
	 * Assignment operator
	 * @param inst Matrix containing the values to be assigned.
	 */
	inline const Matrix3x3& operator=(const Matrix3x3<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, =);
		return *this;
	}

	/**
	 * Comonentwise adding.
	 * @param inst Other matrix to add componentwise.
	 * @returns Self reference.
	 */
	inline const Matrix3x3& operator+=(const Matrix3x3<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, +=);
		return *this;
	}

	/**
	 * Comonentwise subtraction.
	 * @param inst Other matrix to subtract componentwise.
	 * @returns Self reference.
	 */
	inline const Matrix3x3& operator-=(const Matrix3x3<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, -=);
		return *this;
	}

	/**
	 * Compares two matrices for equality.
	 * @param inst Other matrix to be compared.
	 * @returns True if this has nearly the same content as inst.
	 */
	inline bool operator==(const Matrix3x3<Scalar>& inst) const {
		int i;
		const Scalar* a = &_00;
		const Scalar* b = &inst._00;
		for (i = 0; i < 9; i++, a++, b++)
			if (!checkEpsilon(*a - *b))
				return false;
		return true;
	}

	/**
	 * Compares two matrices for equality.
	 * param inst Other matrix to be compared.
	 * @returns True if the matrices differ (more than epsilon).
	 */
	inline bool operator!=(const Matrix3x3<Scalar>& inst) const {
		return !(operator==(inst));
	}

	/**
	 * Matrix by vector multiplication - last component is untouched.
	 * @param v Vector to be transformed.
	 * @returns Result of the multiplication. 4th component corresponds to v[4].
	 */
	inline Vector<Scalar, 4> TransformVector(const Vector<Scalar, 4>& v) const;

	/**
	 * Matrix by vector multiplication.
	 * @param v Vector to be multiplied with the matrix.
	 * @returns Result of the multiplication.
	 */
	inline Vector<Scalar, 3> operator*(const Vector<Scalar, 3>& v) const;

	/**
	 * Matrix by vector multiplication.
	 * @param s Scalar to be multiplied with the matrix.
	 * @returns Result of the multiplication.
	 */
	inline Matrix3x3<Scalar> operator*(const Scalar& s) const;

	/**
	 * Transforms a point itself
	 * @param v Source and destenation vector for transformation.
	 */
	inline void TransformPointSelf(Vector<Scalar, 3>& v) const;

	/**
	 * Transforms a point
	 * @param v Source vector to transform
	 * @returns transformed vector.
	 */
	inline Vector<Scalar, 3> TransformPoint(const Vector<Scalar, 3>& v) const;

	/*
	 * Transforms a homogeneous point
	 * @param v Source vector, treated as (x, z, 1)
	 * @returns Transformed vector. x and y of renormalized homogeneous point
	 * are returned.
	 */
	inline Vector<Scalar, 2> TransformPoint(const Vector<Scalar, 2>& v) const;

	/**
	 * Transforms a point itself
	 * @param v Source and destenation vector for transformation.
	 */
	inline void TransformPointSelf(Scalar* v) const;

	/**
	 * Transforms a point itself
	 * @param v Homogeneous vector representing the source and
	 * destenation for transformation.
	 */
	inline void TransformVectorSelf(Vector<Scalar, 4>& v) const;

	/**
	 * Transforms a point itself
	 * @param v Homogeneous vector representing the source and
	 * destenation for transformation.
	 */
	inline void TransformVectorSelf(Scalar* v) const;

	/**
	 * Comonentwise adding.
	 * @param inst Other matrix for adding.
	 * @returns Result of the operation.
	 */
	inline Matrix3x3 operator+(const Matrix3x3<Scalar>& inst) const {
		Matrix3x3<Scalar> m(inst);
		MATRIX_OPERATOR(*this, m, +=);
		return m;
	}

	/**
	 * Comonentwise subtraction.
	 * @param inst Subtrahend for subtraction.
	 * @returns Result of the operation.
	 */
	inline Matrix3x3 operator-(const Matrix3x3<Scalar>& inst) const {
		Matrix3x3<Scalar> m(*this);
		MATRIX_OPERATOR(inst, m, -=);
		return m;
	}

	/**
	 * Multiplies the matrix with another matrix.
	 * @param inst Other matrix
	 * @returns Self reference.
	 */
	const Matrix3x3& operator*=(const Matrix3x3<Scalar>& inst);
	/**
	 * Multiplies the matrix with another matrix.
	 * @param inst Other matrix stored as rowwise array of scalars
	 * @returns Self reference.
	 */
	const Matrix3x3& operator*=(const Scalar inst[]);
	/**
	 * Multiplies the matrix with a scalar.
	 * @param inst Scalar value for multiplication
	 * @returns Self reference.
	 */
	const Matrix3x3& operator*=(const Scalar& inst);

	/**
	 * Multiplies the matrix with the transpose matrix of another matrix.
	 * @param inst Other matrix stored as columnwise array of scalars
	 */
	void MultTransposed(const Scalar inst[]);

	/**
	 * Matrix multiplication.
	 * @param inst Second matrix for multiplication.
	 * @returns Result of the operation.
	 */
	Matrix3x3 operator*(const Matrix3x3<Scalar>& inst) const;

	/**
	 * Matrix multiplication from the left. self = inst * self.
	 * @param inst First matrix for multiplication.
	 */
	void PreMult(const Matrix3x3<Scalar>& inst);
	/**
	 * Matrix multiplication from the left. self = inst * self.
	 * @param inst Rowwise scalar values of the first matrix for multiplication.
	 */
	void PreMult(const Scalar inst[]);

	/**
	 * Multiplies the matrix with the transpose matrix of another matrix
	 * from the left: self = inst^T * self.
	 * @param inst Other matrix stored as columnwise array of scalars
	 */
	void PreMultTransposed(const Scalar inst[]);

	/** Access operator for reading and writing the scalar values
	 * of the matrix.
	 * @param row The selected row.
	 * @param col The selected column.
	 */
	inline Scalar& operator()(unsigned int row, unsigned int col) {
		return M(row, col);
	}

	/** Access operator for reading the scalar values
	 * of the matrix.
	 * @param row The selected row.
	 * @param col The selected column.
	 */
	inline const Scalar& operator()(unsigned int row, unsigned int col) const {
		return M(row, col);
	}

	/**
	 * Inverts the matrix.
	 * @returns True if successful (full rank).
	 */
	bool Invert();

	/**
	 *  Sets all elements to zero.
	 */
	inline void Clear();
	/**
	 * Sets up an identity matrix.
	 */
	inline void Identity();
	/**
	 * Transposes the matrix.
	 */
	inline void Transpose();
	/**
	 * Returns the transposed matrix.
	 */
	inline Matrix3x3 Transposed();

	/**
	 * Performs a self-multiplication with a translation matrix.
	 * @param x Translation in x direction.
	 * @param y Translation in y direction.
	 */
	inline void Translate2D(Scalar x, Scalar y);
	/**
	 * Performs a self-multiplication with a translation matrix.
	 * @param v Translation.
	 */
	inline void Translate2D(const Vector<Scalar, 2>& v) {
		Translate2D(v[0], v[1]);
	}

	/**
	 * Performs a self-multiplication with a scaling matrix.
	 * @param x Scaling in x direction.
	 * @param y Scaling in y direction.
	 * @param z Scaling in z direction.
	 */
	inline void Scale(Scalar x, Scalar y, Scalar z);
	/**
	 * Performs a self-multiplication with a scaling matrix.
	 * @param v Non-uniform scaling vector.
	 */
	inline void Scale(const Vector<Scalar, 3>& v) { Scale(v[0], v[1], v[2]); }

	/**
	 * Performs a self-multiplication with a 2D scaling matrix.
	 * @param x Scaling in x direction.
	 * @param y Scaling in y direction.
	 */
	inline void Scale2D(Scalar x, Scalar y);
	/**
	 * Performs a self-multiplication with a 2D scaling matrix.
	 * @param v Non-uniform scaling vector.
	 */
	inline void Scale2D(const Vector<Scalar, 2>& v) { Scale2D(v[0], v[1]); }

	/**
	 * Performs a self-multiplication with a rotation matrix for
	 * rotation around an arbitrary axis.
	 * @param angle Angle in degrees.
	 * @param x x Component of the rotation axis.
	 * @param y y Component of the rotation axis.
	 * @param z z Component of the rotation axis.
	 */
	void Rotate(Scalar angle, Scalar x, Scalar y, Scalar z);
	/**
	 * Performs a self-multiplication with a rotation matrix for
	 * rotation around an arbitrary axis.
	 * @param angle Angle in degrees.
	 * @param v Rotation axis.
	 */
	inline void Rotate(Scalar angle, const Vector<Scalar, 3>& v) {
		Rotate(angle, v[0], v[1], v[2]);
	}

	/**
	 * Performs a self-multiplication with a rotation matrix for
	 * rotation around the x axis.
	 * @param angle Angle in degrees.
	 */
	inline void RotateX(Scalar angle);
	/**
	 * Performs a self-multiplication with a rotation matrix for
	 * rotation around the y axis.
	 * @param angle Angle in degrees.
	 */
	inline void RotateY(Scalar angle);
	/**
	 * Performs a self-multiplication with a rotation matrix for
	 * rotation around the z axis.
	 * @param angle Angle in degrees.
	 */
	inline void RotateZ(Scalar angle);

	/**
	 * Performs a self-multiplication with a 2D rotation matrix for
	 * rotation around the x axis.
	 * @remarks CCW if x is right and y is up.
	 * @param angle Angle in degrees.
	 */
	inline void Rotate2D(Scalar angle) { RotateZ(-angle); }

	/**
	 * Perform a self multiplication with a rotation matrix that rotates
	 * v1 into v2.
	 * @param v1 First vector to determine the rotation matrix.
	 * @param v2 Second vector to determine the rotation matrix.
	 */
	void Rotate(const Vector<Scalar, 3>& v1, const Vector<Scalar, 3>& v2) {
		Scalar n1 = v1.norm();
		Scalar n2 = v2.norm();
		Scalar a = (checkEpsilon(n1) || checkEpsilon(n2))
					   ? 0.0
					   : acosT((v1 | v2) / (n1 * n2));
		a *= (180.0 / M_PI);
		Vector<Scalar, 3> vtmp(v1 % v2);
		vtmp *= -1;
		if (!checkEpsilon(vtmp.norm()))
			Rotate(a, vtmp);
	}

	/**
	 * Sets up a transformation that aligns objects according to
	 * a given base.
	 * @remarks The given base vectors are orthonormalized before
	 * the transformation is established.
	 * @param x First base vector.
	 * @param y Second base vector.
	 * @param z Third base vector.
	 * @param inverse If true, the transformation inverse to
	 * the one described above is created.
	 */
	void SetupReferenceFrame(const Vector<Scalar, 3>& x,
							 const Vector<Scalar, 3>& y,
							 const Vector<Scalar, 3>& z,
							 bool inverse);

	/**
	 * Returns the trace of the matrix.
	 * @returns trace of the matrix.
	 */
	inline Scalar Trace() const;

	/**
	 * Returns the determinant of the matrix.
	 * @returns determinant of the matrix.
	 */
	inline Scalar Determinant() const;

	/**
	 * Writes a column vector to memory.
	 * @param col Selected column.
	 * @param data Output pointer.
	 */
	inline void GetCol(int col, Scalar* data) const;
	/**
	 * Writes a row vector to memory.
	 * @param row Selected row.
	 * @param data Output pointer.
	 */
	inline void GetRow(int row, Scalar* data) const;
	/**
	 * Provides reading access to the raw, row first ordered data of the matrix.
	 * @remarks This can be used to pass matrix contents to OpenGL. Transposed
	 * OpenGL functions have to be used!
	 * @returns Pointer to the beginning of the matrix's raw data.
	 */
	inline const Scalar* GetRawData() const { return &_00; }
	/**
	 * Provides read/writeaccess to the raw, row first ordered data of the
	 * matrix.
	 * @remarks This can be used to pass matrix contents to OpenGL. Transposed
	 * OpenGL functions have to be used!
	 * @returns Pointer to the beginning of the matrix's raw data.
	 */
	inline Scalar* GetRawData() { return &_00; }

	/**
	 * Finds a rotation axis and angle for arbitrary rotation matrices.
	 * @remarks Don't try to do this on non-rotation matrices!
	 * @param angle Output: The rotation angle in degrees.
	 * @param axis Output: The rotation axis.
	 * @returns True if successfull.
	 */
	bool FindRotation(Scalar& angle, Vector<Scalar, 3>& axis) const;

	/**
	 * Dumps content in a 'readable' form to stdout.
	 */
	void Print(const char* title = NULL) const;

	/**
	 * Performs a self multiplication with a float 3x3 matrix,
	 * no matter what the Scalar type of the matrix object is.
	 * @param m Float 3x3 matrix for multiplication.
	 */
	void MultMatrixFloat(const base::Matrix3x3<float>& m) {
		Scalar tmpmat[9];
		const float* mraw = m.GetRawData();
		for (int i = 0; i < 9; i++)
			tmpmat[i] = Scalar(mraw[i]);
		multMatrix(tmpmat);
	}
	/**
	 * Performs a self multiplication with a double 3x3 matrix,
	 * no matter what the Scalar type of the matrix object is.
	 * @param m Float 3x3 matrix for multiplication.
	 */
	void MultMatrixDouble(const base::Matrix3x3<double>& m) {
		Scalar tmpmat[9];
		const double* mraw = m.GetRawData();
		for (int i = 0; i < 9; i++)
			tmpmat[i] = Scalar(mraw[i]);
		multMatrix(tmpmat);
	}

	/// Matrix element of row 0, column 0.
	Scalar _00;
	/// Matrix element of row 0, column 1.
	Scalar _01;
	/// Matrix element of row 0, column 2.
	Scalar _02;
	/// Matrix element of row 1, column 0.
	Scalar _10;
	/// Matrix element of row 1, column 1.
	Scalar _11;
	/// Matrix element of row 1, column 2.
	Scalar _12;
	/// Matrix element of row 2, column 0.
	Scalar _20;
	/// Matrix element of row 2, column 1.
	Scalar _21;
	/// Matrix element of row 2, column 2.
	Scalar _22;

   protected:
	/**
	 * Performs a self-multiplication with a matrix given as 9
	 * consecutive values in memory.
	 * @param m Matrix elements, row ordered.
	 */
	void multMatrix(const Scalar* m);
	/**
	 * Performs a self-multiplication with the multiplyer and
	 * destenation  given as 9 consecutive values in memory.
	 * @param dst Place to store the result, row ordered.
	 * @param m Matrix elements, row ordered.
	 */
	void multMatrix(Scalar* dst, const Scalar* m);
};

template <typename Scalar>
Matrix3x3<Scalar>::Matrix3x3(const Vector<Scalar, 3>& x,
							 const Vector<Scalar, 3>& y,
							 const Vector<Scalar, 3>& z) {
	_00 = x[0], _01 = y[0], _02 = z[0];
	_10 = x[1], _11 = y[1], _12 = z[1];
	_20 = x[2], _21 = y[2], _22 = z[2];
}

template <typename Scalar>
void Matrix3x3<Scalar>::Clear() {
	Scalar* m = &_00;
	for (int i = 0; i < 3 * 3; i++)
		*m++ = 0.;
}

template <typename Scalar>
void Matrix3x3<Scalar>::Identity() {
	_00 = 1., _01 = 0., _02 = 0.;
	_10 = 0., _11 = 1., _12 = 0.;
	_20 = 0., _21 = 0., _22 = 1.;
}

template <typename Scalar>
void Matrix3x3<Scalar>::Transpose() {
	Scalar tmp;
	for (int i = 0; i < 3; i++) {
		for (int j = i + 1; j < 3; j++) {
			tmp = M(i, j);
			M(i, j) = M(j, i);
			M(j, i) = tmp;
		}
	}
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::Transposed() {
	Matrix3x3<Scalar> result(*this);
	result.Transpose();
	return result;
}

template <typename Scalar>
void Matrix3x3<Scalar>::GetCol(int col, Scalar* data) const {
	data[0] = M(0, col);
	data[1] = M(1, col);
	data[2] = M(2, col);
}

template <typename Scalar>
void Matrix3x3<Scalar>::GetRow(int row, Scalar* data) const {
	data[0] = M(row, 0);
	data[1] = M(row, 1);
	data[2] = M(row, 2);
}

template <typename Scalar>
Vector<Scalar, 4> Matrix3x3<Scalar>::TransformVector(
	const Vector<Scalar, 4>& v) const {
	return Vector<Scalar, 4>(_00 * v[0] + _01 * v[1] + _02 * v[2],
							 _10 * v[0] + _11 * v[1] + _12 * v[2],
							 _20 * v[0] + _21 * v[1] + _22 * v[2], v[3]);
}

template <typename Scalar>
Vector<Scalar, 3> Matrix3x3<Scalar>::operator*(
	const Vector<Scalar, 3>& v) const {
	return Vector<Scalar, 3>(_00 * v[0] + _01 * v[1] + _02 * v[2],
							 _10 * v[0] + _11 * v[1] + _12 * v[2],
							 _20 * v[0] + _21 * v[1] + _22 * v[2]);
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator*(const Scalar& s) const {
	Matrix3x3<Scalar> result(*this);
	result *= s;

	return result;
}

template <typename Scalar>
Vector<Scalar, 3> Matrix3x3<Scalar>::TransformPoint(
	const Vector<Scalar, 3>& v) const {
	Vector<Scalar, 3> result;
	result[0] = _00 * v[0] + _01 * v[1] + _02 * v[2];
	result[1] = _10 * v[0] + _11 * v[1] + _12 * v[2];
	result[2] = _20 * v[0] + _21 * v[1] + _22 * v[2];

	return result;
}

template <typename Scalar>
Vector<Scalar, 2> Matrix3x3<Scalar>::TransformPoint(
	const Vector<Scalar, 2>& v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02;
	Scalar y = _10 * v[0] + _11 * v[1] + _12;
	Scalar z = _20 * v[0] + _21 * v[1] + _22;
	if (fabsT(z) > 1e-5) {
		Scalar zinv = Scalar(1.0) / z;
		return Vector<Scalar, 2>(x * zinv, y * zinv);
	} else {
		return Vector<Scalar, 2>(0, 0);
	}
}

template <typename Scalar>
void Matrix3x3<Scalar>::TransformPointSelf(Vector<Scalar, 3>& v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2];
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2];
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2];

	v[0] = x;
	v[1] = y;
	v[2] = z;
}

/** transform point (x',y',z',1) = A * (x,y,z,1) */
template <typename Scalar>
void Matrix3x3<Scalar>::TransformPointSelf(Scalar* v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2];
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2];
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2];

	v[0] = x;
	v[1] = y;
	v[2] = z;
}

template <typename Scalar>
void Matrix3x3<Scalar>::TransformVectorSelf(Vector<Scalar, 4>& v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2];
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2];
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2];

	v[0] = x;
	v[1] = y;
	v[2] = z;
}

template <typename Scalar>
void Matrix3x3<Scalar>::TransformVectorSelf(Scalar* v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2];
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2];
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2];

	v[0] = x;
	v[1] = y;
	v[2] = z;
}

template <typename Scalar>
const Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator*=(
	const Matrix3x3<Scalar>& inst) {
#define A(row, col) (&_00)[(row * 3) + col]
#define B(row, col) (&inst._00)[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar ai0 = A(i, 0), ai1 = A(i, 1), ai2 = A(i, 2);
		A(i, 0) = ai0 * B(0, 0) + ai1 * B(1, 0) + ai2 * B(2, 0);
		A(i, 1) = ai0 * B(0, 1) + ai1 * B(1, 1) + ai2 * B(2, 1);
		A(i, 2) = ai0 * B(0, 2) + ai1 * B(1, 2) + ai2 * B(2, 2);
	}
#undef A
#undef B
	return *this;
}

template <typename Scalar>
const Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator*=(const Scalar inst[]) {
#define A(row, col) (&_00)[(row * 3) + col]
#define B(row, col) inst[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar ai0 = A(i, 0), ai1 = A(i, 1), ai2 = A(i, 2);
		A(i, 0) = ai0 * B(0, 0) + ai1 * B(1, 0) + ai2 * B(2, 0);
		A(i, 1) = ai0 * B(0, 1) + ai1 * B(1, 1) + ai2 * B(2, 1);
		A(i, 2) = ai0 * B(0, 2) + ai1 * B(1, 2) + ai2 * B(2, 2);
	}
#undef A
#undef B
	return *this;
}

template <typename Scalar>
const Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator*=(const Scalar& inst) {
	for (int i = 0; i < 9; i++)
		(&_00)[i] *= inst;

	return *this;
}

template <typename Scalar>
void Matrix3x3<Scalar>::MultTransposed(const Scalar inst[]) {
#define A(row, col) (&_00)[(row * 3) + col]
#define B(row, col) inst[(col * 3) + row]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar ai0 = A(i, 0), ai1 = A(i, 1), ai2 = A(i, 2);
		A(i, 0) = ai0 * B(0, 0) + ai1 * B(1, 0) + ai2 * B(2, 0);
		A(i, 1) = ai0 * B(0, 1) + ai1 * B(1, 1) + ai2 * B(2, 1);
		A(i, 2) = ai0 * B(0, 2) + ai1 * B(1, 2) + ai2 * B(2, 2);
	}
#undef A
#undef B
	return;
}

template <typename Scalar>
void Matrix3x3<Scalar>::PreMult(const Matrix3x3<Scalar>& inst) {
#define A(row, col) &(inst._00)[(row * 3) + col]
#define B(row, col) (&_00)[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar b0i = B(0, i), b1i = B(1, i), b2i = B(2, i);
		B(0, i) = A(0, 0) * b0i + A(0, 1) * b1i + A(0, 2) * b2i;
		B(1, i) = A(1, 0) * b0i + A(1, 1) * b1i + A(1, 2) * b2i;
		B(2, i) = A(2, 0) * b0i + A(2, 1) * b1i + A(2, 2) * b2i;
	}
#undef A
#undef B
	return;
}

template <typename Scalar>
void Matrix3x3<Scalar>::PreMult(const Scalar inst[]) {
#define A(row, col) inst[(row * 3) + col]
#define B(row, col) (&_00)[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar b0i = B(0, i), b1i = B(1, i), b2i = B(2, i);
		B(0, i) = A(0, 0) * b0i + A(0, 1) * b1i + A(0, 2) * b2i;
		B(1, i) = A(1, 0) * b0i + A(1, 1) * b1i + A(1, 2) * b2i;
		B(2, i) = A(2, 0) * b0i + A(2, 1) * b1i + A(2, 2) * b2i;
	}
#undef A
#undef B
	return;
}

template <typename Scalar>
void Matrix3x3<Scalar>::PreMultTransposed(const Scalar inst[]) {
#define A(row, col) inst[(col * 3) + row]
#define B(row, col) (&_00)[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar b0i = B(0, i), b1i = B(1, i), b2i = B(2, i);
		B(0, i) = A(0, 0) * b0i + A(0, 1) * b1i + A(0, 2) * b2i;
		B(1, i) = A(1, 0) * b0i + A(1, 1) * b1i + A(1, 2) * b2i;
		B(2, i) = A(2, 0) * b0i + A(2, 1) * b1i + A(2, 2) * b2i;
	}
#undef A
#undef B
	return;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator*(
	const Matrix3x3<Scalar>& inst) const {
	Matrix3x3<Scalar> tmp;

	/* code snippet taken from MesaLib 3.1 */
#define P(row, col) (&tmp._00)[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar ai0 = M(i, 0), ai1 = M(i, 1), ai2 = M(i, 2);
		P(i, 0) = ai0 * inst._00 + ai1 * inst._10 + ai2 * inst._20;
		P(i, 1) = ai0 * inst._01 + ai1 * inst._11 + ai2 * inst._21;
		P(i, 2) = ai0 * inst._02 + ai1 * inst._12 + ai2 * inst._22;
	}
#undef P
	return tmp;
}

template <typename Scalar>
bool Matrix3x3<Scalar>::Invert() {
	Scalar det = Determinant();

	if (det == 0)  // Rank < 3 -> no inverse
		return false;

	Matrix3x3<Scalar> inverse;

	inverse._00 = _11 * _22 - _12 * _21;
	inverse._01 = _02 * _21 - _01 * _22;
	inverse._02 = _01 * _12 - _02 * _11;

	inverse._10 = _12 * _20 - _10 * _22;
	inverse._11 = _00 * _22 - _02 * _20;
	inverse._12 = _02 * _10 - _00 * _12;

	inverse._20 = _10 * _21 - _11 * _20;
	inverse._21 = _01 * _20 - _00 * _21;
	inverse._22 = _00 * _11 - _01 * _10;

	*this = inverse * ((Scalar)1.0f / det);

	return true;
}

template <typename Scalar>
void Matrix3x3<Scalar>::Scale(Scalar x, Scalar y, Scalar z) {
	if (x != 1.0) {
		_00 *= x;
		_10 *= x;
		_20 *= x;
	}
	if (y != 1.0) {
		_01 *= y;
		_11 *= y;
		_21 *= y;
	}
	if (z != 1.0) {
		_02 *= z;
		_12 *= z;
		_22 *= z;
	}
}

template <typename Scalar>
void Matrix3x3<Scalar>::Scale2D(Scalar x, Scalar y) {
	if (x != 1.0) {
		_00 *= x;
		_10 *= x;
		_20 *= x;
	}
	if (y != 1.0) {
		_01 *= y;
		_11 *= y;
		_21 *= y;
	}
}

template <typename Scalar>
inline void Matrix3x3<Scalar>::Translate2D(Scalar x, Scalar y) {
	_02 = _00 * x + _01 * y + _02;
	_12 = _10 * x + _11 * y + _12;
	_22 = _20 * x + _21 * y + _22;
}

template <typename Scalar>
void Matrix3x3<Scalar>::RotateX(Scalar angle) {
	Scalar ca = cosT(angle * Scalar(M_PI / 180.0));
	Scalar sa = sinT(angle * Scalar(M_PI / 180.0));
	Scalar tmp;

	tmp = _01;
	_01 = tmp * ca - _02 * sa;
	_02 = _02 * ca + tmp * sa;

	tmp = _11;
	_11 = tmp * ca - _12 * sa;
	_12 = _12 * ca + tmp * sa;

	tmp = _21;
	_21 = tmp * ca - _22 * sa;
	_22 = _22 * ca + tmp * sa;
}

template <typename Scalar>
void Matrix3x3<Scalar>::RotateY(Scalar angle) {
	Scalar ca = cosT(angle * Scalar(M_PI / 180.0));
	Scalar sa = sinT(angle * Scalar(M_PI / 180.0));
	Scalar tmp;

	tmp = _00;
	_00 = tmp * ca + _02 * sa;
	_02 = -tmp * sa + _02 * ca;

	tmp = _10;
	_10 = tmp * ca + _12 * sa;
	_12 = -tmp * sa + _12 * ca;

	tmp = _20;
	_20 = tmp * ca + _22 * sa;
	_22 = -tmp * sa + _22 * ca;
}

template <typename Scalar>
void Matrix3x3<Scalar>::RotateZ(Scalar angle) {
	Scalar ca = cosT(angle * (M_PI / 180.0));
	Scalar sa = sinT(angle * (M_PI / 180.0));
	Scalar tmp;

	tmp = _00;
	_00 = tmp * ca - _01 * sa;
	_01 = _01 * ca + tmp * sa;

	tmp = _10;
	_10 = tmp * ca - _11 * sa;
	_11 = _11 * ca + tmp * sa;

	tmp = _20;
	_20 = tmp * ca - _21 * sa;
	_21 = _21 * ca + tmp * sa;
}

/* Rotation matrix (taken from Mesa3.1)
 original function contributed by Erich Boleyn (erich@uruk.org) */
template <typename Scalar>
void Matrix3x3<Scalar>::Rotate(Scalar angle, Scalar x, Scalar y, Scalar z) {
	Matrix3x3<Scalar> m;
	Scalar mag, s, c;
	Scalar xx, yy, zz, xy, yz, zx, xs, ys, zs, one_c;

	mag = sqrtT(x * x + y * y + z * z);
	if (mag == 0.) {
		return;
	}

	s = sinT(angle * (Scalar(M_PI) / Scalar(180.)));
	c = cosT(angle * (Scalar(M_PI) / Scalar(180.)));

	x /= mag;
	y /= mag;
	z /= mag;

	xx = x * x;
	yy = y * y;
	zz = z * z;
	xy = x * y;
	yz = y * z;
	zx = z * x;
	xs = x * s;
	ys = y * s;
	zs = z * s;
	one_c = 1.0F - c;

	m._00 = (one_c * xx) + c;
	m._01 = (one_c * xy) + zs;
	m._02 = (one_c * zx) - ys;

	m._10 = (one_c * xy) - zs;
	m._11 = (one_c * yy) + c;
	m._12 = (one_c * yz) + xs;

	// Invert z out to fit to my rotation definition
	m._20 = (one_c * zx) + ys;
	m._21 = (one_c * yz) - xs;
	m._22 = (one_c * zz) + c;

	operator*=(m);
}

/* setup a transformation from a standard coordinate system to a new reference
 frame that is specified by its origin and the orthonormal vectors x, y, and
 z. Units vectors of the source coordinate system will be transformed to the
 respective input vectors. The input vectors will be normalized immediatly
 and then y will be projected orthogonal to x. Finally, z will  bemade
 orthogonal to x and y, but it will keep the same relative direction to them.
 If the flag inverse is specified the transformation will we the inverse of
 the above specified transformation.
 NOTE: this member overwrites the current matrix so there's no need to call
 identity() before doing this operation ! */
template <typename Scalar>
void Matrix3x3<Scalar>::SetupReferenceFrame(const Vector<Scalar, 3>& x,
											const Vector<Scalar, 3>& y,
											const Vector<Scalar, 3>& z,
											bool inverse) {
	Vector<Scalar, 3> y1(y);
	y1.projectNormalTo(x);
	y1.normalize();
	Vector<Scalar, 3> z1(x % y);
	if ((z | z1) < 0)
		z1 = -z1;

	_00 = x[0], _01 = y1[0], _02 = z1[0];
	_10 = x[1], _11 = y1[1], _12 = z1[1];
	_20 = x[2], _21 = y1[2], _22 = z1[2];

	if (inverse)
		Invert();
}

template <typename Scalar>
void Matrix3x3<Scalar>::multMatrix(const Scalar* m) {
#define B(row, col) m[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar mi0 = M(i, 0), mi1 = M(i, 1), mi2 = M(i, 2);
		M(i, 0) = mi0 * B(0, 0) + mi1 * B(1, 0) + mi2 * B(2, 0);
		M(i, 1) = mi0 * B(0, 1) + mi1 * B(1, 1) + mi2 * B(2, 1);
		M(i, 2) = mi0 * B(0, 2) + mi1 * B(1, 2) + mi2 * B(2, 2);
	}
#undef B
}

template <typename Scalar>
void Matrix3x3<Scalar>::multMatrix(Scalar* dst, const Scalar* m) {
#define B(row, col) m[(row * 3) + col]
	int i;
	for (i = 0; i < 3; i++) {
		Scalar mi0 = M(i, 0), mi1 = M(i, 1), mi2 = M(i, 2);
		M(i, 0) = mi0 * B(0, 0) + mi1 * B(1, 0) + mi2 * B(2, 0);
		M(i, 1) = mi0 * B(0, 1) + mi1 * B(1, 1) + mi2 * B(2, 1);
		M(i, 2) = mi0 * B(0, 2) + mi1 * B(1, 2) + mi2 * B(2, 2);
	}
#undef B
}

template <typename Scalar>
bool base::Matrix3x3<Scalar>::FindRotation(Scalar& angle,
										   Vector<Scalar, 3>& axis) const {
	// From Graphics Gems I, pg. 466
	Scalar arg = (_00 + _11 + _22 - 1.0) / 2;
	if (arg > 1.0) {
		// null degree rotation
		axis[0] = axis[1] = axis[2] = 0.;
		angle = 0.;
		return false;
	}
	angle = acosT(arg);
	// modified by Mohan Raj Gupta, IITD

	// !!! temporary solution !!!
	Scalar t = sinT(angle);
	if (checkEpsilon(t * t)) {
		// null degree rotation
		axis[0] = axis[1] = axis[2] = 0.;
		angle = 0.;
		return false;
	}

	t *= 2.0;
	axis[0] = (_12 - _21) / t;
	axis[1] = (_20 - _02) / t;
	axis[2] = (_01 - _10) / t;
	angle = angle * (180.0 / M_PI);
	return true;
}

template <typename Scalar>
Scalar base::Matrix3x3<Scalar>::Trace() const {
	return _00 + _11 + _22;
}

template <typename Scalar>
Scalar base::Matrix3x3<Scalar>::Determinant() const {
	return _00 * _11 * _22 + _01 * _12 * _20 + _02 * _10 * _21 -
		   _02 * _11 * _20 - _01 * _10 * _22 - _00 * _12 * _21;
}

/* ======================================================== */
/* io                                                       */
/* ======================================================== */

/**
 * Outputs a matrix contents via stream.
 * @param os Output stream.
 * @param m Matrix to print.
 */
template <typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const Matrix3x3<Scalar>& m) {
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			os << m(i, j) << " ";
	return os;
}

/**
 * Reads the space-separated components of a matrix from a stream.
 * @param is Input stream.
 * @param m Matrix to be filled with the read data.
 */
template <typename Scalar>
inline std::istream& operator>>(std::istream& is, Matrix3x3<Scalar>& m) {
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			is >> m(i, j);
	return is;
}

template <typename Scalar>
void Matrix3x3<Scalar>::Print(const char* title) const {
	fprintf(stdout, "%s\n", title ? title : "");
	fprintf(stdout, "| % 6.6f  % 6.6f  % 6.6f |\n", _00, _01, _02);
	fprintf(stdout, "| % 6.6f  % 6.6f  % 6.6f |\n", _10, _11, _12);
	fprintf(stdout, "| % 6.6f  % 6.6f  % 6.6f |\n", _20, _21, _22);
	fflush(stdout);
}

/** ======================================================== */
/** typedefs                                                 */
/** ======================================================== */

/// Float 3x3 matrix.
typedef Matrix3x3<float> Matrix3x3f;
/// Double 3x3 matrix.
typedef Matrix3x3<double> Matrix3x3d;

}  // namespace base
#undef MATRIX_OPERATOR
#undef MAT
#undef M
#endif /* _MATRIX3X3_H_ */
