#ifndef _MATRIX4X4_H_
#define _MATRIX4X4_H_

#include "Matrix3x3.hh"
#include "TypelessMath.hh"
#include "Vector.hh"

#include <math.h>
#include <stdio.h>
#include <iostream>

#undef MAT
#define MAT(m, r, c) ((m)[((r) << 2) + (c)])
#define M(r, c) ((&_00)[((r) << 2) + (c)])

#define MATRIX_OPERATOR(src, dst, op) \
	{                                 \
		(dst)._00 op(src)._00;        \
		(dst)._01 op(src)._01;        \
		(dst)._02 op(src)._02;        \
		(dst)._03 op(src)._03;        \
		(dst)._10 op(src)._10;        \
		(dst)._11 op(src)._11;        \
		(dst)._12 op(src)._12;        \
		(dst)._13 op(src)._13;        \
		(dst)._20 op(src)._20;        \
		(dst)._21 op(src)._21;        \
		(dst)._22 op(src)._22;        \
		(dst)._23 op(src)._23;        \
		(dst)._30 op(src)._30;        \
		(dst)._31 op(src)._31;        \
		(dst)._32 op(src)._32;        \
		(dst)._33 op(src)._33;        \
	}
namespace base {

/**
 * Fixed size 4x4 matrix implementation.
 * @remarks Matrix was inplemented for a right hand coordinate system in which
 * x, y and z correspond to right, in and up. All rotations are CCW. This class
 * is intended to be used for multiplying its matrices from to left to
 * column vectors.<br>
 * For extensive and detailed documentation, refer to the very similar
 * Matrix3x3 documentation.
 */
template <typename Scalar>
struct Matrix4x4 {
	/// <i></i>
	typedef Scalar value_type;
	/// Returns number of rows.
	int Rows() const { return 4; }
	/// Returns number of columns.
	int Cols() const { return 4; }

	/** default constructor creates uninitialized values. */
	Matrix4x4() {}

	/** copy constructor (copy all elements) */
	inline Matrix4x4(const Matrix4x4<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, =);
	}

	/** copy constructor (copy all elements) */
	inline Matrix4x4(const Matrix3x3<Scalar>& inst) {
		_00 = inst._00, _01 = inst._01, _02 = inst._02, _03 = (Scalar)0;
		_10 = inst._10, _11 = inst._11, _12 = inst._12, _13 = (Scalar)0;
		_20 = inst._20, _21 = inst._21, _22 = inst._22, _23 = (Scalar)0;
		_30 = (Scalar)0, _31 = (Scalar)0, _32 = (Scalar)0, _33 = (Scalar)1;
	}

	/** copy constructor (copy all elements) */
	template <class otherScalar>
	inline Matrix4x4(const Matrix4x4<otherScalar>& inst) {
		MATRIX_OPERATOR(inst, *this, =);
	}

	/** copy constructor (copy all elements) */
	template <class otherScalar>
	inline Matrix4x4(const Matrix3x3<otherScalar>& inst) {
		_00 = inst._00, _01 = inst._01, _02 = inst._02, _03 = (Scalar)0;
		_10 = inst._10, _11 = inst._11, _12 = inst._12, _13 = (Scalar)0;
		_20 = inst._20, _21 = inst._21, _22 = inst._22, _23 = (Scalar)0;
		_30 = (Scalar)0, _31 = (Scalar)0, _32 = (Scalar)0, _33 = (Scalar)1;
	}

	inline operator Matrix3x3<Scalar>() const {
		Matrix3x3<Scalar> result;
		result._00 = _00, result._01 = _01, result._02 = _02;
		result._10 = _10, result._11 = _11, result._12 = _12;
		result._20 = _20, result._21 = _21, result._22 = _22;

		return result;
	}

	/** setup matrix using an array of N*N scalar values.
	 elements are ordered 'row first' */
	inline Matrix4x4(const Scalar array[]) {
		Scalar* dst = &_00;
		for (int i = 0; i < 4 * 4; i++)
			*dst++ = *array++;
	}

	/** constructor that creates a transformation matrix to a
	 reference frame as specified in the input parameters. */
	inline Matrix4x4(const Vector<Scalar, 3>& origin,
					 const Vector<Scalar, 3>& x,
					 const Vector<Scalar, 3>& y,
					 const Vector<Scalar, 3>& z);

	/** assignment (copy float array to this) */
	inline const Matrix4x4& operator=(const Scalar array[]) {
		Scalar* dst = &_00;
		for (int i = 0; i < 4 * 4; i++)
			*dst++ = *array++;
		return *this;
	}

	/** assignment operator. */
	inline const Matrix4x4& operator=(const Matrix4x4<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, =);
		return *this;
	}

	/** comonentwise adds inst to this and returns self-reference. */
	inline const Matrix4x4& operator+=(const Matrix4x4<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, +=);
		return *this;
	}

	/** comonentwise subtracts inst from this and returns self-reference. */
	inline const Matrix4x4& operator-=(const Matrix4x4<Scalar>& inst) {
		MATRIX_OPERATOR(inst, *this, -=);
		return *this;
	}

	/** compare two matrices. returns true if this has nearly the same
	 content as inst. */
	inline bool operator==(const Matrix4x4<Scalar>& inst) const {
		int i;
		const Scalar* a = &_00;
		const Scalar* b = &inst._00;
		for (i = 0; i < 16; i++, a++, b++)
			if (!checkEpsilon(*a - *b))
				return false;
		return true;
	}

	/** compare two matrices. */
	inline bool operator!=(const Matrix4x4<Scalar>& inst) const {
		return !(operator==(inst));
	}

	/** matrix by vector multiplication. */
	inline Vector<Scalar, 4> operator*(const Vector<Scalar, 4>& v) const;

	/** matrix by scalar multiplication */
	inline Matrix4x4<Scalar> operator*(const Scalar& s) const;

	/** transform point.*/
	inline Vector<Scalar, 3> TransformPoint(const Vector<Scalar, 3>& v) const;

	/** faster transform point assuming last row to be (0,0,0,1); */
	inline Vector<Scalar, 3> TransformPointHom(
		const Vector<Scalar, 3>& v) const;

	/** transform point (source is 3 component scalar vector) .*/
	inline Vector<Scalar, 3> TransformPoint(const Scalar* v) const;

	/** transform point (source is also destination, 3 component scalar
	 * vector).*/
	inline void TransformPointSelf(Vector<Scalar, 3>& v) const;

	/** transform point (source is also destination, 3 component scalar
	 * vector).*/
	inline void TransformPointSelf(Scalar* v) const;

	/** transform direction. */
	inline Vector<Scalar, 3> TransformVector(const Vector<Scalar, 3>& v) const;

	/** returns new matrix which holds the componentwise addition of this and
	 inst. */
	inline Matrix4x4 operator+(const Matrix4x4<Scalar>& inst) const {
		Matrix4x4<Scalar> m(inst);
		MATRIX_OPERATOR(*this, m, +=);
		return m;
	}

	/** returns new matrix which holds 'this - inst' (componentwise). */
	inline Matrix4x4 operator-(const Matrix4x4<Scalar>& inst) const {
		Matrix4x4<Scalar> m(*this);
		MATRIX_OPERATOR(inst, m, -=);
		return m;
	}

	/** matrix multiplication (post): 'self = self * inst'.
	 returns self reference*/
	const Matrix4x4& operator*=(const Matrix4x4<Scalar>& inst);
	const Matrix4x4& operator*=(const Scalar inst[]);
	const Matrix4x4& operator*=(const Scalar& inst);

	/** multiply self with the transposed of 'inst' */
	void MultTransposed(const Scalar inst[]);

	/** matrix multiplication (post): 'new = self * inst'.
	 returns new matrix*/
	Matrix4x4 operator*(const Matrix4x4<Scalar>& inst) const;

	/** matrix multiplication (pre): 'self = inst * self'. */
	void PreMult(const Matrix4x4<Scalar>& inst);
	void PreMult(const Scalar inst[]);
	/** same as above, except that the transposed matrix of inst will be used !
	 */
	void PreMultTransposed(const Scalar inst[]);

	/** access operator (read and write).*/
	inline Scalar& operator()(unsigned int row, unsigned int col) {
		return M(row, col);
	}

	/** access operator (read only).*/
	inline const Scalar& operator()(unsigned int row, unsigned int col) const {
		return M(row, col);
	}

	/** matrix inversion (returns true on success). */
	bool Invert();

	/**
	 * Iverts a view matrix.
	 * @remarks Such a matrix must be an affine transformation only consisting
	 * out of translation and rotation. The upper 3x3 part must be orthonormal.
	 */
	void InvertViewMatrix();
	Matrix4x4<Scalar> InverseViewMatrix() const;

	/** jacobi transformation */
	/* compute the jacobi transformation on a symmetric matrix */
	/* eigenvalues will be returned in _vals,                  */
	/* eigenvectors will be returned in _vecs                  */
	/* if sort is true e.values nor e.vectors are sorted     */
	/*
	 bool jacobi(Vector<Scalar, 4> &_vals, base::Matrix4x4<Scalar> &_vecs,
	 const bool sort = false, const unsigned int _maxIter = 100,
	 const float _epsilon = 1e-6);
	 */

	/** sets all elements to zero. */
	inline void Clear();
	/** setup an identity matrix. */
	inline void Identity();
	/** make a rotation matrix orthonormal again */
	inline void OrthonormalizeRotation();
	/** transpose self. */
	inline void Transpose();

	/** multiply self with translation matrix (x,y,z) */
	inline void Translate(Scalar x, Scalar y, Scalar z);
	/** multiply self with translation matrix (vector for x,y and z) */
	inline void Translate(const Vector<Scalar, 3>& v) {
		Translate(v[0], v[1], v[2]);
	}

	/** multiply self with scaling matrix (x,y,z) */
	inline void Scale(Scalar x, Scalar y, Scalar z);
	/** multiply self with scaling matrix (vector for x,y and z) */
	inline void Scale(const Vector<Scalar, 3>& v) { Scale(v[0], v[1], v[2]); }

	/** multiply self with a rotation matrix
	 (angle in degree, arbitrary axis given by xyz) */
	void Rotate(Scalar angle, Scalar x, Scalar y, Scalar z);
	/** multiply self with a rotation matrix
	 (angle in degree, arbitrary axis given by 3D-vector) */
	inline void Rotate(Scalar angle, const Vector<Scalar, 3>& v) {
		Rotate(angle, v[0], v[1], v[2]);
	}

	/** multiply self with a rotation matrix (angle in degree, x-axis) */
	inline void RotateX(Scalar angle);
	/** multiply self with a rotation matrix (angle in degree, y-axis) */
	inline void RotateY(Scalar angle);
	/** multiply self with a rotation matrix (angle in degree, z-axis) */
	inline void RotateZ(Scalar angle);

	/** multiply self with a rotation matrix that rotates v1 into v2 */
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

	/** multiply self with a perspective projection matrix */
	void Perspective(Scalar fovY, Scalar aspect, Scalar near, Scalar far);

	/** multiply self with a perspective projection matrix */
	/*
	 void perspective_stereo(Scalar fovY, Scalar aspect, Scalar near,
	 Scalar far, Scalar focallength, Scalar eyesep, bool left_or_right);
	 */

	/** multiply self with a perspective projection matrix.
	 this member is used for anti-aliasing, so we got some
	 more parameters for jittering (pix_dx,pix_dy)
	 plus the image resolution res_x and res_y). */
	/*
	 void perspective_aa(Scalar fovY, Scalar aspect, Scalar nearPlane,
	 Scalar farPlane, Scalar pix_dx, Scalar pix_dy, int res_x, int res_y)
	 {
	 Scalar xmin, xmax, ymin, ymax;
	 ymax = nearPlane * tanT(fovY * Scalar(M_PI) / Scalar(360.0));
	 ymin = -ymax;

	 xmin = ymin * aspect;
	 xmax = ymax * aspect;
	 frustum_aa(xmin, xmax, ymin, ymax, nearPlane, farPlane, pix_dx, pix_dy,
	 res_x, res_y);
	 }
	 */

	/** multiply self with the inverse of a perspective projection matrix */
	void InversePerspective(Scalar fovY,
							Scalar aspect,
							Scalar near,
							Scalar far);

	/** multiply self with a perspective projection matrix */
	void Frustum(Scalar left,
				 Scalar right,
				 Scalar bottom,
				 Scalar top,
				 Scalar nearval,
				 Scalar farval);

	/** multiply self with a perspective projection matrix.
	 this member is used for anti-aliasing, so we got some
	 more parameters for jittering (pix_dx,pix_dy)
	 plus the image resolution res_x and res_y). */
	/*
	 void frustum_aa(Scalar left, Scalar right, Scalar bottom, Scalar top,
	 Scalar nearval, Scalar farval, Scalar pix_dx, Scalar pix_dy,
	 int res_x, int res_y)
	 {
	 Scalar dx = -(pix_dx * (right - left) / Scalar(res_x));
	 Scalar dy = -(pix_dy * (top - bottom) / Scalar(res_y));
	 frustum(left + dx, right + dx, bottom + dy, top + dy, nearval, farval);
	 }
	 */

	/** multiply self with the inverse of a perspective projection matrix */
	void InverseFrustum(Scalar left,
						Scalar right,
						Scalar bottom,
						Scalar top,
						Scalar nearval,
						Scalar farval);

	/** setup a transformation from a standard coordinate system to
	 a new reference frame that is specified by its origin and the
	 orthonormal vectors x, y, and z. */
	void SetupReferenceFrame(const Vector<Scalar, 3>& origin,
							 const Vector<Scalar, 3>& x,
							 const Vector<Scalar, 3>& y,
							 const Vector<Scalar, 3>& z,
							 bool inverse);

	/** multiply self with a viewing transformation given by
	 eye position, reference point (center) and an up vector.
	 (similar to gluLookAt) */
	void LookAt(const Vector<Scalar, 3>& eye,
				const Vector<Scalar, 3>& center,
				const Vector<Scalar, 3>& up);

	/** multiply self with a viewing transformation given by
	 eye position, reference point (target) and roll (in degree). */
	inline void Camera(const Vector<Scalar, 3>& eye,
					   const Vector<Scalar, 3>& target,
					   Scalar roll);

	inline Scalar Trace() const;

	/** multiply self with a viewing transformation given by
	 eye position, reference point (target) and roll (in degree).
	 Both, eye and target, are in homegenous coordinates */
	void Camera(const Vector<Scalar, 4>& eye,
				const Vector<Scalar, 4>& target,
				Scalar roll) {
		Camera(eye.centralProjection(), target.centralProjection(), roll);
	}

	/** multiply self with a viewing transformation given by
	 eye position, target direction, and roll (in degree).
	 Both, eye is a 4D point, dir is 3D direction vector */
	void CameraDir(const Vector<Scalar, 4>& eye,
				   const Vector<Scalar, 3>& dir,
				   Scalar roll) {
		Vector<Scalar, 3> eye_p(eye.centralProjection());
		camera(eye_p, eye_p + dir, roll);
	}

	/** get column vectors */
	inline void GetCol(int col, Scalar*) const;
	/** get row vectors */
	inline void GetRow(int row, Scalar*) const;

	/** access to data array. not very nice, but in case of 4x4 matrices
	 this member can be used to pass matrices to OpenGL
	 (if it was transposed before)
	 e.g. glLoadMatrixf(m.getRawData()); */
	inline const Scalar* GetRawData() const { return &_00; }
	inline Scalar* GetRawData() { return &_00; }

	/** find single rotation (arbitrary axis) from a 4x4 matrix that
	 was generated by successive rotation calls.
	 (don't try to do this on non-rotation matrices !!!) */
	bool FindRotation(Scalar& angle, Vector<Scalar, 3>& axis) const;

	/** dump content in a 'readable' form to stdout. */
	void Print(const char* title = NULL) const;

	void MultMatrixFloat(const base::Matrix4x4<float>& m) {
		Scalar tmpmat[16];
		const float* mraw = m.GetRawData();
		for (int i = 0; i < 16; i++)
			tmpmat[i] = Scalar(mraw[i]);
		multMatrix(tmpmat);
	}
	void MultMatrixDouble(const base::Matrix4x4<double>& m) {
		Scalar tmpmat[16];
		const double* mraw = m.GetRawData();
		for (int i = 0; i < 16; i++)
			tmpmat[i] = Scalar(mraw[i]);
		multMatrix(tmpmat);
	}

	inline Vector<Scalar, 3> Position() const {
		return Vector<Scalar, 3>(_03, _13, _23);
	}

	inline void ShiftPosition(const Vector<Scalar, 3>& shift) {
		_03 += shift[0];
		_13 += shift[1];
		_23 += shift[2];
	}

	inline void ReplacePosition(const Vector<Scalar, 3>& pos) {
		_03 = pos[0];
		_13 = pos[1];
		_23 = pos[2];
	}

	inline void ReplaceRotation(const Matrix3x3<Scalar>& rot) {
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				(*this)(i, j) = rot(i, j);
	}

	Scalar _00;
	Scalar _01;
	Scalar _02;
	Scalar _03;
	Scalar _10;
	Scalar _11;
	Scalar _12;
	Scalar _13;
	Scalar _20;
	Scalar _21;
	Scalar _22;
	Scalar _23;
	Scalar _30;
	Scalar _31;
	Scalar _32;
	Scalar _33;

   protected:
	void multMatrix(const Scalar* m);
	void multMatrix(Scalar* dst, const Scalar* m);
};

/** constructor that creates a transformation matrix to a
 reference frame as specified in the input parameters.
 This is the inverse of a calibration matrix.*/
template <typename Scalar>
Matrix4x4<Scalar>::Matrix4x4(const Vector<Scalar, 3>& origin,
							 const Vector<Scalar, 3>& x,
							 const Vector<Scalar, 3>& y,
							 const Vector<Scalar, 3>& z) {
	_00 = x[0], _01 = y[0], _02 = z[0], _03 = origin[0];
	_10 = x[1], _11 = y[1], _12 = z[1], _13 = origin[1];
	_20 = x[2], _21 = y[2], _22 = z[2], _23 = origin[2];
	_30 = 0., _31 = 0., _32 = 0., _33 = 1.;
}

/** sets everything to zero .*/
template <typename Scalar>
void Matrix4x4<Scalar>::Clear() {
	Scalar* m = &_00;
	for (int i = 0; i < 4 * 4; i++)
		*m++ = 0.;
}

/** setup an identity matrix. */
template <typename Scalar>
void Matrix4x4<Scalar>::Identity() {
	_00 = 1., _01 = 0., _02 = 0., _03 = 0.;
	_10 = 0., _11 = 1., _12 = 0., _13 = 0.;
	_20 = 0., _21 = 0., _22 = 1., _23 = 0.;
	_30 = 0., _31 = 0., _32 = 0., _33 = 1.;
}

/** Orthonormalize the row vectors of the rotational part. */
template <typename Scalar>
void Matrix4x4<Scalar>::OrthonormalizeRotation() {
	Vec3f x(_00, _01, _02);
	Vec3f y(_10, _11, _12);
	Vec3f z(_20, _21, _22);

	z = x % y;
	x = y % z;
	x.normalize();
	y.normalize();
	z.normalize();

	_00 = x[0], _01 = x[1], _02 = x[2];
	_10 = y[0], _11 = y[1], _12 = y[2];
	_20 = z[0], _21 = z[1], _22 = z[2];
}

/** transform vector v` = A * v */
template <typename Scalar>
void Matrix4x4<Scalar>::Transpose() {
	Scalar tmp;
	for (int i = 0; i < 4; i++) {
		for (int j = i + 1; j < 4; j++) {
			tmp = M(i, j);
			M(i, j) = M(j, i);
			M(j, i) = tmp;
		}
	}
}

/** get column vectors */
template <typename Scalar>
void Matrix4x4<Scalar>::GetCol(int col, Scalar* data) const {
	data[0] = M(0, col);
	data[1] = M(1, col);
	data[2] = M(2, col);
	data[3] = M(3, col);
}

/** get row vectors */
template <typename Scalar>
void Matrix4x4<Scalar>::GetRow(int row, Scalar* data) const {
	data[0] = M(row, 0);
	data[1] = M(row, 1);
	data[2] = M(row, 2);
	data[3] = M(row, 3);
}

/** transform vector v` = A * v */
template <typename Scalar>
Vector<Scalar, 4> Matrix4x4<Scalar>::operator*(
	const Vector<Scalar, 4>& v) const {
	return Vector<Scalar, 4>(_00 * v[0] + _01 * v[1] + _02 * v[2] + _03 * v[3],
							 _10 * v[0] + _11 * v[1] + _12 * v[2] + _13 * v[3],
							 _20 * v[0] + _21 * v[1] + _22 * v[2] + _23 * v[3],
							 _30 * v[0] + _31 * v[1] + _32 * v[2] + _33 * v[3]);
}

template <typename Scalar>
Matrix4x4<Scalar> Matrix4x4<Scalar>::operator*(const Scalar& s) const {
	Matrix4x4<Scalar> result(*this);
	result *= s;

	return result;
}

/** transform point (x',y',z',1) = A * (x,y,z,1) */
template <typename Scalar>
Vector<Scalar, 3> Matrix4x4<Scalar>::TransformPoint(
	const Vector<Scalar, 3>& v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2] + _03;
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2] + _13;
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2] + _23;
	Scalar w1 = _30 * v[0] + _31 * v[1] + _32 * v[2] + _33;
	if (fabsT(w1) > 1e-5) {
		Scalar w = Scalar(1.0) / w1;
		return Vector<Scalar, 3>(x * w, y * w, z * w);
	} else {
		return Vector<Scalar, 3>(0., 0., 0.);
	}
}

/** transform point (x',y',z',1) = A * (x,y,z,1) assumes last row of
 A to be exact (0,0,0,1) */
template <typename Scalar>
Vector<Scalar, 3> Matrix4x4<Scalar>::TransformPointHom(
	const Vector<Scalar, 3>& v) const {
	return Vector<Scalar, 3>(_00 * v[0] + _01 * v[1] + _02 * v[2] + _03,
							 _10 * v[0] + _11 * v[1] + _12 * v[2] + _13,
							 _20 * v[0] + _21 * v[1] + _22 * v[2] + _23);
}

/** transform point (x',y',z',1) = A * (x,y,z,1) */
template <typename Scalar>
Vector<Scalar, 3> Matrix4x4<Scalar>::TransformPoint(const Scalar* v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2] + _03;
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2] + _13;
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2] + _23;
	Scalar w1 = _30 * v[0] + _31 * v[1] + _32 * v[2] + _33;
	if (fabsT(w1) > 1e-5) {
		Scalar w = 1.0 / w1;
		return Vector<Scalar, 3>(x * w, y * w, z * w);
	} else {
		return Vector<Scalar, 3>(0., 0., 0.);
	}
}

/** transform point (x',y',z',1) = A * (x,y,z,1) */
template <typename Scalar>
void Matrix4x4<Scalar>::TransformPointSelf(Vector<Scalar, 3>& v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2] + _03;
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2] + _13;
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2] + _23;
	Scalar w1 = _30 * v[0] + _31 * v[1] + _32 * v[2] + _33;
	w1 = (fabsT(w1) > 1e-5) ? (1.0 / w1) : 0.0;

	v[0] = x * w1;
	v[1] = y * w1;
	v[2] = z * w1;
}

/** transform point (x',y',z',1) = A * (x,y,z,1) */
template <typename Scalar>
void Matrix4x4<Scalar>::TransformPointSelf(Scalar* v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2] + _03;
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2] + _13;
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2] + _23;
	Scalar w1 = _30 * v[0] + _31 * v[1] + _32 * v[2] + _33;
	w1 = (fabsT(w1) > 1e-5) ? (1.0 / w1) : 0.0;

	v[0] = x * w1;
	v[1] = y * w1;
	v[2] = z * w1;
}

/** transform vector (x',y',z',0) = A * (x,y,z,0) */
template <typename Scalar>
Vector<Scalar, 3> Matrix4x4<Scalar>::TransformVector(
	const Vector<Scalar, 3>& v) const {
	Scalar x = _00 * v[0] + _01 * v[1] + _02 * v[2];
	Scalar y = _10 * v[0] + _11 * v[1] + _12 * v[2];
	Scalar z = _20 * v[0] + _21 * v[1] + _22 * v[2];
	return Vector<Scalar, 3>(x, y, z);
}

/** matrix multiplication (self` = self * inst) */
template <typename Scalar>
const Matrix4x4<Scalar>& Matrix4x4<Scalar>::operator*=(
	const Matrix4x4<Scalar>& inst) {
#define A(row, col) (&_00)[(row << 2) + col]
#define B(row, col) (&inst._00)[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar ai0 = A(i, 0), ai1 = A(i, 1), ai2 = A(i, 2), ai3 = A(i, 3);
		A(i, 0) = ai0 * B(0, 0) + ai1 * B(1, 0) + ai2 * B(2, 0) + ai3 * B(3, 0);
		A(i, 1) = ai0 * B(0, 1) + ai1 * B(1, 1) + ai2 * B(2, 1) + ai3 * B(3, 1);
		A(i, 2) = ai0 * B(0, 2) + ai1 * B(1, 2) + ai2 * B(2, 2) + ai3 * B(3, 2);
		A(i, 3) = ai0 * B(0, 3) + ai1 * B(1, 3) + ai2 * B(2, 3) + ai3 * B(3, 3);
	}
#undef A
#undef B
	return *this;
}

/** matrix multiplication (self` = self * inst) */
template <typename Scalar>
const Matrix4x4<Scalar>& Matrix4x4<Scalar>::operator*=(const Scalar inst[]) {
#define A(row, col) (&_00)[(row << 2) + col]
#define B(row, col) inst[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar ai0 = A(i, 0), ai1 = A(i, 1), ai2 = A(i, 2), ai3 = A(i, 3);
		A(i, 0) = ai0 * B(0, 0) + ai1 * B(1, 0) + ai2 * B(2, 0) + ai3 * B(3, 0);
		A(i, 1) = ai0 * B(0, 1) + ai1 * B(1, 1) + ai2 * B(2, 1) + ai3 * B(3, 1);
		A(i, 2) = ai0 * B(0, 2) + ai1 * B(1, 2) + ai2 * B(2, 2) + ai3 * B(3, 2);
		A(i, 3) = ai0 * B(0, 3) + ai1 * B(1, 3) + ai2 * B(2, 3) + ai3 * B(3, 3);
	}
#undef A
#undef B
	return *this;
}

template <typename Scalar>
const Matrix4x4<Scalar>& Matrix4x4<Scalar>::operator*=(const Scalar& inst) {
	for (int i = 0; i < 16; i++)
		(&_00)[i] *= inst;

	return *this;
}

/** matrix multiplication:
 (multiply the transposed of 'inst' on self)
 self` = self * inst_T  */
template <typename Scalar>
void Matrix4x4<Scalar>::MultTransposed(const Scalar inst[]) {
#define A(row, col) (&_00)[(row << 2) + col]
#define B(row, col) inst[(col << 2) + row]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar ai0 = A(i, 0), ai1 = A(i, 1), ai2 = A(i, 2), ai3 = A(i, 3);
		A(i, 0) = ai0 * B(0, 0) + ai1 * B(1, 0) + ai2 * B(2, 0) + ai3 * B(3, 0);
		A(i, 1) = ai0 * B(0, 1) + ai1 * B(1, 1) + ai2 * B(2, 1) + ai3 * B(3, 1);
		A(i, 2) = ai0 * B(0, 2) + ai1 * B(1, 2) + ai2 * B(2, 2) + ai3 * B(3, 2);
		A(i, 3) = ai0 * B(0, 3) + ai1 * B(1, 3) + ai2 * B(2, 3) + ai3 * B(3, 3);
	}
#undef A
#undef B
	return;
}

/** matrix multiplication (pre): 'self = inst * self'. */
template <typename Scalar>
void Matrix4x4<Scalar>::PreMult(const Matrix4x4<Scalar>& inst) {
#define A(row, col) &(inst._00)[(row << 2) + col]
#define B(row, col) (&_00)[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar b0i = B(0, i), b1i = B(1, i), b2i = B(2, i), b3i = B(3, i);
		B(0, i) = A(0, 0) * b0i + A(0, 1) * b1i + A(0, 2) * b2i + A(0, 3) * b3i;
		B(1, i) = A(1, 0) * b0i + A(1, 1) * b1i + A(1, 2) * b2i + A(1, 3) * b3i;
		B(2, i) = A(2, 0) * b0i + A(2, 1) * b1i + A(2, 2) * b2i + A(2, 3) * b3i;
		B(3, i) = A(3, 0) * b0i + A(3, 1) * b1i + A(3, 2) * b2i + A(3, 3) * b3i;
	}
#undef A
#undef B
	return;
}

/** matrix multiplication (pre): 'self = inst * self'. */
template <typename Scalar>
void Matrix4x4<Scalar>::PreMult(const Scalar inst[]) {
#define A(row, col) inst[(row << 2) + col]
#define B(row, col) (&_00)[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar b0i = B(0, i), b1i = B(1, i), b2i = B(2, i), b3i = B(3, i);
		B(0, i) = A(0, 0) * b0i + A(0, 1) * b1i + A(0, 2) * b2i + A(0, 3) * b3i;
		B(1, i) = A(1, 0) * b0i + A(1, 1) * b1i + A(1, 2) * b2i + A(1, 3) * b3i;
		B(2, i) = A(2, 0) * b0i + A(2, 1) * b1i + A(2, 2) * b2i + A(2, 3) * b3i;
		B(3, i) = A(3, 0) * b0i + A(3, 1) * b1i + A(3, 2) * b2i + A(3, 3) * b3i;
	}
#undef A
#undef B
	return;
}

/** matrix multiplication (pre): 'self = inst_T * self'.
 where 'inst' is the transposed matrix of 'inst'  */
template <typename Scalar>
void Matrix4x4<Scalar>::PreMultTransposed(const Scalar inst[]) {
#define A(row, col) inst[(col << 2) + row]
#define B(row, col) (&_00)[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar b0i = B(0, i), b1i = B(1, i), b2i = B(2, i), b3i = B(3, i);
		B(0, i) = A(0, 0) * b0i + A(0, 1) * b1i + A(0, 2) * b2i + A(0, 3) * b3i;
		B(1, i) = A(1, 0) * b0i + A(1, 1) * b1i + A(1, 2) * b2i + A(1, 3) * b3i;
		B(2, i) = A(2, 0) * b0i + A(2, 1) * b1i + A(2, 2) * b2i + A(2, 3) * b3i;
		B(3, i) = A(3, 0) * b0i + A(3, 1) * b1i + A(3, 2) * b2i + A(3, 3) * b3i;
	}
#undef A
#undef B
	return;
}

/** matrix multiplication (ret = self * inst) */
template <typename Scalar>
Matrix4x4<Scalar> Matrix4x4<Scalar>::operator*(
	const Matrix4x4<Scalar>& inst) const {
	Matrix4x4<Scalar> tmp;

	/** code snippet taken from MesaLib 3.1 */
#define P(row, col) (&tmp._00)[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar ai0 = M(i, 0), ai1 = M(i, 1), ai2 = M(i, 2), ai3 = M(i, 3);
		P(i, 0) =
			ai0 * inst._00 + ai1 * inst._10 + ai2 * inst._20 + ai3 * inst._30;
		P(i, 1) =
			ai0 * inst._01 + ai1 * inst._11 + ai2 * inst._21 + ai3 * inst._31;
		P(i, 2) =
			ai0 * inst._02 + ai1 * inst._12 + ai2 * inst._22 + ai3 * inst._32;
		P(i, 3) =
			ai0 * inst._03 + ai1 * inst._13 + ai2 * inst._23 + ai3 * inst._33;
	}
#undef P
	return tmp;
}

/*
 * Compute inverse of 4x4 transformation matrix.
 * Taken from Mesa3.1
 * Code contributed by Jacques Leroy jle@star.be */
template <typename Scalar>
bool Matrix4x4<Scalar>::Invert() {
	// FIXME: Matrix inversion fails for some view matrices (only in release
	// mode) For an example, create one with camera.Alignment.LookAt(Vec3f(0.0f,
	// 0.0f, 10.0f), Vec3f(0.0f, 0.0f, 0.0f), Vec3f(0.0f, 1.0f, 0.0f));
#define SWAP_ROWS(a, b)   \
	{                     \
		Scalar* _tmp = a; \
		(a) = (b);        \
		(b) = _tmp;       \
	}
	// Scalar *m = &_00;
	Scalar wtmp[4][8];
	Scalar m0, m1, m2, m3, s;
	Scalar *r0, *r1, *r2, *r3;

	r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];

	r0[0] = _00, r0[1] = _01, r0[2] = _02, r0[3] = _03;
	r0[4] = 1.0, r0[5] = 0.0, r0[6] = 0.0, r0[7] = 0.0;

	r1[0] = _10, r1[1] = _11, r1[2] = _12, r1[3] = _13;
	r1[4] = 0.0, r1[5] = 1.0, r1[6] = 0.0, r1[7] = 0.0;

	r2[0] = _20, r2[1] = _21, r2[2] = _22, r2[3] = _23;
	r2[4] = 0.0, r2[5] = 0.0, r2[6] = 1.0, r2[7] = 0.0;

	r3[0] = _30, r3[1] = _31, r3[2] = _32, r3[3] = _33;
	r3[4] = 0.0, r3[5] = 0.0, r3[6] = 0.0, r3[7] = 1.0;

	/* choose pivot - or die */
	if (fabsT(r3[0]) > fabsT(r2[0]))
		SWAP_ROWS(r3, r2);
	if (fabsT(r2[0]) > fabsT(r1[0]))
		SWAP_ROWS(r2, r1);
	if (fabsT(r1[0]) > fabsT(r0[0]))
		SWAP_ROWS(r1, r0);
	if (0.0 == r0[0])
		return false;

	/* eliminate first variable     */
	m1 = r1[0] / r0[0];
	m2 = r2[0] / r0[0];
	m3 = r3[0] / r0[0];
	s = r0[1];
	r1[1] -= m1 * s;
	r2[1] -= m2 * s;
	r3[1] -= m3 * s;
	s = r0[2];
	r1[2] -= m1 * s;
	r2[2] -= m2 * s;
	r3[2] -= m3 * s;
	s = r0[3];
	r1[3] -= m1 * s;
	r2[3] -= m2 * s;
	r3[3] -= m3 * s;
	s = r0[4];
	if (s != 0.0) {
		r1[4] -= m1 * s;
		r2[4] -= m2 * s;
		r3[4] -= m3 * s;
	}
	s = r0[5];
	if (s != 0.0) {
		r1[5] -= m1 * s;
		r2[5] -= m2 * s;
		r3[5] -= m3 * s;
	}
	s = r0[6];
	if (s != 0.0) {
		r1[6] -= m1 * s;
		r2[6] -= m2 * s;
		r3[6] -= m3 * s;
	}
	s = r0[7];
	if (s != 0.0) {
		r1[7] -= m1 * s;
		r2[7] -= m2 * s;
		r3[7] -= m3 * s;
	}

	/* choose pivot - or die */
	if (fabsT(r3[1]) > fabsT(r2[1]))
		SWAP_ROWS(r3, r2);
	if (fabsT(r2[1]) > fabsT(r1[1]))
		SWAP_ROWS(r2, r1);
	if (0.0 == r1[1])
		return false;

	/* eliminate second variable */
	m2 = r2[1] / r1[1];
	m3 = r3[1] / r1[1];
	r2[2] -= m2 * r1[2];
	r3[2] -= m3 * r1[2];
	r2[3] -= m2 * r1[3];
	r3[3] -= m3 * r1[3];
	s = r1[4];
	if (0.0 != s) {
		r2[4] -= m2 * s;
		r3[4] -= m3 * s;
	}
	s = r1[5];
	if (0.0 != s) {
		r2[5] -= m2 * s;
		r3[5] -= m3 * s;
	}
	s = r1[6];
	if (0.0 != s) {
		r2[6] -= m2 * s;
		r3[6] -= m3 * s;
	}
	s = r1[7];
	if (0.0 != s) {
		r2[7] -= m2 * s;
		r3[7] -= m3 * s;
	}

	/* choose pivot - or die */
	if (fabsT(r3[2]) > fabsT(r2[2]))
		SWAP_ROWS(r3, r2);
	if (0.0 == r2[2])
		return false;

	/* eliminate third variable */
	m3 = r3[2] / r2[2];
	r3[3] -= m3 * r2[3], r3[4] -= m3 * r2[4], r3[5] -= m3 * r2[5],
		r3[6] -= m3 * r2[6], r3[7] -= m3 * r2[7];

	/* last check */
	if (0.0 == r3[3])
		return false;

	s = 1.0f / r3[3]; /* now back substitute row 3 */
	r3[4] *= s;
	r3[5] *= s;
	r3[6] *= s;
	r3[7] *= s;

	m2 = r2[3]; /* now back substitute row 2 */
	s = 1.0f / r2[2];
	r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
	r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
	m1 = r1[3];
	r1[4] -= r3[4] * m1, r1[5] -= r3[5] * m1, r1[6] -= r3[6] * m1,
		r1[7] -= r3[7] * m1;
	m0 = r0[3];
	r0[4] -= r3[4] * m0, r0[5] -= r3[5] * m0, r0[6] -= r3[6] * m0,
		r0[7] -= r3[7] * m0;

	m1 = r1[2]; /* now back substitute row 1 */
	s = 1.0f / r1[1];
	r1[4] = s * (r1[4] - r2[4] * m1), r1[5] = s * (r1[5] - r2[5] * m1),
	r1[6] = s * (r1[6] - r2[6] * m1), r1[7] = s * (r1[7] - r2[7] * m1);
	m0 = r0[2];
	r0[4] -= r2[4] * m0, r0[5] -= r2[5] * m0, r0[6] -= r2[6] * m0,
		r0[7] -= r2[7] * m0;

	m0 = r0[1]; /* now back substitute row 0 */
	s = 1.0f / r0[0];
	r0[4] = s * (r0[4] - r1[4] * m0), r0[5] = s * (r0[5] - r1[5] * m0),
	r0[6] = s * (r0[6] - r1[6] * m0), r0[7] = s * (r0[7] - r1[7] * m0);

	_00 = r0[4], _01 = r0[5], _02 = r0[6], _03 = r0[7];
	_10 = r1[4], _11 = r1[5], _12 = r1[6], _13 = r1[7];
	_20 = r2[4], _21 = r2[5], _22 = r2[6], _23 = r2[7];
	_30 = r3[4], _31 = r3[5], _32 = r3[6], _33 = r3[7];

	return true;
#undef SWAP_ROWS
}

template <typename Scalar>
void Matrix4x4<Scalar>::InvertViewMatrix() {
	Transpose();
	_03 = -(_00 * _30 + _01 * _31 + _02 * _32);
	_13 = -(_10 * _30 + _11 * _31 + _12 * _32);
	_23 = -(_20 * _30 + _21 * _31 + _22 * _32);
	_30 = (Scalar)0;
	_31 = (Scalar)0;
	_32 = (Scalar)0;
}

template <typename Scalar>
Matrix4x4<Scalar> Matrix4x4<Scalar>::InverseViewMatrix() const {
	Matrix4x4<Scalar> result(*this);
	result.InvertViewMatrix();
	return result;
}

/* pinched and adopted from Mario */
/*
 template<typename Scalar> bool Matrix4x4<Scalar>::jacobi(
 Vector<Scalar, 4> &_vals, base::Matrix4x4<Scalar> &_vecs,
 const bool sort, const unsigned int _maxIter, const float _epsilon)
 {
 // check symmetry
 int row, col;
 for (row = 0; row < 3; row++) // row
 for (col = row + 1; col < 4; col++) // column
 if (MAT(mat,row,col) != MAT(mat,col,row))
 return false;

 //    typedef SM::value_type T;

 //    assert(_epsilon>=T(0));

 //    DenseMatrix<T> Rotation(n,n),
 //                   Tmp(n,n),
 //                   A(n,n);
 //    load(A,_SM);
 //    load(_Evec,UnitMatrix<T>(_Evec));

 int numIter;
 Matrix4x4<Scalar> A = (*this);
 Matrix4x4<Scalar> Rotation, Tmp, Rot_trans;
 _vecs.identity();

 for (numIter = 0; numIter < _maxIter; numIter++)
 {

 // find largest off-diagonal element (matrix is symmetric)
 int row = 0;
 int col = 1;
 Scalar f_max, f;
 f_max = fabs(MAT( A.mat,row,col));

 for (int rr = 0; rr < 3; rr++)
 for (int cc = rr + 1; cc < 4; cc++)
 if ((f = fabs(MAT( A.mat, rr, cc ))) > f_max)
 {
 f_max = f;
 row = rr;
 col = cc;
 }

 if (f_max <= _epsilon)
 break;

 // compute Jacobi rotation
 Scalar theta = 0.5 * (MAT(A.mat,col,col) - MAT(A.mat,row,row))
 / MAT(A.mat,row,col);
 Scalar t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
 t *= (theta < 0.0) ? -1.0 : 1.0;

 Scalar c = 1.0 / sqrt(1.0 + t * t);
 Scalar s = t * c;

 Rotation.identity();
 MAT(Rotation.mat, row, row) = c;
 MAT(Rotation.mat, col, col) = c;
 MAT(Rotation.mat, row, col) = s;
 MAT(Rotation.mat, col, row) = -s;

 Rot_trans = Rotation;
 Rot_trans.transpone();

 Tmp = Rot_trans * A;
 A = Tmp * Rotation;
 Tmp = _vecs;
 _vecs = Tmp * Rotation;

 } // for: Jacobi-iterations

 for (row = 0; row < 4; row++)
 _vals[row] = MAT(A.mat,row,row);

 if (numIter >= _maxIter)
 {
 fprintf(stderr, "Jacobi Transformation failed\n");
 fprintf(stderr, "numIter: %d, maxIter %d\n", numIter, _maxIter);
 fprintf(stderr, "\noriginal: %2.2f %2.2f %2.2f %2.2f\n", (*this)(0, 0),
 (*this)(1, 0), (*this)(2, 0), (*this)(3, 0));
 fprintf(stderr, "        : %2.2f %2.2f %2.2f %2.2f\n", (*this)(0, 1),
 (*this)(1, 1), (*this)(2, 1), (*this)(3, 1));
 fprintf(stderr, "        : %2.2f %2.2f %2.2f %2.2f\n", (*this)(0, 2),
 (*this)(1, 2), (*this)(2, 2), (*this)(3, 2));
 fprintf(stderr, "        : %2.2f %2.2f %2.2f %2.2f\n\n\n",
 (*this)(0, 3), (*this)(1, 3), (*this)(2, 3), (*this)(3, 3));

 return false;
 }

 if (sort == true)
 {
 int k, j, i;
 Scalar p;

 for (i = 0; i < 4; i++)
 {
 p = _vals[k = i];

 for (j = i + 1; j < 4; j++)
 if (_vals[j] >= p)
 p = _vals[k = j];

 if (k != i)
 {
 _vals[k] = _vals[i];
 _vals[i] = p;
 for (j = 0; j < 4; j++)
 {
 Scalar tmp = MAT(_vecs.mat,j,i);
 MAT(_vecs.mat,j,i) = MAT( _vecs.mat,j,k);
 MAT(_vecs.mat,j,k) = tmp;
 }
 }
 }
 }

 return true;
 }
 */

/** multiply self with a perspective projection matrix */
template <typename Scalar>
void Matrix4x4<Scalar>::Perspective(Scalar fovY,
									Scalar aspect,
									Scalar nearPlane,
									Scalar farPlane) {
	Scalar xmin, xmax, ymin, ymax;

	ymax = nearPlane * tanT(fovY * Scalar(M_PI) / Scalar(360.0));
	ymin = -ymax;

	xmin = ymin * aspect;
	xmax = ymax * aspect;
	Frustum(xmin, xmax, ymin, ymax, nearPlane, farPlane);
}

/** multiply self with a perspective projection matrix */
/*
 template<typename Scalar> void Matrix4x4<Scalar>::perspective_stereo(
 Scalar fovY, Scalar aspect, Scalar nearPlane, Scalar farPlane,
 Scalar focallength, Scalar eyesep, bool left_or_right)
 {
 Scalar xmin, xmax, ymin, ymax;

 Scalar wd2 = nearPlane * tanT(fovY * Scalar(M_PI) / Scalar(360.0));
 Scalar ndfl = nearPlane / focallength;
 Scalar sep = (left_or_right ? Scalar(0.5) : Scalar(-0.5)) * eyesep;

 xmin = -aspect * wd2 + sep * ndfl;
 xmax = aspect * wd2 + sep * ndfl;
 ymax = wd2;
 ymin = -wd2;

 frustum(xmin, xmax, ymin, ymax, nearPlane, farPlane);
 }
 */

/** multiply self with the inverse of a perspective projection matrix */
template <typename Scalar>
void Matrix4x4<Scalar>::InversePerspective(Scalar fovY,
										   Scalar aspect,
										   Scalar nearPlane,
										   Scalar farPlane) {
	Scalar xmin, xmax, ymin, ymax;

	ymax = nearPlane * tanT(fovY * Scalar(M_PI) / Scalar(360.0));
	ymin = -ymax;

	xmin = ymin * aspect;
	xmax = ymax * aspect;
	InverseFrustum(xmin, xmax, ymin, ymax, nearPlane, farPlane);
}

template <typename Scalar>
void Matrix4x4<Scalar>::Frustum(Scalar left,
								Scalar right,
								Scalar bottom,
								Scalar top,
								Scalar nearval,
								Scalar farval) {
	//  assert(nearval > 0.0 && farval > 0.0);

	Scalar x, y, a, b, c, d;
	Matrix4x4<Scalar> m;

	x = (Scalar(2.0) * nearval) / (right - left);
	y = (Scalar(2.0) * nearval) / (top - bottom);
	a = (right + left) / (right - left);
	b = (top + bottom) / (top - bottom);
	c = farval / (farval - nearval);
	d = -(farval * nearval) / (farval - nearval);

	m._00 = x, m._01 = a, m._02 = 0.0F, m._03 = 0.0F;
	m._10 = 0.0F, m._11 = b, m._12 = y, m._13 = 0.0F;
	m._20 = 0.0F, m._21 = c, m._22 = 0.0F, m._23 = d;
	m._30 = 0.0F, m._31 = 1.0F, m._32 = 0.0F, m._33 = 0.0F;

	operator*=(m);
}

/** multiply this with an inverse viewing frustum */
template <typename Scalar>
void Matrix4x4<Scalar>::InverseFrustum(Scalar left,
									   Scalar right,
									   Scalar bottom,
									   Scalar top,
									   Scalar nearval,
									   Scalar farval) {
	assert(nearval > 0.0 && farval > 0.0);

	Scalar x, y, a, b, c, d;
	Matrix4x4<Scalar> m;

	x = (right - left) / (Scalar(2.0) * nearval);
	y = (top - bottom) / (Scalar(2.0) * nearval);
	a = -(right + left) / (Scalar(2.0) * nearval);
	b = -(top + bottom) / (Scalar(2.0) * nearval);
	c = 1 / nearval;
	d = -(farval - nearval) / (farval * nearval);

	m._00 = x, m._01 = 0.0F, m._02 = 0.0F, m._03 = a;
	m._10 = 0.0F, m._11 = 0.0F, m._12 = 0.0F, m._13 = 1.0;
	m._20 = 0.0F, m._21 = y, m._22 = 0.0F, m._23 = b;
	m._30 = 0.0F, m._31 = 0.0F, m._32 = d, m._33 = c;

	operator*=(m);
}

template <typename Scalar>
void Matrix4x4<Scalar>::Scale(Scalar x, Scalar y, Scalar z) {
	if (x != 1.0) {
		_00 *= x;
		_10 *= x;
		_20 *= x;
		_30 *= x;
	}
	if (y != 1.0) {
		_01 *= y;
		_11 *= y;
		_21 *= y;
		_31 *= y;
	}
	if (z != 1.0) {
		_02 *= z;
		_12 *= z;
		_22 *= z;
		_32 *= z;
	}
}

template <typename Scalar>
inline void Matrix4x4<Scalar>::Translate(Scalar x, Scalar y, Scalar z) {
	_03 = _00 * x + _01 * y + _02 * z + _03;
	_13 = _10 * x + _11 * y + _12 * z + _13;
	_23 = _20 * x + _21 * y + _22 * z + _23;
	_33 = _30 * x + _31 * y + _32 * z + _33;
}

template <typename Scalar>
void Matrix4x4<Scalar>::RotateX(Scalar angle) {
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

	tmp = _31;
	_31 = tmp * ca - _32 * sa;
	_32 = _32 * ca + tmp * sa;
}

template <typename Scalar>
void Matrix4x4<Scalar>::RotateY(Scalar angle) {
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

	tmp = _30;
	_30 = tmp * ca + _32 * sa;
	_32 = -tmp * sa + _32 * ca;
}

template <typename Scalar>
void Matrix4x4<Scalar>::RotateZ(Scalar angle) {
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

	tmp = _30;
	_30 = tmp * ca - _31 * sa;
	_31 = _31 * ca + tmp * sa;
}

/* Rotation matrix (taken from Mesa3.1)
 original function contributed by Erich Boleyn (erich@uruk.org) */
template <typename Scalar>
void Matrix4x4<Scalar>::Rotate(Scalar angle, Scalar x, Scalar y, Scalar z) {
	Matrix4x4<Scalar> m;
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
	m._03 = 0.0F;

	m._10 = (one_c * xy) - zs;
	m._11 = (one_c * yy) + c;
	m._12 = (one_c * yz) + xs;
	m._13 = 0.0F;

	// Invert z out to fit to my rotation definition
	m._20 = (one_c * zx) + ys;
	m._21 = (one_c * yz) - xs;
	m._22 = (one_c * zz) + c;
	m._23 = 0.0F;

	m._30 = 0.0F;
	m._31 = 0.0F;
	m._32 = 0.0F;
	m._33 = 1.0F;

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
void Matrix4x4<Scalar>::SetupReferenceFrame(const Vector<Scalar, 3>& origin,
											const Vector<Scalar, 3>& x,
											const Vector<Scalar, 3>& y,
											const Vector<Scalar, 3>& z,
											bool inverse) {
	Vector<Scalar, 3> y1(y);
	y1.projectNormalTo(x);
	y1.normalize();
	Vector<Scalar, 3> z1(x % y);
	if ((z | z1) < 0)
		z1 = -z1;

	_00 = x[0], _01 = y1[0], _02 = z1[0], _03 = origin[0];
	_10 = x[1], _11 = y1[1], _12 = z1[1], _13 = origin[1];
	_20 = x[2], _21 = y1[2], _22 = z1[2], _23 = origin[2];
	_30 = 0., _31 = 0., _32 = 0., _33 = 1.;

	if (inverse)
		Invert();
}

/** matrix multiplication: multiply by m (which should point to 16 scalar
 * values) */
template <typename Scalar>
void Matrix4x4<Scalar>::multMatrix(const Scalar* m) {
#define B(row, col) m[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar mi0 = M(i, 0), mi1 = M(i, 1), mi2 = M(i, 2), mi3 = M(i, 3);
		M(i, 0) = mi0 * B(0, 0) + mi1 * B(1, 0) + mi2 * B(2, 0) + mi3 * B(3, 0);
		M(i, 1) = mi0 * B(0, 1) + mi1 * B(1, 1) + mi2 * B(2, 1) + mi3 * B(3, 1);
		M(i, 2) = mi0 * B(0, 2) + mi1 * B(1, 2) + mi2 * B(2, 2) + mi3 * B(3, 2);
		M(i, 3) = mi0 * B(0, 3) + mi1 * B(1, 3) + mi2 * B(2, 3) + mi3 * B(3, 3);
	}
#undef B
}

/** matrix multiplication: multiply by m (which should point to 16 scalar
 * values) */
template <typename Scalar>
void Matrix4x4<Scalar>::multMatrix(Scalar* dst, const Scalar* m) {
#define B(row, col) m[(row << 2) + col]
	int i;
	for (i = 0; i < 4; i++) {
		Scalar mi0 = M(i, 0), mi1 = M(i, 1), mi2 = M(i, 2), mi3 = M(i, 3);
		M(i, 0) = mi0 * B(0, 0) + mi1 * B(1, 0) + mi2 * B(2, 0) + mi3 * B(3, 0);
		M(i, 1) = mi0 * B(0, 1) + mi1 * B(1, 1) + mi2 * B(2, 1) + mi3 * B(3, 1);
		M(i, 2) = mi0 * B(0, 2) + mi1 * B(1, 2) + mi2 * B(2, 2) + mi3 * B(3, 2);
		M(i, 3) = mi0 * B(0, 3) + mi1 * B(1, 3) + mi2 * B(2, 3) + mi3 * B(3, 3);
	}
#undef B
}

template <typename Scalar>
bool base::Matrix4x4<Scalar>::FindRotation(Scalar& angle,
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

/** multiply self with a viewing transformation given by
 eye position, reference point (center) and an up vector.
 (similar to gluLookAt) */
template <typename Scalar>
void base::Matrix4x4<Scalar>::LookAt(const Vector<Scalar, 3>& eye,
									 const Vector<Scalar, 3>& center,
									 const Vector<Scalar, 3>& up) {
	Vector<Scalar, 3> y(center - eye);
	y.normalize();

	Vector<Scalar, 3> x(y % up);
	Vector<Scalar, 3> z(x % y);
	x.normalize();
	z.normalize();

	Matrix4x4<Scalar> t;

	t._00 = x[0], t._01 = x[1], t._02 = x[2], t._03 = 0.0;
	t._10 = y[0], t._11 = y[1], t._12 = y[2], t._13 = 0.0;
	t._20 = z[0], t._21 = z[1], t._22 = z[2], t._23 = 0.0;
	t._30 = 0.0, t._31 = 0.0, t._32 = 0.0, t._33 = 1.0;

	operator*=(t);
	// move eye to origin
	Translate(-eye);
}

/** multiply self with a viewing transformation given by
 eye position, reference point (target) and roll (in degree).
 Handling of degenerated cases (viewing direction 0,1,0 or 0,-1,0
 is done as in 3D Studio Max.
 */
template <typename Scalar>
void base::Matrix4x4<Scalar>::Camera(const Vector<Scalar, 3>& eye,
									 const Vector<Scalar, 3>& target,
									 Scalar roll) {
	Matrix4x4<Scalar> t;
	base::Vector<Scalar, 3> x, z;

	// Get our direction vector (the Z vector component of the
	// and make sure it's normalized into a unit vector
	base::Vector<Scalar, 3> y(target - eye);
	y.normalize();

	if (checkEpsilon(y[0]) && checkEpsilon(y[1]))
		z = base::Vector<Scalar, 3>(-y[1], 0.0, 0.0);
	else
		z = base::Vector<Scalar, 3>(0.0, 0.0, 1.0);

	x = y % z;
	x.normalize();

	z = x % y;
	y.normalize();

	t._00 = x[0], t._01 = x[1], t._02 = x[2], t._03 = 0.0;
	t._10 = y[0], t._11 = y[1], t._12 = y[2], t._13 = 0.0;
	t._20 = z[0], t._21 = z[1], t._22 = z[2], t._23 = 0.0;
	t._30 = 0.0, t._31 = 0.0, t._32 = 0.0, t._33 = 1.0;

	base::Matrix4x4<Scalar> rollm;
	rollm.Identity();
	rollm.RotateY(roll);
	operator*=(rollm);

	operator*=(t);
	Translate(-eye[0], -eye[1], -eye[2]);
}

template <typename Scalar>
Scalar base::Matrix4x4<Scalar>::Trace() const {
	return _00 + _11 + _22 + _33;
}

/** ======================================================== */
/** io                                                       */
/** ======================================================== */

/** output matrix */
template <typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const Matrix4x4<Scalar>& m) {
	int i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++)
			os << m(i, j) << " ";
		os << std::endl;
	}
	return os;
}

/** read the space-separated components of a vector from a stream */
template <typename Scalar>
inline std::istream& operator>>(std::istream& is, Matrix4x4<Scalar>& m) {
	int i, j;
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			is >> m(i, j);
	return is;
}

/** debug output */
template <typename Scalar>
void Matrix4x4<Scalar>::Print(const char* title) const {
	fprintf(stdout, "%s\n", title ? title : "");
	fprintf(stdout, "| % 6.6f  % 6.6f  % 6.6f  % 6.6f |\n", _00, _01, _02, _03);
	fprintf(stdout, "| % 6.6f  % 6.6f  % 6.6f  % 6.6f |\n", _10, _11, _12, _13);
	fprintf(stdout, "| % 6.6f  % 6.6f  % 6.6f  % 6.6f |\n", _20, _21, _22, _23);
	fprintf(stdout, "| % 6.6f  % 6.6f  % 6.6f  % 6.6f |\n", _30, _31, _32, _33);
	fflush(stdout);
}

/** ======================================================== */
/** typedefs                                                 */
/** ======================================================== */

/// Float 4x4 matrix.
typedef Matrix4x4<float> Matrix4x4f;
/// Douböe 4x4 matrix.
typedef Matrix4x4<double> Matrix4x4d;

}  // namespace base
#undef MATRIX_OPERATOR
#undef MAT
#undef M
#endif /* _MATRIX4X4_H_ */
