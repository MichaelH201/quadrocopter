#ifndef SRC_SCENE_CAMERAINTRINSICS_H_
#define SRC_SCENE_CAMERAINTRINSICS_H_

#include "../math/Matrix2x2.hh"
#include "../math/Vector.hh"

#define UNDISTORT_ITERATIONS 3	// TODO: Maybe tune?

#define PARAM_CAMERA_SX 0
#define PARAM_CAMERA_SY 1
#define PARAM_CAMERA_F 2
#define PARAM_CAMERA_PX 3
#define PARAM_CAMERA_PY 4
#define PARAM_CAMERA_R0 5
#define PARAM_CAMERA_R1 6
#define PARAM_CAMERA_R2 7
#define PARAM_CAMERA_R3 8
#define PARAM_CAMERA_R4 9
#define PARAM_CAMERA_R5 10
#define PARAM_CAMERA_T0 11
#define PARAM_CAMERA_T1 12

/**
 * Represents a camera's intrinsic parameters and provides transformations
 * between image- and viewspace.
 *
 * @tparam T Scalar type based on which data is stored.
 */
template <typename T>
struct CameraIntrinsics {
	/// Size of the camera images in pixels.
	base::Vector<T, 2> ImageSize;
	/// Focal length in x/y direction in pixels.
	T FocalLength;
	/// Principal point in x/y direction in pixels.
	base::Vector<T, 2> PrincipalPoint;
	/// Radial distortion
	base::Vector<T, 6> RadialDistortion;
	/// Tangential distortion
	base::Vector<T, 2> TangentialDistortion;

	/**
	 * Initializes all values with signaling NaNs to provoke errors if not
	 * initialized correctly later.
	 *
	 */
	CameraIntrinsics() {
		ImageSize[0] = std::numeric_limits<T>::signaling_NaN();
		ImageSize[1] = std::numeric_limits<T>::signaling_NaN();
		FocalLength = std::numeric_limits<T>::signaling_NaN();
		PrincipalPoint[0] = std::numeric_limits<T>::signaling_NaN();
		PrincipalPoint[1] = std::numeric_limits<T>::signaling_NaN();
		for (int d = 0; d < RadialDistortion.dim(); d++) {
			RadialDistortion[d] = std::numeric_limits<T>::signaling_NaN();
		}
		TangentialDistortion[0] = std::numeric_limits<T>::signaling_NaN();
		TangentialDistortion[1] = std::numeric_limits<T>::signaling_NaN();
	}

	/**
	 * Applies radial/tangential distortion on a normalized image position.
	 * @param in Normalized image position, e.g. viewspace coordinates with
	 * perspective division applied.
	 * @returns Distorted normalized image position.
	 */
	base::Vector<T, 2> applyDistortion(const base::Vector<T, 2>& in);

	/**
	 * Transforms a point from camera (view) space to image space (x, y, depth).
	 * @param in Input camera (view) space point.
	 * @returns Transformed point in image space (x, y, depth).
	 */
	base::Vector<T, 3> CameraSpace2Image(const base::Vector<T, 3>& in);

	/**
	 * Transforms a point from image space (x, y, depth) to camera (view) space.
	 * To invert the polynomlial radial/tangential distortion, 3 iterations of
	 * Gauss Newton optimization are performed which proved to provide an almost
	 * perfect and, more important, smooth and therefore differentiable result.
	 * @param in Input point in image space (x, y, depth).
	 * @returns Transformed point in camera (view) space.
	 */
	base::Vector<T, 3> Image2CameraSpace(const base::Vector<T, 3>& in);
};

template <typename T>
inline base::Vector<T, 2> CameraIntrinsics<T>::applyDistortion(
	const base::Vector<T, 2>& in) {
	// Radial and tangential distortion
	T radSquared = in[0] * in[0] + in[1] * in[1];

	T radDistFactor =
		(1.0 + RadialDistortion[0] * radSquared +
		 RadialDistortion[1] * radSquared * radSquared +
		 RadialDistortion[2] * radSquared * radSquared * radSquared) /
		(1.0 + RadialDistortion[3] * radSquared +
		 RadialDistortion[4] * radSquared * radSquared +
		 RadialDistortion[5] * radSquared * radSquared * radSquared);
	base::Vector<T, 2> normDistPos;
	normDistPos[0] =
		in[0] * radDistFactor + 2.0 * TangentialDistortion[0] * in[0] * in[1] +
		TangentialDistortion[1] * (radSquared + 2.0 * in[0] * in[0]);
	normDistPos[1] =
		in[1] * radDistFactor +
		TangentialDistortion[0] * (radSquared + 2.0 * in[1] * in[1]) +
		2.0 * TangentialDistortion[1] * in[0] * in[1];

	return normDistPos;
}

template <typename T>
inline base::Vector<T, 3> CameraIntrinsics<T>::CameraSpace2Image(
	const base::Vector<T, 3>& in) {
	// Perspective division
	base::Vector<T, 2> normalizedPos(in[0] / in[2], in[1] / in[2]);

	// Radial and tangential distortion
	base::Vector<T, 2> normDistPos = applyDistortion(normalizedPos);

	// Sensor alignment
	base::Vector<T, 3> imagePoint;
	imagePoint[0] = normDistPos[0] * FocalLength + (PrincipalPoint[0]);
	imagePoint[1] = normDistPos[1] * FocalLength + (PrincipalPoint[1]);
	imagePoint[2] = in[2];

	return imagePoint;
}

template <typename T>
inline base::Vector<T, 3> CameraIntrinsics<T>::Image2CameraSpace(
	const base::Vector<T, 3>& in) {
	// Delta on value for creating a derivative
	// Undo sensor alignment
	base::Vector<T, 2> normDistPos;
	normDistPos[0] = (in[0] - (PrincipalPoint[0])) / FocalLength;
	normDistPos[1] = (in[1] - (PrincipalPoint[1])) / FocalLength;

	// Gauss newton optimization of rectified position. Initialize with
	// distorted position.
	base::Vector<T, 2> normalizedPos = normDistPos;
	for (unsigned int i = 0; i < UNDISTORT_ITERATIONS; i++) {
		// Calculate finite distance gradients for x and y position
		base::Vector<T, 2> currentDistortedPos = applyDistortion(normalizedPos);
		base::Vector<T, 2> error = currentDistortedPos - normDistPos;
		base::Vector<T, 2> deltaX =
			(applyDistortion(normalizedPos + base::Vector<T, 2>(0.1, 0.0)) -
			 applyDistortion(normalizedPos + base::Vector<T, 2>(-0.1, 0.0))) /
			0.2;
		base::Vector<T, 2> deltaY =
			(applyDistortion(normalizedPos + base::Vector<T, 2>(0.0, 0.1)) -
			 applyDistortion(normalizedPos + base::Vector<T, 2>(0.0, -0.1))) /
			0.2;

		base::Matrix2x2<T> A;
		A._00 = deltaX[0];
		A._01 = deltaY[0];
		A._10 = deltaX[1];
		A._11 = deltaY[1];

		base::Matrix2x2<double> Ainv = A.Inverse();

		base::Vector<T, 2> delta = Ainv * error;

		normalizedPos -= delta;
	}

	// Extrude to space
	base::Vector<T, 3> result(normalizedPos[0] * in[2],
							  normalizedPos[1] * in[2], in[2]);

	return result;
}

#endif /* SRC_SCENE_CAMERAINTRINSICS_H_ */
