#ifndef CAMERAPOSEDATA_H_
#define CAMERAPOSEDATA_H_

#include "../math/Matrix4x4.hh"
#include "../math/SE3.hh"
#include "../math/Vector.hh"
#include "Ray.h"

#include <limits>

/**
 * Extrinsic camera data class
 */
template <typename T>
struct CameraPoseData {
	/// Rot/Trans/Scale transformation from world- to viewspace.
	SE3<T> View2World;

	/**
	 * Standard constructor. Initializes everything with invalid values ->
	 * triggers errors if used before init
	 */
	CameraPoseData() {
#ifndef NDEBUG
		View2World.Rotation[0] = std::numeric_limits<T>::signaling_NaN();
		View2World.Rotation[1] = std::numeric_limits<T>::signaling_NaN();
		View2World.Rotation[2] = std::numeric_limits<T>::signaling_NaN();
		View2World.Translation[0] = std::numeric_limits<T>::signaling_NaN();
		View2World.Translation[1] = std::numeric_limits<T>::signaling_NaN();
		View2World.Translation[2] = std::numeric_limits<T>::signaling_NaN();
#endif
	}
	/**
	 * Cast constructor.
	 * @param reference Reference object which might have another scalar type.
	 */
	template <typename oT>
	explicit CameraPoseData(const CameraPoseData<oT>& reference)
		: View2World(reference.View2World) {}

	/**
	 * Returns the position of the camera in world space.
	 * @returns Camera position in world space.
	 */
	base::Vector<T, 3> GetPosition() const;
	/**
	 * Sets the camera position in world space.
	 * @param worldspacePosition.
	 */
	void SetPosition(const base::Vector<T, 3>& worldspacePosition);
	/**
	 * Adjusts camera position and rotation such that it looks from a certain
	 * position to a target, being oriented according to an up vector.
	 * @param worldspacePosition Position of the camera.
	 * @param worldspaceTarget Target onto which the camera looks.
	 * @param worldspaceUp Up-vector along which the camera is oriented.
	 */
	void LookAt(const base::Vector<T, 3>& worldspacePosition,
				const base::Vector<T, 3>& worldspaceTarget,
				const base::Vector<T, 3>& worldspaceDown);
	/**
	 * Transforms a viewspace direction into a world space ray.
	 * @param viewspaceDirection Normalized view space direction.
	 * @returns Ray in world space.
	 */
	Ray<T> GetWorldspaceRay(const base::Vector<T, 3>& viewspaceDirection) const;
	/**
	 * Converts a world space direction into a view space direction.
	 * @param worldspaceDirection World space direction vector.
	 * @returns Viewspace direction.
	 */
	base::Vector<T, 3> GetViewspaceDirectionFromDir(
		const base::Vector<T, 3>& worldspaceDirection) const;
	/**
	 * Computes a viewspace direction vector from a world space point.
	 * @param worldspacePoint World space point.
	 * @returns Viewspace direction.
	 */
	base::Vector<T, 3> GetViewspaceDirectionFromPoint(
		const base::Vector<T, 3>& worldspacePoint) const;
	/**
	 * Transforms a world space point into a view space point.
	 * @param worldspacePoint World space point.
	 * @returns View space point.
	 */
	base::Vector<T, 3> GetViewspacePoint(
		const base::Vector<T, 3>& worldspacePoint) const;
};

template <typename T>
base::Vector<T, 3> CameraPoseData<T>::GetPosition() const {
	base::Vector<T, 3> result = View2World.Translation;

	return result;
}

template <typename T>
void CameraPoseData<T>::SetPosition(
	const base::Vector<T, 3>& worldspacePosition) {
	View2World.Translation = worldspacePosition;
}

template <typename T>
void CameraPoseData<T>::LookAt(const base::Vector<T, 3>& worldspacePosition,
							   const base::Vector<T, 3>& worldspaceTarget,
							   const base::Vector<T, 3>& worldspaceDown) {
	View2World.Translation = worldspacePosition;

	// Calculate orientation components
	base::Vector<T, 3> forward = worldspaceTarget - worldspacePosition;
	forward.normalize();
	base::Vector<T, 3> right = worldspaceDown % forward;
	right.normalize();
	base::Vector<T, 3> down = forward % right;
	down.normalize();

	// From Graphics Gems I, pg. 466
	T arg = (right[0] + down[1] + forward[2] - 1.0) / 2;
	if (arg > 1.0) {
		View2World.Rotation = base::Vector<T, 3>(0.0, 0.0, 0.0);
		return;
	}
	T angle = acos(arg);
	T t = sin(angle);
	if ((t * t) < std::numeric_limits<T>::epsilon()) {
		View2World.Rotation = base::Vector<T, 3>(0.0, 0.0, 0.0);
		return;
	}

	// t *= 2.0;
	View2World.Rotation[0] = (down[2] - forward[1]);   // / t;
	View2World.Rotation[1] = (forward[0] - right[2]);  // / t;
	View2World.Rotation[2] = (right[1] - down[0]);	   // / t;
	View2World.Rotation.normalize();

	View2World.Rotation *= angle;
}

template <typename T>
Ray<T> CameraPoseData<T>::GetWorldspaceRay(
	const base::Vector<T, 3>& viewspaceDirection) const {
	Ray<T> result;
	result.Direction = View2World.TransformDirection(viewspaceDirection);
	result.Origin = GetPosition();

	return result;
}

template <typename T>
base::Vector<T, 3> CameraPoseData<T>::GetViewspaceDirectionFromDir(
	const base::Vector<T, 3>& worldspaceDirection) const {
	return View2World.InverseTransformDirection(worldspaceDirection);
}

template <typename T>
base::Vector<T, 3> CameraPoseData<T>::GetViewspaceDirectionFromPoint(
	const base::Vector<T, 3>& worldspacePoint) const {
	base::Vector<T, 3> result =
		View2World.InverseTransformPoint(worldspacePoint);
	result.normalize();

	return result;
}

template <typename T>
inline base::Vector<T, 3> CameraPoseData<T>::GetViewspacePoint(
	const base::Vector<T, 3>& worldspacePoint) const {
	base::Vector<T, 3> result =
		View2World.InverseTransformPoint(worldspacePoint);

	return result;
}

typedef CameraPoseData<float> CameraPoseDataf;
typedef CameraPoseData<double> CameraPoseDatad;

#endif /* CAMERAPOSEDATA_H_ */
