#ifndef SRC_SCENE_SE3_H_
#define SRC_SCENE_SE3_H_

#include "Matrix4x4.hh"
#include "Vector.hh"

#include <ceres/rotation.h>

/**
 * Function for composing two rodrigues rotation vectors to one that represents
 * the composed rotations of the input.
 * @param rotation1 Rotation that is performed first
 * @param rotation2 Rotation that is performed second
 * @returns Rotation that represends first rotation1, then rotation2. The
 * result's norm is always smaller than pi.
 */
template <typename T>
base::Vector<T, 3> composeRodrigues(const base::Vector<T, 3>& rotation1,
									const base::Vector<T, 3>& rotation2) {
	base::Vector<T, 4> qrot1;
	base::Vector<T, 4> qrot2;
	base::Vector<T, 4> qrotComb;

	ceres::AngleAxisToQuaternion((const T*)rotation1, (T*)qrot1);
	ceres::AngleAxisToQuaternion((const T*)rotation2, (T*)qrot2);
	ceres::QuaternionProduct((T*)qrot2, (T*)qrot1, (T*)qrotComb);

	base::Vector<T, 3> result;

	ceres::QuaternionToAngleAxis((T*)qrotComb, (T*)result);

	return result;
}

/**
 * Function composing two translations of camera poses. Treats the additional
 * rotation of the first translation due to the rotation of the second pose
 * which is applied later correctly.
 * @param translation1 Translation of the first pose.
 * @param translation2 Translation of the second pose.
 * @param rotation2 Rotation of the second pose which is applied onto
 * translation1 because the second pose is applied after the first translation.
 * @returns Composed translation corresponding to first applying pose1, then
 * pose2.
 */
template <typename T>
base::Vector<T, 3> composeTranslation(const base::Vector<T, 3>& translation1,
									  const base::Vector<T, 3>& translation2,
									  const base::Vector<T, 3>& rotation2) {
	base::Vector<T, 3> rotatedTranslation1;
	ceres::AngleAxisRotatePoint((const T*)rotation2, (const T*)translation1,
								(T*)rotatedTranslation1);

	base::Vector<T, 3> result = rotatedTranslation1 + translation2;

	return result;
}

template <typename T>
struct SE3 {
	base::Vector<T, 3> Rotation;
	base::Vector<T, 3> Translation;

	/**
	 * Standard constructor. Does not initialize anything.
	 */
	SE3() {}

	/**
	 * Init constructor. Initializes by attributes.
	 * @param rotation Rotation to be used.
	 * @param translation Translation to be used.
	 */
	SE3(const base::Vector<T, 3>& rotation,
		const base::Vector<T, 3>& translation)
		: Rotation(rotation), Translation(translation) {}

	/**
	 * Cast constructor.
	 * @param reference Reference object which might have another scalar type.
	 */
	template <typename oT>
	explicit SE3(const SE3<oT>& reference) {
		for (unsigned int i = 0; i < 3; i++)
			Rotation[i] = (T)reference.Rotation[i];
		for (unsigned int i = 0; i < 3; i++)
			Translation[i] = (T)reference.Translation[i];
	}

	/**
	 * Sets the transformation to identity (clears both vectors).
	 */
	void SetIdentity();

	/**
	 * Checks if the transformation is identity (all vectors are 0).
	 * @returns True if this is an identity transformation.
	 */
	bool IsIdentity() const;

	bool operator==(const SE3<T>& other);

	/**
	 * Transform the pose to a normalized representation which means that the
	 * Rodrigues Vector's norm should be between 0 and pi.
	 */
	void NormalizeRotation();
	/**
	 * Returns a relative pose that has the inverse effect of the original one.
	 * @returns Has the inverse effect of the original one.
	 */
	SE3<T> Inverse() const;
	/**
	 * Returns the rotation of the inverse transformation.
	 * @returns Scale of the inverse transformation.
	 */
	base::Vector<T, 3> InverseRotation() const;
	/**
	 * Returns the translation of the inverse transformation.
	 * @returns Translation of the inverse transformation.
	 */
	base::Vector<T, 3> InverseTranslation() const;
	/**
	 * Sets the translation of the SIM3, so that the inverse translation
	 * corresponds to the given value.
	 * @param invTranslation Inverse translation that should be matched.
	 */
	void SetInverseTranslation(const base::Vector<T, 3>& invTranslation) const;
	/**
	 * Applies the SE3 transformation rot(x) + t to a point x.
	 * @param in Input point.
	 * @returns Transformed point.
	 */
	base::Vector<T, 3> TransformPoint(const base::Vector<T, 3>& in) const;
	/**
	 * Applies the inverse transformation of TransformPoint.
	 * @param in Input point.
	 * @returns Transformed point.
	 */
	base::Vector<T, 3> InverseTransformPoint(
		const base::Vector<T, 3>& in) const;
	/**
	 * Applies the rotation transformation rot(x) to a point x.
	 * @param in Input point.
	 * @returns Transformed point.
	 */
	base::Vector<T, 3> TransformDirection(const base::Vector<T, 3>& in) const;
	/**
	 * Applies the inverse rotation of TransformDirection.
	 * @param in Input point.
	 * @returns Transformed point.
	 */
	base::Vector<T, 3> InverseTransformDirection(
		const base::Vector<T, 3>& in) const;
	/**
	 * Composition of two scaled relative poses.
	 * @param other Other pose to be composed with this one.
	 * @returns A scaled relative pose that has the same effect as applying
	 * the two inputs, first this one, then the other.
	 */
	SE3<T> operator*(const SE3<T>& other) const;

	/**
	 * Returns a matrix that applies the same transformation.
	 * @returns Transformation matrix that corresponds to the SIM3
	 */
	base::Matrix4x4<T> GetMatrix() const;
};

template <typename T>
inline void SE3<T>::SetIdentity() {
	Rotation.clear();
	Translation.clear();
}

template <typename T>
inline bool SE3<T>::IsIdentity() const {
	if (!Rotation.IsZero())
		return false;
	if (!Translation.IsZero())
		return false;
	return true;
}

template <typename T>
inline bool SE3<T>::operator==(const SE3<T>& other) {
	if (Rotation != other.Rotation)
		return false;
	if (Translation != other.Translation)
		return false;
	return true;
}

template <typename T>
void SE3<T>::NormalizeRotation() {
	while (Rotation.norm() > M_PI) {
		base::Vector<T, 3> rotationAxis = Rotation;
		rotationAxis.normalize();
		Rotation = Rotation - 2.0 * M_PI * rotationAxis;

		// Catch numerical problem of going from +PI to -PI
		if ((Rotation | rotationAxis) < 0.0)
			// Rotation now points in other direction than former axis -> 0
			// passed
			break;
	}
}

template <typename T>
inline SE3<T> SE3<T>::Inverse() const {
	SE3<T> result;

	result.Rotation = InverseRotation();
	result.Translation = InverseTranslation();

	return result;
}

template <typename T>
inline base::Vector<T, 3> SE3<T>::InverseRotation() const {
	base::Vector<T, 3> result = -Rotation;

	return result;
}

template <typename T>
inline base::Vector<T, 3> SE3<T>::InverseTranslation() const {
	base::Vector<T, 3> result;
	ceres::AngleAxisRotatePoint((const T*)(-Rotation), (T*)(-Translation),
								(T*)(result));

	return result;
}

template <typename T>
inline void SE3<T>::SetInverseTranslation(
	const base::Vector<T, 3>& invTranslation) const {
	// Do exactly the opposite of the inverse translation of SIM3.
	base::Vector<T, 3> scaledWorldspacePosition = invTranslation;
	ceres::AngleAxisRotatePoint((const T*)(&Rotation),
								(const T*)(-scaledWorldspacePosition),
								(T*)(&Translation));
}

template <typename T>
inline base::Vector<T, 3> SE3<T>::TransformPoint(
	const base::Vector<T, 3>& in) const {
	base::Vector<T, 3> result;
	ceres::AngleAxisRotatePoint((const T*)(Rotation), (const T*)(in),
								(T*)(result));
	result += Translation;
	return result;
}

template <typename T>
inline base::Vector<T, 3> SE3<T>::InverseTransformPoint(
	const base::Vector<T, 3>& in) const {
	base::Vector<T, 3> translatedScaled = (in - Translation);
	base::Vector<T, 3> result;
	ceres::AngleAxisRotatePoint((const T*)(-Rotation),
								(const T*)(translatedScaled), (T*)(result));

	return result;
}

template <typename T>
inline base::Vector<T, 3> SE3<T>::TransformDirection(
	const base::Vector<T, 3>& in) const {
	base::Vector<T, 3> result;
	ceres::AngleAxisRotatePoint((const T*)(Rotation), (const T*)(in),
								(T*)(result));
	return result;
}

template <typename T>
inline base::Vector<T, 3> SE3<T>::InverseTransformDirection(
	const base::Vector<T, 3>& in) const {
	base::Vector<T, 3> translatedScaled = in;
	base::Vector<T, 3> result;
	ceres::AngleAxisRotatePoint((const T*)(-Rotation),
								(const T*)(translatedScaled), (T*)(result));

	return result;
}

template <typename T>
inline SE3<T> SE3<T>::operator*(const SE3<T>& other) const {
	SE3<T> result;

	result.Rotation = composeRodrigues(Rotation, other.Rotation);

	base::Vector<T, 3> rotatedTranslation;
	ceres::AngleAxisRotatePoint((const T*)other.Rotation, (const T*)Translation,
								(T*)rotatedTranslation);
	base::Vector<T, 3> rotatedScaledTranslation = rotatedTranslation;
	result.Translation = rotatedScaledTranslation + other.Translation;

	return result;
}

template <typename T>
inline base::Matrix4x4<T> SE3<T>::GetMatrix() const {
	base::Matrix4x4<T> result;
	result.Identity();
	result.Rotate(-Rotation.norm() * 180.0 / M_PI, Rotation[0], Rotation[1],
				  Rotation[2]);
	result.ReplacePosition(Translation);
	return result;
}

#endif /* SRC_SCENE_SE3_H_ */
