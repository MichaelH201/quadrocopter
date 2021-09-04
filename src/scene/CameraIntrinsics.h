#ifndef SRC_SCENE_CAMERAINTRINSICS_H_
#define SRC_SCENE_CAMERAINTRINSICS_H_

#include "opencv2/opencv.hpp"
#include <iostream>

struct CameraIntrinsics {
	/// Size of the camera images in pixels.
	std::vector<int> ImageSize = std::vector<int>(2);
	/// Focal length in x/y direction in pixels.
	float FocalLength;
	/// Principal point in x/y direction in pixels.
	std::vector<float> PrincipalPoint = std::vector<float>(2);

	cv::Mat DistortionCoefficients;
	std::vector<cv::Mat> RotationVector;
	std::vector<cv::Mat> TranslationVector;

	/**
	 * Initializes all values with signaling NaNs to provoke errors if not
	 * initialized correctly later.
	 *
	 */
	CameraIntrinsics() {
		ImageSize[0] = std::numeric_limits<int>::signaling_NaN();
		ImageSize[1] = std::numeric_limits<int>::signaling_NaN();
		FocalLength = std::numeric_limits<float>::signaling_NaN();
		PrincipalPoint[0] = std::numeric_limits<float>::signaling_NaN();
		PrincipalPoint[1] = std::numeric_limits<float>::signaling_NaN();
	}

	std::string toString() {
	    return ("> Image Size: [" + std::to_string((int)ImageSize[0]) + ", " + std::to_string((int)ImageSize[1]) + "]" + "\n"
	    + "> Focal Length: " + std::to_string(FocalLength) + "\n"
	    + "> Principal Point: [" + std::to_string(PrincipalPoint[0]) + ", " + std::to_string(PrincipalPoint[1]) + "]");
	};
};

#endif // SRC_SCENE_CAMERAINTRINSICS_H_
