#ifndef RAY_H_
#define RAY_H_

#include "opencv2/opencv.hpp"

/**
 * Ray class with origin and direction.
 */
template <typename T>
struct Ray {
	/// Origin of the ray.
	cv::Mat_<T> Origin;
	/// Direction of the ray.
	cv::Mat_<T> Direction;
};

typedef Ray<float> Rayf;
typedef Ray<double> Rayd;

#endif /* RAY_H_ */
