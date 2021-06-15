#ifndef RAY_H_
#define RAY_H_

#include "../math/Vector.hh"

/**
 * Ray class with origin and direction.
 */
template <typename T>
struct Ray {
	/// Origin of the ray.
	base::Vector<T, 3> Origin;
	/// Direction of the ray.
	base::Vector<T, 3> Direction;
};

typedef Ray<float> Rayf;
typedef Ray<double> Rayd;

#endif /* RAY_H_ */
