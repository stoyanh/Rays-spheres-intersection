#include "Utils.h"
#include <xmmintrin.h>
#include <limits>
#include <cmath>

using std::numeric_limits;

namespace Intersection
{
	IntersectionData intersectSingleSphere(const Ray& ray, const Sphere& sphere)
	{
		IntersectionData data;
		float A = 1.f, B = 0.f, C = 0.f;

		for(int i = 0; i < 3; ++i)
		{
			B += (ray.direction.coords[i] * ray.origin.coords[i] - ray.direction.coords[i] * sphere.center.coords[i]);
			C += (ray.origin.coords[i] * ray.origin.coords[i] -
				  2.f * ray.origin.coords[i] * sphere.center.coords[i] +
				  sphere.center.coords[i] * sphere.center.coords[i]);
		}
		C -= sphere.radius * sphere.radius;
		float discriminant = B * B - 4.f * C;
		if(discriminant < 0)
		{
			data.intersection = false;
			return data;
		}

		float squareRoot = sqrt(discriminant);
		float point1 = (-B - squareRoot) / 2.f;
		if(point1 > 0)
		{
			data.intersection = true;
			data.tIntersection = point1;
			return data;
		}

		data.tIntersection = (-B + squareRoot) / 2.f;
		return data;
	}

	IntersectionData intersectRaySpheres(const Ray& ray, const vector<int>& spheresIndices,
			const Spheres& spheres)
	{
		const int maxSpheresToCheck = 4;
		IntersectionData result;
		result.intersection = false;
		result.tIntersection = numeric_limits<float>::max();

		int remainder = spheresIndices.size() % maxSpheresToCheck;
		bool canUseSIMD = (remainder < spheresIndices.size());

		int nonSIMDStartPos = 0;

		if(canUseSIMD)
		{
			const int spheresToSIMDCheck = spheresIndices.size() - remainder;
			nonSIMDStartPos = spheresToSIMDCheck;
			//Vec4Float a = _mm_set1_ps(1.f); when rayDir is normalized a is 1
			Vec4Float b = _mm_set1_ps(0.f);
			Vec4Float c = b;
			Vec4Float D = c;

			Vec4Float centerCoords[3], radiuses;

			for(int i = 0; i < spheresToSIMDCheck; i += 4)
			{
				for(int j = 0; j < 3; ++j)
				{
					centerCoords[j] = _mm_set_ps(
							spheres.centerCoords[j][spheresIndices[i]], spheres.centerCoords[j][spheresIndices[i + 1]],
							spheres.centerCoords[j][spheresIndices[i + 2]], spheres.centerCoords[j][spheresIndices[i + 3]]
					);

					radiuses = _mm_set_ps(
							spheres.radiuses[spheresIndices[i]], spheres.radiuses[spheresIndices[i + 1]],
							spheres.radiuses[spheresIndices[i + 2]], spheres.radiuses[spheresIndices[i + 2]]
					);

					b += 2.f * ray.direction.coords[j] * (ray.origin.coords[j] - centerCoords[j]);
					c += (ray.origin.coords[j] - centerCoords[j]) * (ray.origin.coords[j] - centerCoords[j]);
				}
				D = b * b - 4.f * c;

				Vec4Float mask = _mm_cmpge_ps(D, _mm_set_ps1(0.f));
				Vec4Float squareRootD = _mm_sqrt_ps(D);
				D = _mm_and_ps(squareRootD, mask);

				Vec4Float t1, t2;
				t1 = _mm_or_ps((-b - squareRootD) * 0.5f, _mm_andnot_ps(mask, D));
				t2 = _mm_or_ps((-b + squareRootD) * 0.5f, _mm_andnot_ps(mask, D));

				float tRes = result.tIntersection;
				for(int j = 0; j < 4; ++j)
				{
					if(t1[j] >= 0 && t1[j] < tRes)
					{
						tRes = t1[j];
					}
					if(t2[j] >= 0 && t2[j] < tRes)
					{
						tRes = t2[j];
					}
				}

				if(tRes	< result.tIntersection)

					result.intersection = true;
					result.tIntersection = tRes;
				}
			}

			for(int i = nonSIMDStartPos; i < spheresIndices.size(); ++i)
			{
				IntersectionData data;
				int idx = spheresIndices[i];
				Sphere sphere;
				sphere.center.x = spheres.centerCoords[0][idx];
				sphere.center.y = spheres.centerCoords[1][idx];
				sphere.center.z = spheres.centerCoords[2][idx];
				sphere.radius = spheres.radiuses[idx];
				data = intersectSingleSphere(ray, sphere);

				if(data.intersection && data.tIntersection < result.tIntersection)
				{
					result = data;
				}
			}

			return result;
	}

}
