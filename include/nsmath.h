#ifndef NSMATH_H
#define NSMATH_H

#define PI 3.14159265359f
#define EPS 0.0001f

#include <exception>
#include <stdexcept>
#include <cstdlib>
#include <cmath>
#include <string>
#include <sstream>
#include <edglobal.h>
#include <vector>

namespace std
{
template<class T>
T round(const T & n)
{
	return static_cast<T>(static_cast<int32_t>(n+0.5));
}
};

float clampf(float pVal, const float & pMin, const float & pMax);
double clamp(double pVal, const double & pMin, const double & pMax);
float fractf(const float & num);
double fract(const double & num);

float lerp(int32_t low, int32_t high, int32_t middle);
float lerp(uint32_t low, uint32_t high, uint32_t middle);
float lerp(float low, float high, float middle);
float lerp(float low, float high, float middle);
double lerp(double low, double high, double middle);

template<class T>
T degrees(const T & val);

template<class T>
T radians(const T & val);

float RandomFloat(float pHigh = 1.0f, float pLow = 0.0f);

#include "nsmat4.h"

// Math typedefs
typedef NSVec2<uint8_t> cvec2;
typedef NSVec2<int32_t> ivec2;
typedef NSVec2<uint8_t> ucvec2;
typedef NSVec2<uint32_t> uivec2;
typedef NSVec2<float> fvec2;
typedef NSVec2<double> vec2;

typedef NSVec3<uint8_t> cvec3;
typedef NSVec3<int32_t> ivec3;
typedef NSVec3<uint8_t> ucvec3;
typedef NSVec3<uint32_t> uivec3;
typedef NSVec3<float> fvec3;
typedef NSVec3<double> vec3;

typedef NSVec4<uint8_t> cvec4;
typedef NSVec4<int32_t> ivec4;
typedef NSVec4<uint8_t> ucvec4;
typedef NSVec4<uint32_t> uivec4;
typedef NSVec4<float> fvec4;
typedef NSVec4<double> vec4;

typedef NSQuat<uint8_t> cquat;
typedef NSQuat<int32_t> iquat;
typedef NSQuat<uint8_t> ucquat;
typedef NSQuat<uint32_t> uiquat;
typedef NSQuat<float> fquat;
typedef NSQuat<double> quat;

typedef NSMat2<uint8_t> cmat2;
typedef NSMat2<int32_t> imat2;
typedef NSMat2<uint8_t> ucmat2;
typedef NSMat2<uint32_t> uimat2;
typedef NSMat2<float> fmat2;
typedef NSMat2<double> mat2;

typedef NSMat3<uint8_t> cmat3;
typedef NSMat3<int32_t> imat3;
typedef NSMat3<uint8_t> ucmat3;
typedef NSMat3<uint32_t> uimat3;
typedef NSMat3<float> fmat3;
typedef NSMat3<double> mat3;

typedef NSMat4<uint8_t> cmat4;
typedef NSMat4<uint8_t> ucmat4;
typedef NSMat4<int32_t> imat4;
typedef NSMat4<uint32_t> uimat4;
typedef NSMat4<float> fmat4;
typedef NSMat4<double> mat4;

template<class T>
T degrees(const T & val)
{
	return static_cast<T>(180 / PI) * val;
}

template<class T>
T radians(const T & val)
{
	return static_cast<T>(PI / 180) * val;
}

struct NSBoundingBox
{
	NSBoundingBox(
		const std::vector<fvec3> & pVertices = std::vector<fvec3>()
		);

	enum Face {
		None,
		Bottom,
		Top,
		Left,
		Right,
		Back,
		Front
	};

	void calculate(
		const std::vector<fvec3> & pVertices,
		const fmat4 & pTransform = fmat4()
		);

	fvec3 center(const Face & pFace = None);

	void clear();

	float dx();
	float dy();
	float dz();

	void extend(
		const std::vector<fvec3> & pVertices,
		const fmat4 & pTransform = fmat4()
		);

	void set(
		const fvec3 & pMin,
		const fvec3 pMax
		);

	float volume();

	fvec3 mMin;
	fvec3 mMax;
	fvec3 mVerts[8];

private:
	void _updateVerts();
};

#endif
