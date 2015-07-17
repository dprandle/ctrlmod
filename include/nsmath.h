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
	return static_cast<T>(static_cast<int>(n+0.5));
}
};

float clampf(float pVal, const float & pMin, const float & pMax);
double clamp(double pVal, const double & pMin, const double & pMax);
float fractf(const float & num);
double fract(const double & num);

float lerp(int low, int high, int middle);
float lerp(uint low, uint high, uint middle);
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
typedef NSVec2<char> cvec2;
typedef NSVec2<int> ivec2;
typedef NSVec2<unsigned char> ucvec2;
typedef NSVec2<uint> uivec2;
typedef NSVec2<float> fvec2;
typedef NSVec2<double> vec2;

typedef NSVec3<char> cvec3;
typedef NSVec3<int> ivec3;
typedef NSVec3<unsigned char> ucvec3;
typedef NSVec3<uint> uivec3;
typedef NSVec3<float> fvec3;
typedef NSVec3<double> vec3;

typedef NSVec4<char> cvec4;
typedef NSVec4<int> ivec4;
typedef NSVec4<unsigned char> ucvec4;
typedef NSVec4<uint> uivec4;
typedef NSVec4<float> fvec4;
typedef NSVec4<double> vec4;

typedef NSQuat<char> cquat;
typedef NSQuat<int> iquat;
typedef NSQuat<unsigned char> ucquat;
typedef NSQuat<uint> uiquat;
typedef NSQuat<float> fquat;
typedef NSQuat<double> quat;

typedef NSMat2<char> cmat2;
typedef NSMat2<int> imat2;
typedef NSMat2<unsigned char> ucmat2;
typedef NSMat2<uint> uimat2;
typedef NSMat2<float> fmat2;
typedef NSMat2<double> mat2;

typedef NSMat3<char> cmat3;
typedef NSMat3<int> imat3;
typedef NSMat3<unsigned char> ucmat3;
typedef NSMat3<uint> uimat3;
typedef NSMat3<float> fmat3;
typedef NSMat3<double> mat3;

typedef NSMat4<char> cmat4;
typedef NSMat4<unsigned char> ucmat4;
typedef NSMat4<int> imat4;
typedef NSMat4<uint> uimat4;
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
