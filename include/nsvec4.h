
#ifndef NSVEC4_H
#define NSVEC4_H

#include "nsvec3.h"

template <class T>
NSVec4<T> operator*(const int32_t & pLHS, const NSVec4<T> & pRHS);

template <class T>
NSVec4<T> operator*(const float & pLHS, const NSVec4<T> & pRHS);

template <class T>
NSVec4<T> operator*(const double & pLHS, const NSVec4<T> & pRHS);

template<class T>
NSVec4<T> operator/(const int32_t & pLHS, const NSVec4<T> & pRHS);

template <class T>
NSVec4<T> operator/(const float & pLHS, const NSVec4<T> & pRHS);

template <class T>
NSVec4<T> operator/(const double & pLHS, const NSVec4<T> & pRHS);

template <class T>
NSVec4<T> abs(const NSVec4<T> & pVec);

template <class T>
NSVec4<T> axisAngle(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool rads = false);

template <class T>
NSVec4<T> axisAngle(const NSQuat<T> & orientation, bool rads = false);

template <class T>
NSVec4<T> axisAngle(const NSMat3<T> & rotationMat3, bool rads = false);

template <class T>
NSVec4<T> axisAngle(const NSMat4<T> & transform, bool rads = false);

template <class T>
NSVec4<T> axisAngle(const NSVec3<T> & vec, const NSVec3<T> & toVec, bool rads = false);

template <class T>
NSVec4<T> ceil(const NSVec4<T> & pVec);

template <class T>
NSVec4<T> clamp(const NSVec4<T> & pVec, const T & pMin, const T & pMax);

template <class T>
T distance(const NSVec4<T> & lvec, const NSVec4<T> & rvec);

template <class T>
T dot(const NSVec4<T> & pLeft, const NSVec4<T> & pRight);

template <class T>
NSVec4<T> floor(const NSVec4<T> & pVec);

template <class T>
NSVec4<T> fract(const NSVec4<T> & vec);

template <class T>
T length(const NSVec4<T> & pVec);

template <class T, class T2>
NSVec4<T> lerp(const NSVec4<T> & lhs, const NSVec4<T> & rhs, T2 scalingFactor);

template <class T>
NSVec4<T> min(const NSVec4<T> & pLeft, const NSVec4<T> & pRight);

template <class T>
NSVec4<T> max(const NSVec4<T> & pLeft, const NSVec4<T> & pRight);

template <class T>
NSVec4<T> normalize(const NSVec4<T> & pRHS);

template <class T>
NSVec4<T> round(const NSVec4<T> & pVec);

template<class PUPer, class T>
void pup(PUPer & p, NSVec4<T> & v4);

template <class T>
struct NSVec4
{
	NSVec4(const NSVec4<T> & copy) : x(copy.x), y(copy.y), z(copy.z), w(copy.w) {}
	NSVec4(const T & val = static_cast<T>(0)) : x(val), y(val), z(val), w(val) {}
	NSVec4(const T & pX, const T & pY, const T & pZ = static_cast<T>(0), const T & pW = static_cast<T>(0)) : x(pX), y(pY), z(pZ), w(pW) {}
	NSVec4(const NSVec3<T> & xyz, const T & pW = static_cast<T>(1)) : x(xyz.x), y(xyz.y), z(xyz.z), w(pW) {}
	NSVec4(const T & pX, const NSVec3<T> & yzw) : x(pX), y(yzw.x), z(yzw.y), w(yzw.z) {}
	NSVec4(const NSVec2<T> & xy, const T & pZ = static_cast<T>(0), const T & pW = static_cast<T>(0)) : x(xy.x), y(xy.y), z(pZ), w(pW) {}
	NSVec4(const T & pX, const NSVec2<T> & yz, const T & pW = static_cast<T>(0)) : x(pX), y(yz.x), z(yz.z), w(pW) {}
	NSVec4(const T & pX, const T & pY, const NSVec2<T> & zw) : x(pX), y(pY), z(zw.x), w(zw.y) {}

	NSVec3<T> & abs()
	{
		x = static_cast<T>(std::abs(x));
		y = static_cast<T>(std::abs(y));
		z = static_cast<T>(std::abs(z));
		w = static_cast<T>(std::abs(w));
		return *this;
	}

	NSVec4<T> & axisAngleFrom(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool rads = false)
	{
		return axisAngleFrom(NSQuat<T>().from(euler, order, rads), rads);
	}

	NSVec4<T> & axisAngleFrom(const NSQuat<T> & orientation, bool rads = false)
	{
		// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
		w = static_cast<T>(2 * std::acos(orientation.w));

		if (!rads)
			w = degrees(w);

		T den = static_cast<T>(std::sqrt(1 - orientation.w*orientation.w));
		if (den < EPS)
		{
			x = orientation.x;
			y = orientation.y;
			z = orientation.z;
		}
		else
		{
			x = orientation.x / den;
			y = orientation.y / den;
			z = orientation.z / den;
		}
		return *this;
	}

	NSVec4<T> & axisAngleFrom(const NSMat3<T> & rotationMat3, bool rads = false)
	{
		return axisAngleFrom(NSQuat<T>().from(rotationMat3), rads);
	}

	NSVec4<T> & axisAngleFrom(const NSMat4<T> & transform, bool rads = false)
	{
		return axisAngleFrom(NSQuat<T>().from(transform), rads);
	}

	NSVec4<T> & axisAngleFrom(const NSVec3<T> & vec, const NSVec3<T> & toVec, bool rads = false)
	{
		return axisAngleFrom(NSQuat<T>().from(vec, toVec), rads);
	}

	NSVec4<T> & ceil()
	{
		x = static_cast<T>(std::ceil(x));
		y = static_cast<T>(std::ceil(y));
		z = static_cast<T>(std::ceil(z));
		w = static_cast<T>(std::ceil(w));
		return *this;
	}

	NSVec4<T> & clamp(const T & min = static_cast<T>(0), const T & max = static_cast<T>(0))
	{
		if (x < min)
			x = min;
		if (y < min)
			y = min;
		if (z < min)
			z = min;
		if (w < min)
			w = min;
		if (x > max)
			x = max;
		if (y > max)
			y = max;
		if (z > max)
			z = max;
		if (w > max)
			w = max;
		return *this;
	}

	T distanceTo(const NSVec4<T> & pVec) const
	{
		return ((pVec - *this).length());
	}

	NSVec4<T> & floor()
	{
		x = static_cast<T>(std::floor(x));
		y = static_cast<T>(std::floor(y));
		z = static_cast<T>(std::floor(z));
		w = static_cast<T>(std::floor(w));
		return *this;
	}

	NSVec4<T> & fract()
	{
		x -= static_cast<T>(std::floor(x));
		y -= static_cast<T>(std::floor(y));
		z -= static_cast<T>(std::floor(z));
		w -= static_cast<T>(std::floor(w));
		return *this;
	}

	T length() const
	{
		return static_cast<T>(sqrt(x*x + y*y + z*z + w*w));
	}

	T lengthSq() const
	{
		return x*x + y*y + z*z + w*w;
	}

	template<class T2 >
	NSVec4<T> & lerp(const NSVec4<T> & vec, const T2 & scalingFactor)
	{
		x += static_cast<T>((vec.x - x)*scalingFactor);
		y += static_cast<T>((vec.y - y)*scalingFactor);
		z += static_cast<T>((vec.z - z)*scalingFactor);
		w += static_cast<T>((vec.w - w)*scalingFactor);
		return *this;
	}

	T min()
	{
		T m1 = xyz().min();
		if (m1 < w)
			return m1;
		return w;
	}

	NSVec4<T> & minimize(const NSVec4<T> & rhs)
	{
		if (x < rhs.x)
			x = rhs.x;
		if (y < rhs.y)
			y = rhs.y;
		if (z < rhs.z)
			z = rhs.z;
		if (w < rhs.w)
			w = rhs.w;
		return *this;
	}

	T max()
	{
		T m1 = xyz().max();
		if (m1 > w)
			return m1;
		return w;
	}

	NSVec4<T> & maximize(const NSVec4<T> & rhs)
	{
		if (x > rhs.x)
			x = rhs.x;
		if (y > rhs.y)
			y = rhs.y;
		if (z > rhs.z)
			z = rhs.z;
		if (w > rhs.w)
			w = rhs.w;
		return *this;
	}

	NSVec4<T> & normalize()
	{
		T l = length();
		if (l == static_cast<T>(0))
			return *this;
		return *this *= static_cast<T>(1) / l;
	}

	NSVec4<T> & round()
	{
		x = static_cast<T>(std::round(x));
		y = static_cast<T>(std::round(y));
		z = static_cast<T>(std::round(z));
		w = static_cast<T>(std::round(w));
		return *this;
	}

	NSVec4<T> & roundToZero()
	{
		if (::abs(x) < EPS)
			x = 0;
		if (::abs(y) < EPS)
			y = 0;
		if (::abs(z) < EPS)
			z = 0;
		if (::abs(w) < EPS)
			w = 0;
		return *this;
	}

	NSVec3<T> & scalingFrom(const NSMat3<T> & transform)
	{
		x = transform[0].length();
		y = transform[1].length();
		z = transform[2].length();
		w = static_cast<T>(1);
		return *this;
	}

	NSVec3<T> & scalingFrom(const NSMat4<T> & transform)
	{
		x = sqrt(transform[0][0] * transform[0][0] + transform[0][1] * transform[0][1] + transform[0][2] * transform[0][2]);
		y = sqrt(transform[1][0] * transform[1][0] + transform[1][1] * transform[1][1] + transform[1][2] * transform[1][2]);
		z = sqrt(transform[2][0] * transform[2][0] + transform[2][1] * transform[2][1] + transform[2][2] * transform[2][2]);
		w = static_cast<T>(1);
		return *this;
	}

	NSVec4<T> & set(const T & pVal)
	{
		x = y = z = w = pVal;
		return *this;
	}

	NSVec4<T> & set(const T & pX, const T & pY, const T & pZ, const T & pW)
	{
		x = pX; y = pY; z = pZ; w = pW;
		return *this;
	}

	NSVec4<T> & set(const NSVec3<T> & xyz, const T & pW)
	{
		x = xyz.x; y = xyz.y; z = xyz.z; w = pW;
		return *this;
	}

	NSVec4<T> & set(const T & pX, const NSVec3<T> & yzw)
	{
		x = pX; y = yzw.x; z = yzw.y; w = yzw.z;
		return *this;
	}

	NSVec4<T> & set(const NSVec2<T> & xy, const T & pZ, const T & pW)
	{
		x = xy.x; y = xy.y; z = pZ; w = pW;
		return *this;
	}

	NSVec4<T> & set(const T & pX, const NSVec2<T> & yz, const T & pW)
	{
		x = pX; y = yz.x; z = yz.y; w = pW;
		return *this;
	}

	NSVec4<T> & set(const T & pX, const T & pY, const NSVec2<T> & zw)
	{
		x = pX; y = pY; z = zw.x; w = zw.y;
		return *this;
	}

	NSVec4<T> & setLength(const T & len)
	{
		T l = length();

		if (l == static_cast<T>(0))
			return *this;

		T mult = len / l;
		(*this) *= mult;
		return *this;
	}

	std::string toString()
	{
		std::ostringstream ss;
		ss << "[" << x << " " << y << " " << z << " " << w << "]";
		return ss.str();
	}

	NSVec4<T> & translationFrom(const NSMat4<T> & transform)
	{
		*this = transform(3);
		w = static_cast<T>(1);
		return *this;
	}

	// overloaded operators
	NSVec4<T> operator+(const NSVec4<T> & rhs) const
	{
		return NSVec4<T>(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
	}

	NSVec4<T> operator-(const NSVec4<T> & rhs) const
	{
		return NSVec4<T>(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
	}

	T operator*(const NSVec4<T> & rhs) const // dot product
	{
		return x*rhs.x + y*rhs.y + z*rhs.z + w*rhs.w;
	}

	NSMat4<T> operator^(const NSVec4<T> & pRHS) const
	{
		NSMat4<T> ret;
		ret[0] = x * pRHS;
		ret[1] = y * pRHS;
		ret[2] = z * pRHS;
		ret[3] = w * pRHS;
		return ret;
	}

	NSVec4<T> operator%(const NSVec4<T> & rhs) const // component wise scalar product
	{
		return NSVec4<T>(x*rhs.x, y*rhs.y, z*rhs.z, w*rhs.w);
	}

	NSVec4<T> operator/(const NSVec4<T> & rhs) const
	{
		return NSVec4<T>(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w);
	}

	NSVec4<T> operator*(const T & rhs) const
	{
		return NSVec4<T>(x * rhs, y * rhs, z * rhs, w * rhs);
	}

	NSVec4<T> operator/(const T & rhs) const
	{
		return NSVec4<T>(x / rhs, y / rhs, z / rhs, w / rhs);
	}

	NSVec4<T> & operator=(const NSVec4<T> & rhs)
	{
		if (this == &rhs)
			return *this;
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
		return *this;
	}

	NSVec4<T> operator++(int32_t)
	{
		NSVec4<T> ret(*this);
		++(*this);
		return ret;
	}

	NSVec4<T> operator--(int32_t)
	{
		NSVec4<T> ret(*this);
		--(*this);
		return ret;
	}

	NSVec4<T> & operator++()
	{
		++x; ++y; ++z; ++w;
		return *this;
	}

	NSVec4<T> & operator--()
	{
		--x; --y; --z; --w;
		return *this;
	}

	NSVec4<T> & operator+=(const NSVec4<T> & rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w;
		return *this;
	}

	NSVec4<T> & operator-=(const NSVec4<T> & rhs)
	{
		x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w;
		return *this;
	}

	NSVec4<T> & operator%=(const NSVec4<T> & rhs)
	{
		x *= rhs.x; y *= rhs.y; z *= rhs.z; w *= rhs.w;
		return *this;
	}

	NSVec4<T> & operator/=(const NSVec4<T> & rhs)
	{
		x /= rhs.x; y /= rhs.y; z /= rhs.z; w /= rhs.w;
		return *this;
	}

	NSVec4<T> & operator*=(const T & rhs)
	{
		x *= rhs; y *= rhs; z *= rhs; w *= rhs;
		return *this;
	}

	NSVec4<T> & operator/=(const T & rhs)
	{
		x /= rhs; y /= rhs; z /= rhs; w /= rhs;
		return *this;
	}

	bool operator==(const NSVec4<T> & rhs) const
	{
		return ((x == rhs.x) && (y == rhs.y) && (z == rhs.z) && (w == rhs.w));
	}

	bool operator!=(const NSVec4<T> & rhs) const
	{
		return !(*this == rhs);
	}

	bool operator==(const T & rhs) const
	{
		return ((x == rhs) && (y == rhs) && (z == rhs) && (w == rhs));
	}

	bool operator!=(const T & rhs) const
	{
		return !(*this == rhs);
	}

	const T & operator[](const uint32_t & pVal) const
	{
		if (pVal > 3)
			throw(std::out_of_range("vec4 index out of range"));
		return data[pVal];
	}

	T & operator[](const uint32_t & pVal)
	{
		if (pVal > 3)
			throw(std::out_of_range("vec4 index out of range"));
		return data[pVal];
	}

	// Vec4 Swizzles
	inline NSVec4<T> xxxx() const { return NSVec4<T>(x, x, x, x); }
	inline NSVec4<T> xxxy() const { return NSVec4<T>(x, x, x, y); }
	inline NSVec4<T> xxxz() const { return NSVec4<T>(x, x, x, z); }
	inline NSVec4<T> xxxw() const { return NSVec4<T>(x, x, x, w); }
	inline NSVec4<T> xxyx() const { return NSVec4<T>(x, x, y, x); }
	inline NSVec4<T> xxyy() const { return NSVec4<T>(x, x, y, y); }
	inline NSVec4<T> xxyz() const { return NSVec4<T>(x, x, y, z); }
	inline NSVec4<T> xxyw() const { return NSVec4<T>(x, x, y, w); }
	inline NSVec4<T> xxzx() const { return NSVec4<T>(x, x, z, x); }
	inline NSVec4<T> xxzy() const { return NSVec4<T>(x, x, z, y); }
	inline NSVec4<T> xxzz() const { return NSVec4<T>(x, x, z, z); }
	inline NSVec4<T> xxzw() const { return NSVec4<T>(x, x, z, w); }
	inline NSVec4<T> xxwx() const { return NSVec4<T>(x, x, w, x); }
	inline NSVec4<T> xxwy() const { return NSVec4<T>(x, x, w, y); }
	inline NSVec4<T> xxwz() const { return NSVec4<T>(x, x, w, z); }
	inline NSVec4<T> xxww() const { return NSVec4<T>(x, x, w, w); }
	inline NSVec4<T> xyxx() const { return NSVec4<T>(x, y, x, x); }
	inline NSVec4<T> xyxy() const { return NSVec4<T>(x, y, x, y); }
	inline NSVec4<T> xyxz() const { return NSVec4<T>(x, y, x, z); }
	inline NSVec4<T> xyxw() const { return NSVec4<T>(x, y, x, w); }
	inline NSVec4<T> xyyx() const { return NSVec4<T>(x, y, y, x); }
	inline NSVec4<T> xyyy() const { return NSVec4<T>(x, y, y, y); }
	inline NSVec4<T> xyyz() const { return NSVec4<T>(x, y, y, z); }
	inline NSVec4<T> xyyw() const { return NSVec4<T>(x, y, y, w); }
	inline NSVec4<T> xywx() const { return NSVec4<T>(x, y, w, x); }
	inline NSVec4<T> xywy() const { return NSVec4<T>(x, y, w, y); }
	inline NSVec4<T> xywz() const { return NSVec4<T>(x, y, w, z); }
	inline NSVec4<T> xyww() const { return NSVec4<T>(x, y, w, w); }
	inline NSVec4<T> xzxx() const { return NSVec4<T>(x, z, x, x); }
	inline NSVec4<T> xzxy() const { return NSVec4<T>(x, z, x, y); }
	inline NSVec4<T> xzxz() const { return NSVec4<T>(x, z, x, z); }
	inline NSVec4<T> xzxw() const { return NSVec4<T>(x, z, x, w); }
	inline NSVec4<T> xzyx() const { return NSVec4<T>(x, z, y, x); }
	inline NSVec4<T> xzyy() const { return NSVec4<T>(x, z, y, y); }
	inline NSVec4<T> xzyz() const { return NSVec4<T>(x, z, y, z); }
	inline NSVec4<T> xzyw() const { return NSVec4<T>(x, z, y, w); }
	inline NSVec4<T> xzzx() const { return NSVec4<T>(x, z, z, x); }
	inline NSVec4<T> xzzy() const { return NSVec4<T>(x, z, z, y); }
	inline NSVec4<T> xzzz() const { return NSVec4<T>(x, z, z, z); }
	inline NSVec4<T> xzzw() const { return NSVec4<T>(x, z, z, w); }
	inline NSVec4<T> xzwx() const { return NSVec4<T>(x, z, w, x); }
	inline NSVec4<T> xzwy() const { return NSVec4<T>(x, z, w, y); }
	inline NSVec4<T> xzwz() const { return NSVec4<T>(x, z, w, z); }
	inline NSVec4<T> xzww() const { return NSVec4<T>(x, z, w, w); }
	inline NSVec4<T> xwxx() const { return NSVec4<T>(x, w, x, x); }
	inline NSVec4<T> xwxy() const { return NSVec4<T>(x, w, x, y); }
	inline NSVec4<T> xwxz() const { return NSVec4<T>(x, w, x, z); }
	inline NSVec4<T> xwxw() const { return NSVec4<T>(x, w, x, w); }
	inline NSVec4<T> xwyx() const { return NSVec4<T>(x, w, y, x); }
	inline NSVec4<T> xwyy() const { return NSVec4<T>(x, w, y, y); }
	inline NSVec4<T> xwyz() const { return NSVec4<T>(x, w, y, z); }
	inline NSVec4<T> xwyw() const { return NSVec4<T>(x, w, y, w); }
	inline NSVec4<T> xwzx() const { return NSVec4<T>(x, w, z, x); }
	inline NSVec4<T> xwzy() const { return NSVec4<T>(x, w, z, y); }
	inline NSVec4<T> xwzz() const { return NSVec4<T>(x, w, z, z); }
	inline NSVec4<T> xwzw() const { return NSVec4<T>(x, w, z, w); }
	inline NSVec4<T> xwwx() const { return NSVec4<T>(x, w, w, x); }
	inline NSVec4<T> xwwy() const { return NSVec4<T>(x, w, w, y); }
	inline NSVec4<T> xwwz() const { return NSVec4<T>(x, w, w, z); }
	inline NSVec4<T> xwww() const { return NSVec4<T>(x, w, w, w); }
	inline NSVec4<T> yxxx() const { return NSVec4<T>(y, x, x, x); }
	inline NSVec4<T> yxxy() const { return NSVec4<T>(y, x, x, y); }
	inline NSVec4<T> yxxz() const { return NSVec4<T>(y, x, x, z); }
	inline NSVec4<T> yxxw() const { return NSVec4<T>(y, x, x, w); }
	inline NSVec4<T> yxyx() const { return NSVec4<T>(y, x, y, x); }
	inline NSVec4<T> yxyy() const { return NSVec4<T>(y, x, y, y); }
	inline NSVec4<T> yxyz() const { return NSVec4<T>(y, x, y, z); }
	inline NSVec4<T> yxyw() const { return NSVec4<T>(y, x, y, w); }
	inline NSVec4<T> yxzx() const { return NSVec4<T>(y, x, z, x); }
	inline NSVec4<T> yxzy() const { return NSVec4<T>(y, x, z, y); }
	inline NSVec4<T> yxzz() const { return NSVec4<T>(y, x, z, z); }
	inline NSVec4<T> yxzw() const { return NSVec4<T>(y, x, z, w); }
	inline NSVec4<T> yxwx() const { return NSVec4<T>(y, x, w, x); }
	inline NSVec4<T> yxwy() const { return NSVec4<T>(y, x, w, y); }
	inline NSVec4<T> yxwz() const { return NSVec4<T>(y, x, w, z); }
	inline NSVec4<T> yxww() const { return NSVec4<T>(y, x, w, w); }
	inline NSVec4<T> yyxx() const { return NSVec4<T>(y, y, x, x); }
	inline NSVec4<T> yyxy() const { return NSVec4<T>(y, y, x, y); }
	inline NSVec4<T> yyxz() const { return NSVec4<T>(y, y, x, z); }
	inline NSVec4<T> yyxw() const { return NSVec4<T>(y, y, x, w); }
	inline NSVec4<T> yyyx() const { return NSVec4<T>(y, y, y, x); }
	inline NSVec4<T> yyyy() const { return NSVec4<T>(y, y, y, y); }
	inline NSVec4<T> yyyz() const { return NSVec4<T>(y, y, y, z); }
	inline NSVec4<T> yyyw() const { return NSVec4<T>(y, y, y, w); }
	inline NSVec4<T> yyzx() const { return NSVec4<T>(y, y, z, x); }
	inline NSVec4<T> yyzy() const { return NSVec4<T>(y, y, z, y); }
	inline NSVec4<T> yyzz() const { return NSVec4<T>(y, y, z, z); }
	inline NSVec4<T> yyzw() const { return NSVec4<T>(y, y, z, w); }
	inline NSVec4<T> yywx() const { return NSVec4<T>(y, y, w, x); }
	inline NSVec4<T> yywy() const { return NSVec4<T>(y, y, w, y); }
	inline NSVec4<T> yywz() const { return NSVec4<T>(y, y, w, z); }
	inline NSVec4<T> yyww() const { return NSVec4<T>(y, y, w, w); }
	inline NSVec4<T> yzxx() const { return NSVec4<T>(y, z, x, x); }
	inline NSVec4<T> yzxy() const { return NSVec4<T>(y, z, x, y); }
	inline NSVec4<T> yzxz() const { return NSVec4<T>(y, z, x, z); }
	inline NSVec4<T> yzxw() const { return NSVec4<T>(y, z, x, w); }
	inline NSVec4<T> yzyx() const { return NSVec4<T>(y, z, y, x); }
	inline NSVec4<T> yzyy() const { return NSVec4<T>(y, z, y, y); }
	inline NSVec4<T> yzyz() const { return NSVec4<T>(y, z, y, z); }
	inline NSVec4<T> yzyw() const { return NSVec4<T>(y, z, y, w); }
	inline NSVec4<T> yzzx() const { return NSVec4<T>(y, z, z, x); }
	inline NSVec4<T> yzzy() const { return NSVec4<T>(y, z, z, y); }
	inline NSVec4<T> yzzz() const { return NSVec4<T>(y, z, z, z); }
	inline NSVec4<T> yzzw() const { return NSVec4<T>(y, z, z, w); }
	inline NSVec4<T> yzwx() const { return NSVec4<T>(y, z, w, x); }
	inline NSVec4<T> yzwy() const { return NSVec4<T>(y, z, w, y); }
	inline NSVec4<T> yzwz() const { return NSVec4<T>(y, z, w, z); }
	inline NSVec4<T> yzww() const { return NSVec4<T>(y, z, w, w); }
	inline NSVec4<T> ywxx() const { return NSVec4<T>(y, w, x, x); }
	inline NSVec4<T> ywxy() const { return NSVec4<T>(y, w, x, y); }
	inline NSVec4<T> ywxz() const { return NSVec4<T>(y, w, x, z); }
	inline NSVec4<T> ywxw() const { return NSVec4<T>(y, w, x, w); }
	inline NSVec4<T> ywyx() const { return NSVec4<T>(y, w, y, x); }
	inline NSVec4<T> ywyy() const { return NSVec4<T>(y, w, y, y); }
	inline NSVec4<T> ywyz() const { return NSVec4<T>(y, w, y, z); }
	inline NSVec4<T> ywyw() const { return NSVec4<T>(y, w, y, w); }
	inline NSVec4<T> ywzx() const { return NSVec4<T>(y, w, z, x); }
	inline NSVec4<T> ywzy() const { return NSVec4<T>(y, w, z, y); }
	inline NSVec4<T> ywzz() const { return NSVec4<T>(y, w, z, z); }
	inline NSVec4<T> ywzw() const { return NSVec4<T>(y, w, z, w); }
	inline NSVec4<T> ywwx() const { return NSVec4<T>(y, w, w, x); }
	inline NSVec4<T> ywwy() const { return NSVec4<T>(y, w, w, y); }
	inline NSVec4<T> ywwz() const { return NSVec4<T>(y, w, w, z); }
	inline NSVec4<T> ywww() const { return NSVec4<T>(y, w, w, w); }
	inline NSVec4<T> zxxx() const { return NSVec4<T>(z, x, x, x); }
	inline NSVec4<T> zxxy() const { return NSVec4<T>(z, x, x, y); }
	inline NSVec4<T> zxxz() const { return NSVec4<T>(z, x, x, z); }
	inline NSVec4<T> zxxw() const { return NSVec4<T>(z, x, x, w); }
	inline NSVec4<T> zxyx() const { return NSVec4<T>(z, x, y, x); }
	inline NSVec4<T> zxyy() const { return NSVec4<T>(z, x, y, y); }
	inline NSVec4<T> zxyz() const { return NSVec4<T>(z, x, y, z); }
	inline NSVec4<T> zxyw() const { return NSVec4<T>(z, x, y, w); }
	inline NSVec4<T> zxzx() const { return NSVec4<T>(z, x, z, x); }
	inline NSVec4<T> zxzy() const { return NSVec4<T>(z, x, z, y); }
	inline NSVec4<T> zxzz() const { return NSVec4<T>(z, x, z, z); }
	inline NSVec4<T> zxzw() const { return NSVec4<T>(z, x, z, w); }
	inline NSVec4<T> zxwx() const { return NSVec4<T>(z, x, w, x); }
	inline NSVec4<T> zxwy() const { return NSVec4<T>(z, x, w, y); }
	inline NSVec4<T> zxwz() const { return NSVec4<T>(z, x, w, z); }
	inline NSVec4<T> zxww() const { return NSVec4<T>(z, x, w, w); }
	inline NSVec4<T> zyxx() const { return NSVec4<T>(z, y, x, x); }
	inline NSVec4<T> zyxy() const { return NSVec4<T>(z, y, x, y); }
	inline NSVec4<T> zyxz() const { return NSVec4<T>(z, y, x, z); }
	inline NSVec4<T> zyxw() const { return NSVec4<T>(z, y, x, w); }
	inline NSVec4<T> zyyx() const { return NSVec4<T>(z, y, y, x); }
	inline NSVec4<T> zyyy() const { return NSVec4<T>(z, y, y, y); }
	inline NSVec4<T> zyyz() const { return NSVec4<T>(z, y, y, z); }
	inline NSVec4<T> zyyw() const { return NSVec4<T>(z, y, y, w); }
	inline NSVec4<T> zyzx() const { return NSVec4<T>(z, y, z, x); }
	inline NSVec4<T> zyzy() const { return NSVec4<T>(z, y, z, y); }
	inline NSVec4<T> zyzz() const { return NSVec4<T>(z, y, z, z); }
	inline NSVec4<T> zyzw() const { return NSVec4<T>(z, y, z, w); }
	inline NSVec4<T> zywx() const { return NSVec4<T>(z, y, w, x); }
	inline NSVec4<T> zywy() const { return NSVec4<T>(z, y, w, y); }
	inline NSVec4<T> zywz() const { return NSVec4<T>(z, y, w, z); }
	inline NSVec4<T> zyww() const { return NSVec4<T>(z, y, w, w); }
	inline NSVec4<T> zzxx() const { return NSVec4<T>(z, z, x, x); }
	inline NSVec4<T> zzxy() const { return NSVec4<T>(z, z, x, y); }
	inline NSVec4<T> zzxz() const { return NSVec4<T>(z, z, x, z); }
	inline NSVec4<T> zzxw() const { return NSVec4<T>(z, z, x, w); }
	inline NSVec4<T> zzyx() const { return NSVec4<T>(z, z, y, x); }
	inline NSVec4<T> zzyy() const { return NSVec4<T>(z, z, y, y); }
	inline NSVec4<T> zzyz() const { return NSVec4<T>(z, z, y, z); }
	inline NSVec4<T> zzyw() const { return NSVec4<T>(z, z, y, w); }
	inline NSVec4<T> zzzx() const { return NSVec4<T>(z, z, z, x); }
	inline NSVec4<T> zzzy() const { return NSVec4<T>(z, z, z, y); }
	inline NSVec4<T> zzzz() const { return NSVec4<T>(z, z, z, z); }
	inline NSVec4<T> zzzw() const { return NSVec4<T>(z, z, z, w); }
	inline NSVec4<T> zzwx() const { return NSVec4<T>(z, z, w, x); }
	inline NSVec4<T> zzwy() const { return NSVec4<T>(z, z, w, y); }
	inline NSVec4<T> zzwz() const { return NSVec4<T>(z, z, w, z); }
	inline NSVec4<T> zzww() const { return NSVec4<T>(z, z, w, w); }
	inline NSVec4<T> zwxx() const { return NSVec4<T>(z, w, x, x); }
	inline NSVec4<T> zwxy() const { return NSVec4<T>(z, w, x, y); }
	inline NSVec4<T> zwxz() const { return NSVec4<T>(z, w, x, z); }
	inline NSVec4<T> zwxw() const { return NSVec4<T>(z, w, x, w); }
	inline NSVec4<T> zwyx() const { return NSVec4<T>(z, w, y, x); }
	inline NSVec4<T> zwyy() const { return NSVec4<T>(z, w, y, y); }
	inline NSVec4<T> zwyz() const { return NSVec4<T>(z, w, y, z); }
	inline NSVec4<T> zwyw() const { return NSVec4<T>(z, w, y, w); }
	inline NSVec4<T> zwzx() const { return NSVec4<T>(z, w, z, x); }
	inline NSVec4<T> zwzy() const { return NSVec4<T>(z, w, z, y); }
	inline NSVec4<T> zwzz() const { return NSVec4<T>(z, w, z, z); }
	inline NSVec4<T> zwzw() const { return NSVec4<T>(z, w, z, w); }
	inline NSVec4<T> zwwx() const { return NSVec4<T>(z, w, w, x); }
	inline NSVec4<T> zwwy() const { return NSVec4<T>(z, w, w, y); }
	inline NSVec4<T> zwwz() const { return NSVec4<T>(z, w, w, z); }
	inline NSVec4<T> zwww() const { return NSVec4<T>(z, w, w, w); }
	inline NSVec4<T> wxxx() const { return NSVec4<T>(w, x, x, x); }
	inline NSVec4<T> wxxy() const { return NSVec4<T>(w, x, x, y); }
	inline NSVec4<T> wxxz() const { return NSVec4<T>(w, x, x, z); }
	inline NSVec4<T> wxxw() const { return NSVec4<T>(w, x, x, w); }
	inline NSVec4<T> wxyx() const { return NSVec4<T>(w, x, y, x); }
	inline NSVec4<T> wxyy() const { return NSVec4<T>(w, x, y, y); }
	inline NSVec4<T> wxyz() const { return NSVec4<T>(w, x, y, z); }
	inline NSVec4<T> wxyw() const { return NSVec4<T>(w, x, y, w); }
	inline NSVec4<T> wxzx() const { return NSVec4<T>(w, x, z, x); }
	inline NSVec4<T> wxzy() const { return NSVec4<T>(w, x, z, y); }
	inline NSVec4<T> wxzz() const { return NSVec4<T>(w, x, z, z); }
	inline NSVec4<T> wxzw() const { return NSVec4<T>(w, x, z, w); }
	inline NSVec4<T> wxwx() const { return NSVec4<T>(w, x, w, x); }
	inline NSVec4<T> wxwy() const { return NSVec4<T>(w, x, w, y); }
	inline NSVec4<T> wxwz() const { return NSVec4<T>(w, x, w, z); }
	inline NSVec4<T> wxww() const { return NSVec4<T>(w, x, w, w); }
	inline NSVec4<T> wyxx() const { return NSVec4<T>(w, y, x, x); }
	inline NSVec4<T> wyxy() const { return NSVec4<T>(w, y, x, y); }
	inline NSVec4<T> wyxz() const { return NSVec4<T>(w, y, x, z); }
	inline NSVec4<T> wyxw() const { return NSVec4<T>(w, y, x, w); }
	inline NSVec4<T> wyyx() const { return NSVec4<T>(w, y, y, x); }
	inline NSVec4<T> wyyy() const { return NSVec4<T>(w, y, y, y); }
	inline NSVec4<T> wyyz() const { return NSVec4<T>(w, y, y, z); }
	inline NSVec4<T> wyyw() const { return NSVec4<T>(w, y, y, w); }
	inline NSVec4<T> wyzx() const { return NSVec4<T>(w, y, z, x); }
	inline NSVec4<T> wyzy() const { return NSVec4<T>(w, y, z, y); }
	inline NSVec4<T> wyzz() const { return NSVec4<T>(w, y, z, z); }
	inline NSVec4<T> wyzw() const { return NSVec4<T>(w, y, z, w); }
	inline NSVec4<T> wywx() const { return NSVec4<T>(w, y, w, x); }
	inline NSVec4<T> wywy() const { return NSVec4<T>(w, y, w, y); }
	inline NSVec4<T> wywz() const { return NSVec4<T>(w, y, w, z); }
	inline NSVec4<T> wyww() const { return NSVec4<T>(w, y, w, w); }
	inline NSVec4<T> wzxx() const { return NSVec4<T>(w, z, x, x); }
	inline NSVec4<T> wzxy() const { return NSVec4<T>(w, z, x, y); }
	inline NSVec4<T> wzxz() const { return NSVec4<T>(w, z, x, z); }
	inline NSVec4<T> wzxw() const { return NSVec4<T>(w, z, x, w); }
	inline NSVec4<T> wzyx() const { return NSVec4<T>(w, z, y, x); }
	inline NSVec4<T> wzyy() const { return NSVec4<T>(w, z, y, y); }
	inline NSVec4<T> wzyz() const { return NSVec4<T>(w, z, y, z); }
	inline NSVec4<T> wzyw() const { return NSVec4<T>(w, z, y, w); }
	inline NSVec4<T> wzzx() const { return NSVec4<T>(w, z, z, x); }
	inline NSVec4<T> wzzy() const { return NSVec4<T>(w, z, z, y); }
	inline NSVec4<T> wzzz() const { return NSVec4<T>(w, z, z, z); }
	inline NSVec4<T> wzzw() const { return NSVec4<T>(w, z, z, w); }
	inline NSVec4<T> wzwx() const { return NSVec4<T>(w, z, w, x); }
	inline NSVec4<T> wzwy() const { return NSVec4<T>(w, z, w, y); }
	inline NSVec4<T> wzwz() const { return NSVec4<T>(w, z, w, z); }
	inline NSVec4<T> wzww() const { return NSVec4<T>(w, z, w, w); }
	inline NSVec4<T> wwxx() const { return NSVec4<T>(w, w, x, x); }
	inline NSVec4<T> wwxy() const { return NSVec4<T>(w, w, x, y); }
	inline NSVec4<T> wwxz() const { return NSVec4<T>(w, w, x, z); }
	inline NSVec4<T> wwxw() const { return NSVec4<T>(w, w, x, w); }
	inline NSVec4<T> wwyx() const { return NSVec4<T>(w, w, y, x); }
	inline NSVec4<T> wwyy() const { return NSVec4<T>(w, w, y, y); }
	inline NSVec4<T> wwyz() const { return NSVec4<T>(w, w, y, z); }
	inline NSVec4<T> wwyw() const { return NSVec4<T>(w, w, y, w); }
	inline NSVec4<T> wwzx() const { return NSVec4<T>(w, w, z, x); }
	inline NSVec4<T> wwzy() const { return NSVec4<T>(w, w, z, y); }
	inline NSVec4<T> wwzz() const { return NSVec4<T>(w, w, z, z); }
	inline NSVec4<T> wwzw() const { return NSVec4<T>(w, w, z, w); }
	inline NSVec4<T> wwwx() const { return NSVec4<T>(w, w, w, x); }
	inline NSVec4<T> wwwy() const { return NSVec4<T>(w, w, w, y); }
	inline NSVec4<T> wwwz() const { return NSVec4<T>(w, w, w, z); }
	inline NSVec4<T> wwww() const { return NSVec4<T>(w, w, w, w); }

	inline NSVec4<T> rrrr() const { return NSVec4<T>(r, r, r, r); }
	inline NSVec4<T> rrrg() const { return NSVec4<T>(r, r, r, g); }
	inline NSVec4<T> rrrb() const { return NSVec4<T>(r, r, r, b); }
	inline NSVec4<T> rrra() const { return NSVec4<T>(r, r, r, a); }
	inline NSVec4<T> rrgr() const { return NSVec4<T>(r, r, g, r); }
	inline NSVec4<T> rrgg() const { return NSVec4<T>(r, r, g, g); }
	inline NSVec4<T> rrgb() const { return NSVec4<T>(r, r, g, b); }
	inline NSVec4<T> rrga() const { return NSVec4<T>(r, r, g, a); }
	inline NSVec4<T> rrbr() const { return NSVec4<T>(r, r, b, r); }
	inline NSVec4<T> rrbg() const { return NSVec4<T>(r, r, b, g); }
	inline NSVec4<T> rrbb() const { return NSVec4<T>(r, r, b, b); }
	inline NSVec4<T> rrba() const { return NSVec4<T>(r, r, b, a); }
	inline NSVec4<T> rrar() const { return NSVec4<T>(r, r, a, r); }
	inline NSVec4<T> rrag() const { return NSVec4<T>(r, r, a, g); }
	inline NSVec4<T> rrab() const { return NSVec4<T>(r, r, a, b); }
	inline NSVec4<T> rraa() const { return NSVec4<T>(r, r, a, a); }
	inline NSVec4<T> rgrr() const { return NSVec4<T>(r, g, r, r); }
	inline NSVec4<T> rgrg() const { return NSVec4<T>(r, g, r, g); }
	inline NSVec4<T> rgrb() const { return NSVec4<T>(r, g, r, b); }
	inline NSVec4<T> rgra() const { return NSVec4<T>(r, g, r, a); }
	inline NSVec4<T> rggr() const { return NSVec4<T>(r, g, g, r); }
	inline NSVec4<T> rggg() const { return NSVec4<T>(r, g, g, g); }
	inline NSVec4<T> rggb() const { return NSVec4<T>(r, g, g, b); }
	inline NSVec4<T> rgga() const { return NSVec4<T>(r, g, g, a); }
	inline NSVec4<T> rgbr() const { return NSVec4<T>(r, g, b, r); }
	inline NSVec4<T> rgbg() const { return NSVec4<T>(r, g, b, g); }
	inline NSVec4<T> rgbb() const { return NSVec4<T>(r, g, b, b); }
	inline NSVec4<T> rgar() const { return NSVec4<T>(r, g, a, r); }
	inline NSVec4<T> rgag() const { return NSVec4<T>(r, g, a, g); }
	inline NSVec4<T> rgab() const { return NSVec4<T>(r, g, a, b); }
	inline NSVec4<T> rgaa() const { return NSVec4<T>(r, g, a, a); }
	inline NSVec4<T> rbrr() const { return NSVec4<T>(r, b, r, r); }
	inline NSVec4<T> rbrg() const { return NSVec4<T>(r, b, r, g); }
	inline NSVec4<T> rbrb() const { return NSVec4<T>(r, b, r, b); }
	inline NSVec4<T> rbra() const { return NSVec4<T>(r, b, r, a); }
	inline NSVec4<T> rbgr() const { return NSVec4<T>(r, b, g, r); }
	inline NSVec4<T> rbgg() const { return NSVec4<T>(r, b, g, g); }
	inline NSVec4<T> rbgb() const { return NSVec4<T>(r, b, g, b); }
	inline NSVec4<T> rbga() const { return NSVec4<T>(r, b, g, a); }
	inline NSVec4<T> rbbr() const { return NSVec4<T>(r, b, b, r); }
	inline NSVec4<T> rbbg() const { return NSVec4<T>(r, b, b, g); }
	inline NSVec4<T> rbbb() const { return NSVec4<T>(r, b, b, b); }
	inline NSVec4<T> rbba() const { return NSVec4<T>(r, b, b, a); }
	inline NSVec4<T> rbar() const { return NSVec4<T>(r, b, a, r); }
	inline NSVec4<T> rbag() const { return NSVec4<T>(r, b, a, g); }
	inline NSVec4<T> rbab() const { return NSVec4<T>(r, b, a, b); }
	inline NSVec4<T> rbaa() const { return NSVec4<T>(r, b, a, a); }
	inline NSVec4<T> rarr() const { return NSVec4<T>(r, a, r, r); }
	inline NSVec4<T> rarg() const { return NSVec4<T>(r, a, r, g); }
	inline NSVec4<T> rarb() const { return NSVec4<T>(r, a, r, b); }
	inline NSVec4<T> rara() const { return NSVec4<T>(r, a, r, a); }
	inline NSVec4<T> ragr() const { return NSVec4<T>(r, a, g, r); }
	inline NSVec4<T> ragg() const { return NSVec4<T>(r, a, g, g); }
	inline NSVec4<T> ragb() const { return NSVec4<T>(r, a, g, b); }
	inline NSVec4<T> raga() const { return NSVec4<T>(r, a, g, a); }
	inline NSVec4<T> rabr() const { return NSVec4<T>(r, a, b, r); }
	inline NSVec4<T> rabg() const { return NSVec4<T>(r, a, b, g); }
	inline NSVec4<T> rabb() const { return NSVec4<T>(r, a, b, b); }
	inline NSVec4<T> raba() const { return NSVec4<T>(r, a, b, a); }
	inline NSVec4<T> raar() const { return NSVec4<T>(r, a, a, r); }
	inline NSVec4<T> raag() const { return NSVec4<T>(r, a, a, g); }
	inline NSVec4<T> raab() const { return NSVec4<T>(r, a, a, b); }
	inline NSVec4<T> raaa() const { return NSVec4<T>(r, a, a, a); }
	inline NSVec4<T> grrr() const { return NSVec4<T>(g, r, r, r); }
	inline NSVec4<T> grrg() const { return NSVec4<T>(g, r, r, g); }
	inline NSVec4<T> grrb() const { return NSVec4<T>(g, r, r, b); }
	inline NSVec4<T> grra() const { return NSVec4<T>(g, r, r, a); }
	inline NSVec4<T> grgr() const { return NSVec4<T>(g, r, g, r); }
	inline NSVec4<T> grgg() const { return NSVec4<T>(g, r, g, g); }
	inline NSVec4<T> grgb() const { return NSVec4<T>(g, r, g, b); }
	inline NSVec4<T> grga() const { return NSVec4<T>(g, r, g, a); }
	inline NSVec4<T> grbr() const { return NSVec4<T>(g, r, b, r); }
	inline NSVec4<T> grbg() const { return NSVec4<T>(g, r, b, g); }
	inline NSVec4<T> grbb() const { return NSVec4<T>(g, r, b, b); }
	inline NSVec4<T> grba() const { return NSVec4<T>(g, r, b, a); }
	inline NSVec4<T> grar() const { return NSVec4<T>(g, r, a, r); }
	inline NSVec4<T> grag() const { return NSVec4<T>(g, r, a, g); }
	inline NSVec4<T> grab() const { return NSVec4<T>(g, r, a, b); }
	inline NSVec4<T> graa() const { return NSVec4<T>(g, r, a, a); }
	inline NSVec4<T> ggrr() const { return NSVec4<T>(g, g, r, r); }
	inline NSVec4<T> ggrg() const { return NSVec4<T>(g, g, r, g); }
	inline NSVec4<T> ggrb() const { return NSVec4<T>(g, g, r, b); }
	inline NSVec4<T> ggra() const { return NSVec4<T>(g, g, r, a); }
	inline NSVec4<T> gggr() const { return NSVec4<T>(g, g, g, r); }
	inline NSVec4<T> gggg() const { return NSVec4<T>(g, g, g, g); }
	inline NSVec4<T> gggb() const { return NSVec4<T>(g, g, g, b); }
	inline NSVec4<T> ggga() const { return NSVec4<T>(g, g, g, a); }
	inline NSVec4<T> ggbr() const { return NSVec4<T>(g, g, b, r); }
	inline NSVec4<T> ggbg() const { return NSVec4<T>(g, g, b, g); }
	inline NSVec4<T> ggbb() const { return NSVec4<T>(g, g, b, b); }
	inline NSVec4<T> ggba() const { return NSVec4<T>(g, g, b, a); }
	inline NSVec4<T> ggar() const { return NSVec4<T>(g, g, a, r); }
	inline NSVec4<T> ggag() const { return NSVec4<T>(g, g, a, g); }
	inline NSVec4<T> ggab() const { return NSVec4<T>(g, g, a, b); }
	inline NSVec4<T> ggaa() const { return NSVec4<T>(g, g, a, a); }
	inline NSVec4<T> gbrr() const { return NSVec4<T>(g, b, r, r); }
	inline NSVec4<T> gbrg() const { return NSVec4<T>(g, b, r, g); }
	inline NSVec4<T> gbrb() const { return NSVec4<T>(g, b, r, b); }
	inline NSVec4<T> gbra() const { return NSVec4<T>(g, b, r, a); }
	inline NSVec4<T> gbgr() const { return NSVec4<T>(g, b, g, r); }
	inline NSVec4<T> gbgg() const { return NSVec4<T>(g, b, g, g); }
	inline NSVec4<T> gbgb() const { return NSVec4<T>(g, b, g, b); }
	inline NSVec4<T> gbga() const { return NSVec4<T>(g, b, g, a); }
	inline NSVec4<T> gbbr() const { return NSVec4<T>(g, b, b, r); }
	inline NSVec4<T> gbbg() const { return NSVec4<T>(g, b, b, g); }
	inline NSVec4<T> gbbb() const { return NSVec4<T>(g, b, b, b); }
	inline NSVec4<T> gbba() const { return NSVec4<T>(g, b, b, a); }
	inline NSVec4<T> gbar() const { return NSVec4<T>(g, b, a, r); }
	inline NSVec4<T> gbag() const { return NSVec4<T>(g, b, a, g); }
	inline NSVec4<T> gbab() const { return NSVec4<T>(g, b, a, b); }
	inline NSVec4<T> gbaa() const { return NSVec4<T>(g, b, a, a); }
	inline NSVec4<T> garr() const { return NSVec4<T>(g, a, r, r); }
	inline NSVec4<T> garg() const { return NSVec4<T>(g, a, r, g); }
	inline NSVec4<T> garb() const { return NSVec4<T>(g, a, r, b); }
	inline NSVec4<T> gara() const { return NSVec4<T>(g, a, r, a); }
	inline NSVec4<T> gagr() const { return NSVec4<T>(g, a, g, r); }
	inline NSVec4<T> gagg() const { return NSVec4<T>(g, a, g, g); }
	inline NSVec4<T> gagb() const { return NSVec4<T>(g, a, g, b); }
	inline NSVec4<T> gaga() const { return NSVec4<T>(g, a, g, a); }
	inline NSVec4<T> gabr() const { return NSVec4<T>(g, a, b, r); }
	inline NSVec4<T> gabg() const { return NSVec4<T>(g, a, b, g); }
	inline NSVec4<T> gabb() const { return NSVec4<T>(g, a, b, b); }
	inline NSVec4<T> gaba() const { return NSVec4<T>(g, a, b, a); }
	inline NSVec4<T> gaar() const { return NSVec4<T>(g, a, a, r); }
	inline NSVec4<T> gaag() const { return NSVec4<T>(g, a, a, g); }
	inline NSVec4<T> gaab() const { return NSVec4<T>(g, a, a, b); }
	inline NSVec4<T> gaaa() const { return NSVec4<T>(g, a, a, a); }
	inline NSVec4<T> brrr() const { return NSVec4<T>(b, r, r, r); }
	inline NSVec4<T> brrg() const { return NSVec4<T>(b, r, r, g); }
	inline NSVec4<T> brrb() const { return NSVec4<T>(b, r, r, b); }
	inline NSVec4<T> brra() const { return NSVec4<T>(b, r, r, a); }
	inline NSVec4<T> brgr() const { return NSVec4<T>(b, r, g, r); }
	inline NSVec4<T> brgg() const { return NSVec4<T>(b, r, g, g); }
	inline NSVec4<T> brgb() const { return NSVec4<T>(b, r, g, b); }
	inline NSVec4<T> brga() const { return NSVec4<T>(b, r, g, a); }
	inline NSVec4<T> brbr() const { return NSVec4<T>(b, r, b, r); }
	inline NSVec4<T> brbg() const { return NSVec4<T>(b, r, b, g); }
	inline NSVec4<T> brbb() const { return NSVec4<T>(b, r, b, b); }
	inline NSVec4<T> brba() const { return NSVec4<T>(b, r, b, a); }
	inline NSVec4<T> brar() const { return NSVec4<T>(b, r, a, r); }
	inline NSVec4<T> brag() const { return NSVec4<T>(b, r, a, g); }
	inline NSVec4<T> brab() const { return NSVec4<T>(b, r, a, b); }
	inline NSVec4<T> braa() const { return NSVec4<T>(b, r, a, a); }
	inline NSVec4<T> bgrr() const { return NSVec4<T>(b, g, r, r); }
	inline NSVec4<T> bgrg() const { return NSVec4<T>(b, g, r, g); }
	inline NSVec4<T> bgrb() const { return NSVec4<T>(b, g, r, b); }
	inline NSVec4<T> bgra() const { return NSVec4<T>(b, g, r, a); }
	inline NSVec4<T> bggr() const { return NSVec4<T>(b, g, g, r); }
	inline NSVec4<T> bggg() const { return NSVec4<T>(b, g, g, g); }
	inline NSVec4<T> bggb() const { return NSVec4<T>(b, g, g, b); }
	inline NSVec4<T> bgga() const { return NSVec4<T>(b, g, g, a); }
	inline NSVec4<T> bgbr() const { return NSVec4<T>(b, g, b, r); }
	inline NSVec4<T> bgbg() const { return NSVec4<T>(b, g, b, g); }
	inline NSVec4<T> bgbb() const { return NSVec4<T>(b, g, b, b); }
	inline NSVec4<T> bgba() const { return NSVec4<T>(b, g, b, a); }
	inline NSVec4<T> bgar() const { return NSVec4<T>(b, g, a, r); }
	inline NSVec4<T> bgag() const { return NSVec4<T>(b, g, a, g); }
	inline NSVec4<T> bgab() const { return NSVec4<T>(b, g, a, b); }
	inline NSVec4<T> bgaa() const { return NSVec4<T>(b, g, a, a); }
	inline NSVec4<T> bbrr() const { return NSVec4<T>(b, b, r, r); }
	inline NSVec4<T> bbrg() const { return NSVec4<T>(b, b, r, g); }
	inline NSVec4<T> bbrb() const { return NSVec4<T>(b, b, r, b); }
	inline NSVec4<T> bbra() const { return NSVec4<T>(b, b, r, a); }
	inline NSVec4<T> bbgr() const { return NSVec4<T>(b, b, g, r); }
	inline NSVec4<T> bbgg() const { return NSVec4<T>(b, b, g, g); }
	inline NSVec4<T> bbgb() const { return NSVec4<T>(b, b, g, b); }
	inline NSVec4<T> bbga() const { return NSVec4<T>(b, b, g, a); }
	inline NSVec4<T> bbbr() const { return NSVec4<T>(b, b, b, r); }
	inline NSVec4<T> bbbg() const { return NSVec4<T>(b, b, b, g); }
	inline NSVec4<T> bbbb() const { return NSVec4<T>(b, b, b, b); }
	inline NSVec4<T> bbba() const { return NSVec4<T>(b, b, b, a); }
	inline NSVec4<T> bbar() const { return NSVec4<T>(b, b, a, r); }
	inline NSVec4<T> bbag() const { return NSVec4<T>(b, b, a, g); }
	inline NSVec4<T> bbab() const { return NSVec4<T>(b, b, a, b); }
	inline NSVec4<T> bbaa() const { return NSVec4<T>(b, b, a, a); }
	inline NSVec4<T> barr() const { return NSVec4<T>(b, a, r, r); }
	inline NSVec4<T> barg() const { return NSVec4<T>(b, a, r, g); }
	inline NSVec4<T> barb() const { return NSVec4<T>(b, a, r, b); }
	inline NSVec4<T> bara() const { return NSVec4<T>(b, a, r, a); }
	inline NSVec4<T> bagr() const { return NSVec4<T>(b, a, g, r); }
	inline NSVec4<T> bagg() const { return NSVec4<T>(b, a, g, g); }
	inline NSVec4<T> bagb() const { return NSVec4<T>(b, a, g, b); }
	inline NSVec4<T> baga() const { return NSVec4<T>(b, a, g, a); }
	inline NSVec4<T> babr() const { return NSVec4<T>(b, a, b, r); }
	inline NSVec4<T> babg() const { return NSVec4<T>(b, a, b, g); }
	inline NSVec4<T> babb() const { return NSVec4<T>(b, a, b, b); }
	inline NSVec4<T> baba() const { return NSVec4<T>(b, a, b, a); }
	inline NSVec4<T> baar() const { return NSVec4<T>(b, a, a, r); }
	inline NSVec4<T> baag() const { return NSVec4<T>(b, a, a, g); }
	inline NSVec4<T> baab() const { return NSVec4<T>(b, a, a, b); }
	inline NSVec4<T> baaa() const { return NSVec4<T>(b, a, a, a); }
	inline NSVec4<T> arrr() const { return NSVec4<T>(a, r, r, r); }
	inline NSVec4<T> arrg() const { return NSVec4<T>(a, r, r, g); }
	inline NSVec4<T> arrb() const { return NSVec4<T>(a, r, r, b); }
	inline NSVec4<T> arra() const { return NSVec4<T>(a, r, r, a); }
	inline NSVec4<T> argr() const { return NSVec4<T>(a, r, g, r); }
	inline NSVec4<T> argg() const { return NSVec4<T>(a, r, g, g); }
	inline NSVec4<T> argb() const { return NSVec4<T>(a, r, g, b); }
	inline NSVec4<T> arga() const { return NSVec4<T>(a, r, g, a); }
	inline NSVec4<T> arbr() const { return NSVec4<T>(a, r, b, r); }
	inline NSVec4<T> arbg() const { return NSVec4<T>(a, r, b, g); }
	inline NSVec4<T> arbb() const { return NSVec4<T>(a, r, b, b); }
	inline NSVec4<T> arba() const { return NSVec4<T>(a, r, b, a); }
	inline NSVec4<T> arar() const { return NSVec4<T>(a, r, a, r); }
	inline NSVec4<T> arag() const { return NSVec4<T>(a, r, a, g); }
	inline NSVec4<T> arab() const { return NSVec4<T>(a, r, a, b); }
	inline NSVec4<T> araa() const { return NSVec4<T>(a, r, a, a); }
	inline NSVec4<T> agrr() const { return NSVec4<T>(a, g, r, r); }
	inline NSVec4<T> agrg() const { return NSVec4<T>(a, g, r, g); }
	inline NSVec4<T> agrb() const { return NSVec4<T>(a, g, r, b); }
	inline NSVec4<T> agra() const { return NSVec4<T>(a, g, r, a); }
	inline NSVec4<T> aggr() const { return NSVec4<T>(a, g, g, r); }
	inline NSVec4<T> aggg() const { return NSVec4<T>(a, g, g, g); }
	inline NSVec4<T> aggb() const { return NSVec4<T>(a, g, g, b); }
	inline NSVec4<T> agga() const { return NSVec4<T>(a, g, g, a); }
	inline NSVec4<T> agbr() const { return NSVec4<T>(a, g, b, r); }
	inline NSVec4<T> agbg() const { return NSVec4<T>(a, g, b, g); }
	inline NSVec4<T> agbb() const { return NSVec4<T>(a, g, b, b); }
	inline NSVec4<T> agba() const { return NSVec4<T>(a, g, b, a); }
	inline NSVec4<T> agar() const { return NSVec4<T>(a, g, a, r); }
	inline NSVec4<T> agag() const { return NSVec4<T>(a, g, a, g); }
	inline NSVec4<T> agab() const { return NSVec4<T>(a, g, a, b); }
	inline NSVec4<T> agaa() const { return NSVec4<T>(a, g, a, a); }
	inline NSVec4<T> abrr() const { return NSVec4<T>(a, b, r, r); }
	inline NSVec4<T> abrg() const { return NSVec4<T>(a, b, r, g); }
	inline NSVec4<T> abrb() const { return NSVec4<T>(a, b, r, b); }
	inline NSVec4<T> abra() const { return NSVec4<T>(a, b, r, a); }
	inline NSVec4<T> abgr() const { return NSVec4<T>(a, b, g, r); }
	inline NSVec4<T> abgg() const { return NSVec4<T>(a, b, g, g); }
	inline NSVec4<T> abgb() const { return NSVec4<T>(a, b, g, b); }
	inline NSVec4<T> abga() const { return NSVec4<T>(a, b, g, a); }
	inline NSVec4<T> abbr() const { return NSVec4<T>(a, b, b, r); }
	inline NSVec4<T> abbg() const { return NSVec4<T>(a, b, b, g); }
	inline NSVec4<T> abbb() const { return NSVec4<T>(a, b, b, b); }
	inline NSVec4<T> abba() const { return NSVec4<T>(a, b, b, a); }
	inline NSVec4<T> abar() const { return NSVec4<T>(a, b, a, r); }
	inline NSVec4<T> abag() const { return NSVec4<T>(a, b, a, g); }
	inline NSVec4<T> abab() const { return NSVec4<T>(a, b, a, b); }
	inline NSVec4<T> abaa() const { return NSVec4<T>(a, b, a, a); }
	inline NSVec4<T> aarr() const { return NSVec4<T>(a, a, r, r); }
	inline NSVec4<T> aarg() const { return NSVec4<T>(a, a, r, g); }
	inline NSVec4<T> aarb() const { return NSVec4<T>(a, a, r, b); }
	inline NSVec4<T> aara() const { return NSVec4<T>(a, a, r, a); }
	inline NSVec4<T> aagr() const { return NSVec4<T>(a, a, g, r); }
	inline NSVec4<T> aagg() const { return NSVec4<T>(a, a, g, g); }
	inline NSVec4<T> aagb() const { return NSVec4<T>(a, a, g, b); }
	inline NSVec4<T> aaga() const { return NSVec4<T>(a, a, g, a); }
	inline NSVec4<T> aabr() const { return NSVec4<T>(a, a, b, r); }
	inline NSVec4<T> aabg() const { return NSVec4<T>(a, a, b, g); }
	inline NSVec4<T> aabb() const { return NSVec4<T>(a, a, b, b); }
	inline NSVec4<T> aaba() const { return NSVec4<T>(a, a, b, a); }
	inline NSVec4<T> aaar() const { return NSVec4<T>(a, a, a, r); }
	inline NSVec4<T> aaag() const { return NSVec4<T>(a, a, a, g); }
	inline NSVec4<T> aaab() const { return NSVec4<T>(a, a, a, b); }
	inline NSVec4<T> aaaa() const { return NSVec4<T>(a, a, a, a); }

	inline NSVec4<T> ssss() const { return NSVec4<T>(s, s, s, s); }
	inline NSVec4<T> ssst() const { return NSVec4<T>(s, s, s, t); }
	inline NSVec4<T> sssp() const { return NSVec4<T>(s, s, s, p); }
	inline NSVec4<T> sssq() const { return NSVec4<T>(s, s, s, q); }
	inline NSVec4<T> ssts() const { return NSVec4<T>(s, s, t, s); }
	inline NSVec4<T> sstt() const { return NSVec4<T>(s, s, t, t); }
	inline NSVec4<T> sstp() const { return NSVec4<T>(s, s, t, p); }
	inline NSVec4<T> sstq() const { return NSVec4<T>(s, s, t, q); }
	inline NSVec4<T> ssps() const { return NSVec4<T>(s, s, p, s); }
	inline NSVec4<T> sspt() const { return NSVec4<T>(s, s, p, t); }
	inline NSVec4<T> sspp() const { return NSVec4<T>(s, s, p, p); }
	inline NSVec4<T> sspq() const { return NSVec4<T>(s, s, p, q); }
	inline NSVec4<T> ssqs() const { return NSVec4<T>(s, s, q, s); }
	inline NSVec4<T> ssqt() const { return NSVec4<T>(s, s, q, t); }
	inline NSVec4<T> ssqp() const { return NSVec4<T>(s, s, q, p); }
	inline NSVec4<T> ssqq() const { return NSVec4<T>(s, s, q, q); }
	inline NSVec4<T> stss() const { return NSVec4<T>(s, t, s, s); }
	inline NSVec4<T> stst() const { return NSVec4<T>(s, t, s, t); }
	inline NSVec4<T> stsp() const { return NSVec4<T>(s, t, s, p); }
	inline NSVec4<T> stsq() const { return NSVec4<T>(s, t, s, q); }
	inline NSVec4<T> stts() const { return NSVec4<T>(s, t, t, s); }
	inline NSVec4<T> sttt() const { return NSVec4<T>(s, t, t, t); }
	inline NSVec4<T> sttp() const { return NSVec4<T>(s, t, t, p); }
	inline NSVec4<T> sttq() const { return NSVec4<T>(s, t, t, q); }
	inline NSVec4<T> stps() const { return NSVec4<T>(s, t, p, s); }
	inline NSVec4<T> stpt() const { return NSVec4<T>(s, t, p, t); }
	inline NSVec4<T> stpp() const { return NSVec4<T>(s, t, p, p); }
	inline NSVec4<T> stqs() const { return NSVec4<T>(s, t, q, s); }
	inline NSVec4<T> stqt() const { return NSVec4<T>(s, t, q, t); }
	inline NSVec4<T> stqp() const { return NSVec4<T>(s, t, q, p); }
	inline NSVec4<T> stqq() const { return NSVec4<T>(s, t, q, q); }
	inline NSVec4<T> spss() const { return NSVec4<T>(s, p, s, s); }
	inline NSVec4<T> spst() const { return NSVec4<T>(s, p, s, t); }
	inline NSVec4<T> spsp() const { return NSVec4<T>(s, p, s, p); }
	inline NSVec4<T> spsq() const { return NSVec4<T>(s, p, s, q); }
	inline NSVec4<T> spts() const { return NSVec4<T>(s, p, t, s); }
	inline NSVec4<T> sptt() const { return NSVec4<T>(s, p, t, t); }
	inline NSVec4<T> sptp() const { return NSVec4<T>(s, p, t, p); }
	inline NSVec4<T> sptq() const { return NSVec4<T>(s, p, t, q); }
	inline NSVec4<T> spps() const { return NSVec4<T>(s, p, p, s); }
	inline NSVec4<T> sppt() const { return NSVec4<T>(s, p, p, t); }
	inline NSVec4<T> sppp() const { return NSVec4<T>(s, p, p, p); }
	inline NSVec4<T> sppq() const { return NSVec4<T>(s, p, p, q); }
	inline NSVec4<T> spqs() const { return NSVec4<T>(s, p, q, s); }
	inline NSVec4<T> spqt() const { return NSVec4<T>(s, p, q, t); }
	inline NSVec4<T> spqp() const { return NSVec4<T>(s, p, q, p); }
	inline NSVec4<T> spqq() const { return NSVec4<T>(s, p, q, q); }
	inline NSVec4<T> sqss() const { return NSVec4<T>(s, q, s, s); }
	inline NSVec4<T> sqst() const { return NSVec4<T>(s, q, s, t); }
	inline NSVec4<T> sqsp() const { return NSVec4<T>(s, q, s, p); }
	inline NSVec4<T> sqsq() const { return NSVec4<T>(s, q, s, q); }
	inline NSVec4<T> sqts() const { return NSVec4<T>(s, q, t, s); }
	inline NSVec4<T> sqtt() const { return NSVec4<T>(s, q, t, t); }
	inline NSVec4<T> sqtp() const { return NSVec4<T>(s, q, t, p); }
	inline NSVec4<T> sqtq() const { return NSVec4<T>(s, q, t, q); }
	inline NSVec4<T> sqps() const { return NSVec4<T>(s, q, p, s); }
	inline NSVec4<T> sqpt() const { return NSVec4<T>(s, q, p, t); }
	inline NSVec4<T> sqpp() const { return NSVec4<T>(s, q, p, p); }
	inline NSVec4<T> sqpq() const { return NSVec4<T>(s, q, p, q); }
	inline NSVec4<T> sqqs() const { return NSVec4<T>(s, q, q, s); }
	inline NSVec4<T> sqqt() const { return NSVec4<T>(s, q, q, t); }
	inline NSVec4<T> sqqp() const { return NSVec4<T>(s, q, q, p); }
	inline NSVec4<T> sqqq() const { return NSVec4<T>(s, q, q, q); }
	inline NSVec4<T> tsss() const { return NSVec4<T>(t, s, s, s); }
	inline NSVec4<T> tsst() const { return NSVec4<T>(t, s, s, t); }
	inline NSVec4<T> tssp() const { return NSVec4<T>(t, s, s, p); }
	inline NSVec4<T> tssq() const { return NSVec4<T>(t, s, s, q); }
	inline NSVec4<T> tsts() const { return NSVec4<T>(t, s, t, s); }
	inline NSVec4<T> tstt() const { return NSVec4<T>(t, s, t, t); }
	inline NSVec4<T> tstp() const { return NSVec4<T>(t, s, t, p); }
	inline NSVec4<T> tstq() const { return NSVec4<T>(t, s, t, q); }
	inline NSVec4<T> tsps() const { return NSVec4<T>(t, s, p, s); }
	inline NSVec4<T> tspt() const { return NSVec4<T>(t, s, p, t); }
	inline NSVec4<T> tspp() const { return NSVec4<T>(t, s, p, p); }
	inline NSVec4<T> tspq() const { return NSVec4<T>(t, s, p, q); }
	inline NSVec4<T> tsqs() const { return NSVec4<T>(t, s, q, s); }
	inline NSVec4<T> tsqt() const { return NSVec4<T>(t, s, q, t); }
	inline NSVec4<T> tsqp() const { return NSVec4<T>(t, s, q, p); }
	inline NSVec4<T> tsqq() const { return NSVec4<T>(t, s, q, q); }
	inline NSVec4<T> ttss() const { return NSVec4<T>(t, t, s, s); }
	inline NSVec4<T> ttst() const { return NSVec4<T>(t, t, s, t); }
	inline NSVec4<T> ttsp() const { return NSVec4<T>(t, t, s, p); }
	inline NSVec4<T> ttsq() const { return NSVec4<T>(t, t, s, q); }
	inline NSVec4<T> ttts() const { return NSVec4<T>(t, t, t, s); }
	inline NSVec4<T> tttt() const { return NSVec4<T>(t, t, t, t); }
	inline NSVec4<T> tttp() const { return NSVec4<T>(t, t, t, p); }
	inline NSVec4<T> tttq() const { return NSVec4<T>(t, t, t, q); }
	inline NSVec4<T> ttps() const { return NSVec4<T>(t, t, p, s); }
	inline NSVec4<T> ttpt() const { return NSVec4<T>(t, t, p, t); }
	inline NSVec4<T> ttpp() const { return NSVec4<T>(t, t, p, p); }
	inline NSVec4<T> ttpq() const { return NSVec4<T>(t, t, p, q); }
	inline NSVec4<T> ttqs() const { return NSVec4<T>(t, t, q, s); }
	inline NSVec4<T> ttqt() const { return NSVec4<T>(t, t, q, t); }
	inline NSVec4<T> ttqp() const { return NSVec4<T>(t, t, q, p); }
	inline NSVec4<T> ttqq() const { return NSVec4<T>(t, t, q, q); }
	inline NSVec4<T> tpss() const { return NSVec4<T>(t, p, s, s); }
	inline NSVec4<T> tpst() const { return NSVec4<T>(t, p, s, t); }
	inline NSVec4<T> tpsp() const { return NSVec4<T>(t, p, s, p); }
	inline NSVec4<T> tpsq() const { return NSVec4<T>(t, p, s, q); }
	inline NSVec4<T> tpts() const { return NSVec4<T>(t, p, t, s); }
	inline NSVec4<T> tptt() const { return NSVec4<T>(t, p, t, t); }
	inline NSVec4<T> tptp() const { return NSVec4<T>(t, p, t, p); }
	inline NSVec4<T> tptq() const { return NSVec4<T>(t, p, t, q); }
	inline NSVec4<T> tpps() const { return NSVec4<T>(t, p, p, s); }
	inline NSVec4<T> tppt() const { return NSVec4<T>(t, p, p, t); }
	inline NSVec4<T> tppp() const { return NSVec4<T>(t, p, p, p); }
	inline NSVec4<T> tppq() const { return NSVec4<T>(t, p, p, q); }
	inline NSVec4<T> tpqs() const { return NSVec4<T>(t, p, q, s); }
	inline NSVec4<T> tpqt() const { return NSVec4<T>(t, p, q, t); }
	inline NSVec4<T> tpqp() const { return NSVec4<T>(t, p, q, p); }
	inline NSVec4<T> tpqq() const { return NSVec4<T>(t, p, q, q); }
	inline NSVec4<T> tqss() const { return NSVec4<T>(t, q, s, s); }
	inline NSVec4<T> tqst() const { return NSVec4<T>(t, q, s, t); }
	inline NSVec4<T> tqsp() const { return NSVec4<T>(t, q, s, p); }
	inline NSVec4<T> tqsq() const { return NSVec4<T>(t, q, s, q); }
	inline NSVec4<T> tqts() const { return NSVec4<T>(t, q, t, s); }
	inline NSVec4<T> tqtt() const { return NSVec4<T>(t, q, t, t); }
	inline NSVec4<T> tqtp() const { return NSVec4<T>(t, q, t, p); }
	inline NSVec4<T> tqtq() const { return NSVec4<T>(t, q, t, q); }
	inline NSVec4<T> tqps() const { return NSVec4<T>(t, q, p, s); }
	inline NSVec4<T> tqpt() const { return NSVec4<T>(t, q, p, t); }
	inline NSVec4<T> tqpp() const { return NSVec4<T>(t, q, p, p); }
	inline NSVec4<T> tqpq() const { return NSVec4<T>(t, q, p, q); }
	inline NSVec4<T> tqqs() const { return NSVec4<T>(t, q, q, s); }
	inline NSVec4<T> tqqt() const { return NSVec4<T>(t, q, q, t); }
	inline NSVec4<T> tqqp() const { return NSVec4<T>(t, q, q, p); }
	inline NSVec4<T> tqqq() const { return NSVec4<T>(t, q, q, q); }
	inline NSVec4<T> psss() const { return NSVec4<T>(p, s, s, s); }
	inline NSVec4<T> psst() const { return NSVec4<T>(p, s, s, t); }
	inline NSVec4<T> pssp() const { return NSVec4<T>(p, s, s, p); }
	inline NSVec4<T> pssq() const { return NSVec4<T>(p, s, s, q); }
	inline NSVec4<T> psts() const { return NSVec4<T>(p, s, t, s); }
	inline NSVec4<T> pstt() const { return NSVec4<T>(p, s, t, t); }
	inline NSVec4<T> pstp() const { return NSVec4<T>(p, s, t, p); }
	inline NSVec4<T> pstq() const { return NSVec4<T>(p, s, t, q); }
	inline NSVec4<T> psps() const { return NSVec4<T>(p, s, p, s); }
	inline NSVec4<T> pspt() const { return NSVec4<T>(p, s, p, t); }
	inline NSVec4<T> pspp() const { return NSVec4<T>(p, s, p, p); }
	inline NSVec4<T> pspq() const { return NSVec4<T>(p, s, p, q); }
	inline NSVec4<T> psqs() const { return NSVec4<T>(p, s, q, s); }
	inline NSVec4<T> psqt() const { return NSVec4<T>(p, s, q, t); }
	inline NSVec4<T> psqp() const { return NSVec4<T>(p, s, q, p); }
	inline NSVec4<T> psqq() const { return NSVec4<T>(p, s, q, q); }
	inline NSVec4<T> ptss() const { return NSVec4<T>(p, t, s, s); }
	inline NSVec4<T> ptst() const { return NSVec4<T>(p, t, s, t); }
	inline NSVec4<T> ptsp() const { return NSVec4<T>(p, t, s, p); }
	inline NSVec4<T> ptsq() const { return NSVec4<T>(p, t, s, q); }
	inline NSVec4<T> ptts() const { return NSVec4<T>(p, t, t, s); }
	inline NSVec4<T> pttt() const { return NSVec4<T>(p, t, t, t); }
	inline NSVec4<T> pttp() const { return NSVec4<T>(p, t, t, p); }
	inline NSVec4<T> pttq() const { return NSVec4<T>(p, t, t, q); }
	inline NSVec4<T> ptps() const { return NSVec4<T>(p, t, p, s); }
	inline NSVec4<T> ptpt() const { return NSVec4<T>(p, t, p, t); }
	inline NSVec4<T> ptpp() const { return NSVec4<T>(p, t, p, p); }
	inline NSVec4<T> ptpq() const { return NSVec4<T>(p, t, p, q); }
	inline NSVec4<T> ptqs() const { return NSVec4<T>(p, t, q, s); }
	inline NSVec4<T> ptqt() const { return NSVec4<T>(p, t, q, t); }
	inline NSVec4<T> ptqp() const { return NSVec4<T>(p, t, q, p); }
	inline NSVec4<T> ptqq() const { return NSVec4<T>(p, t, q, q); }
	inline NSVec4<T> ppss() const { return NSVec4<T>(p, p, s, s); }
	inline NSVec4<T> ppst() const { return NSVec4<T>(p, p, s, t); }
	inline NSVec4<T> ppsp() const { return NSVec4<T>(p, p, s, p); }
	inline NSVec4<T> ppsq() const { return NSVec4<T>(p, p, s, q); }
	inline NSVec4<T> ppts() const { return NSVec4<T>(p, p, t, s); }
	inline NSVec4<T> pptt() const { return NSVec4<T>(p, p, t, t); }
	inline NSVec4<T> pptp() const { return NSVec4<T>(p, p, t, p); }
	inline NSVec4<T> pptq() const { return NSVec4<T>(p, p, t, q); }
	inline NSVec4<T> ppps() const { return NSVec4<T>(p, p, p, s); }
	inline NSVec4<T> pppt() const { return NSVec4<T>(p, p, p, t); }
	inline NSVec4<T> pppp() const { return NSVec4<T>(p, p, p, p); }
	inline NSVec4<T> pppq() const { return NSVec4<T>(p, p, p, q); }
	inline NSVec4<T> ppqs() const { return NSVec4<T>(p, p, q, s); }
	inline NSVec4<T> ppqt() const { return NSVec4<T>(p, p, q, t); }
	inline NSVec4<T> ppqp() const { return NSVec4<T>(p, p, q, p); }
	inline NSVec4<T> ppqq() const { return NSVec4<T>(p, p, q, q); }
	inline NSVec4<T> pqss() const { return NSVec4<T>(p, q, s, s); }
	inline NSVec4<T> pqst() const { return NSVec4<T>(p, q, s, t); }
	inline NSVec4<T> pqsp() const { return NSVec4<T>(p, q, s, p); }
	inline NSVec4<T> pqsq() const { return NSVec4<T>(p, q, s, q); }
	inline NSVec4<T> pqts() const { return NSVec4<T>(p, q, t, s); }
	inline NSVec4<T> pqtt() const { return NSVec4<T>(p, q, t, t); }
	inline NSVec4<T> pqtp() const { return NSVec4<T>(p, q, t, p); }
	inline NSVec4<T> pqtq() const { return NSVec4<T>(p, q, t, q); }
	inline NSVec4<T> pqps() const { return NSVec4<T>(p, q, p, s); }
	inline NSVec4<T> pqpt() const { return NSVec4<T>(p, q, p, t); }
	inline NSVec4<T> pqpp() const { return NSVec4<T>(p, q, p, p); }
	inline NSVec4<T> pqpq() const { return NSVec4<T>(p, q, p, q); }
	inline NSVec4<T> pqqs() const { return NSVec4<T>(p, q, q, s); }
	inline NSVec4<T> pqqt() const { return NSVec4<T>(p, q, q, t); }
	inline NSVec4<T> pqqp() const { return NSVec4<T>(p, q, q, p); }
	inline NSVec4<T> pqqq() const { return NSVec4<T>(p, q, q, q); }
	inline NSVec4<T> qsss() const { return NSVec4<T>(q, s, s, s); }
	inline NSVec4<T> qsst() const { return NSVec4<T>(q, s, s, t); }
	inline NSVec4<T> qssp() const { return NSVec4<T>(q, s, s, p); }
	inline NSVec4<T> qssq() const { return NSVec4<T>(q, s, s, q); }
	inline NSVec4<T> qsts() const { return NSVec4<T>(q, s, t, s); }
	inline NSVec4<T> qstt() const { return NSVec4<T>(q, s, t, t); }
	inline NSVec4<T> qstp() const { return NSVec4<T>(q, s, t, p); }
	inline NSVec4<T> qstq() const { return NSVec4<T>(q, s, t, q); }
	inline NSVec4<T> qsps() const { return NSVec4<T>(q, s, p, s); }
	inline NSVec4<T> qspt() const { return NSVec4<T>(q, s, p, t); }
	inline NSVec4<T> qspp() const { return NSVec4<T>(q, s, p, p); }
	inline NSVec4<T> qspq() const { return NSVec4<T>(q, s, p, q); }
	inline NSVec4<T> qsqs() const { return NSVec4<T>(q, s, q, s); }
	inline NSVec4<T> qsqt() const { return NSVec4<T>(q, s, q, t); }
	inline NSVec4<T> qsqp() const { return NSVec4<T>(q, s, q, p); }
	inline NSVec4<T> qsqq() const { return NSVec4<T>(q, s, q, q); }
	inline NSVec4<T> qtss() const { return NSVec4<T>(q, t, s, s); }
	inline NSVec4<T> qtst() const { return NSVec4<T>(q, t, s, t); }
	inline NSVec4<T> qtsp() const { return NSVec4<T>(q, t, s, p); }
	inline NSVec4<T> qtsq() const { return NSVec4<T>(q, t, s, q); }
	inline NSVec4<T> qtts() const { return NSVec4<T>(q, t, t, s); }
	inline NSVec4<T> qttt() const { return NSVec4<T>(q, t, t, t); }
	inline NSVec4<T> qttp() const { return NSVec4<T>(q, t, t, p); }
	inline NSVec4<T> qttq() const { return NSVec4<T>(q, t, t, q); }
	inline NSVec4<T> qtps() const { return NSVec4<T>(q, t, p, s); }
	inline NSVec4<T> qtpt() const { return NSVec4<T>(q, t, p, t); }
	inline NSVec4<T> qtpp() const { return NSVec4<T>(q, t, p, p); }
	inline NSVec4<T> qtpq() const { return NSVec4<T>(q, t, p, q); }
	inline NSVec4<T> qtqs() const { return NSVec4<T>(q, t, q, s); }
	inline NSVec4<T> qtqt() const { return NSVec4<T>(q, t, q, t); }
	inline NSVec4<T> qtqp() const { return NSVec4<T>(q, t, q, p); }
	inline NSVec4<T> qtqq() const { return NSVec4<T>(q, t, q, q); }
	inline NSVec4<T> qpss() const { return NSVec4<T>(q, p, s, s); }
	inline NSVec4<T> qpst() const { return NSVec4<T>(q, p, s, t); }
	inline NSVec4<T> qpsp() const { return NSVec4<T>(q, p, s, p); }
	inline NSVec4<T> qpsq() const { return NSVec4<T>(q, p, s, q); }
	inline NSVec4<T> qpts() const { return NSVec4<T>(q, p, t, s); }
	inline NSVec4<T> qptt() const { return NSVec4<T>(q, p, t, t); }
	inline NSVec4<T> qptp() const { return NSVec4<T>(q, p, t, p); }
	inline NSVec4<T> qptq() const { return NSVec4<T>(q, p, t, q); }
	inline NSVec4<T> qpps() const { return NSVec4<T>(q, p, p, s); }
	inline NSVec4<T> qppt() const { return NSVec4<T>(q, p, p, t); }
	inline NSVec4<T> qppp() const { return NSVec4<T>(q, p, p, p); }
	inline NSVec4<T> qppq() const { return NSVec4<T>(q, p, p, q); }
	inline NSVec4<T> qpqs() const { return NSVec4<T>(q, p, q, s); }
	inline NSVec4<T> qpqt() const { return NSVec4<T>(q, p, q, t); }
	inline NSVec4<T> qpqp() const { return NSVec4<T>(q, p, q, p); }
	inline NSVec4<T> qpqq() const { return NSVec4<T>(q, p, q, q); }
	inline NSVec4<T> qqss() const { return NSVec4<T>(q, q, s, s); }
	inline NSVec4<T> qqst() const { return NSVec4<T>(q, q, s, t); }
	inline NSVec4<T> qqsp() const { return NSVec4<T>(q, q, s, p); }
	inline NSVec4<T> qqsq() const { return NSVec4<T>(q, q, s, q); }
	inline NSVec4<T> qqts() const { return NSVec4<T>(q, q, t, s); }
	inline NSVec4<T> qqtt() const { return NSVec4<T>(q, q, t, t); }
	inline NSVec4<T> qqtp() const { return NSVec4<T>(q, q, t, p); }
	inline NSVec4<T> qqtq() const { return NSVec4<T>(q, q, t, q); }
	inline NSVec4<T> qqps() const { return NSVec4<T>(q, q, p, s); }
	inline NSVec4<T> qqpt() const { return NSVec4<T>(q, q, p, t); }
	inline NSVec4<T> qqpp() const { return NSVec4<T>(q, q, p, p); }
	inline NSVec4<T> qqpq() const { return NSVec4<T>(q, q, p, q); }
	inline NSVec4<T> qqqs() const { return NSVec4<T>(q, q, q, s); }
	inline NSVec4<T> qqqt() const { return NSVec4<T>(q, q, q, t); }
	inline NSVec4<T> qqqp() const { return NSVec4<T>(q, q, q, p); }
	inline NSVec4<T> qqqq() const { return NSVec4<T>(q, q, q, q); }

	// Vec 3 Swizzles
	inline NSVec3<T> xxx() const { return NSVec3<T>(x, x, x); }
	inline NSVec3<T> xxy() const { return NSVec3<T>(x, x, y); }
	inline NSVec3<T> xxz() const { return NSVec3<T>(x, x, z); }
	inline NSVec3<T> xxw() const { return NSVec3<T>(x, x, w); }
	inline NSVec3<T> xyx() const { return NSVec3<T>(x, y, x); }
	inline NSVec3<T> xyy() const { return NSVec3<T>(x, y, y); }
	inline NSVec3<T> xyz() const { return NSVec3<T>(x, y, z); }
	inline NSVec3<T> xyw() const { return NSVec3<T>(x, y, w); }
	inline NSVec3<T> xzx() const { return NSVec3<T>(x, z, x); }
	inline NSVec3<T> xzy() const { return NSVec3<T>(x, z, y); }
	inline NSVec3<T> xzz() const { return NSVec3<T>(x, z, z); }
	inline NSVec3<T> xzw() const { return NSVec3<T>(x, z, w); }
	inline NSVec3<T> xwx() const { return NSVec3<T>(x, w, x); }
	inline NSVec3<T> xwy() const { return NSVec3<T>(x, w, y); }
	inline NSVec3<T> xwz() const { return NSVec3<T>(x, w, z); }
	inline NSVec3<T> xww() const { return NSVec3<T>(x, w, w); }
	inline NSVec3<T> yxx() const { return NSVec3<T>(y, x, x); }
	inline NSVec3<T> yxy() const { return NSVec3<T>(y, x, y); }
	inline NSVec3<T> yxz() const { return NSVec3<T>(y, x, z); }
	inline NSVec3<T> yxw() const { return NSVec3<T>(y, x, w); }
	inline NSVec3<T> yyx() const { return NSVec3<T>(y, y, x); }
	inline NSVec3<T> yyy() const { return NSVec3<T>(y, y, y); }
	inline NSVec3<T> yyz() const { return NSVec3<T>(y, y, z); }
	inline NSVec3<T> yyw() const { return NSVec3<T>(y, y, w); }
	inline NSVec3<T> yzx() const { return NSVec3<T>(y, z, x); }
	inline NSVec3<T> yzy() const { return NSVec3<T>(y, z, y); }
	inline NSVec3<T> yzz() const { return NSVec3<T>(y, z, z); }
	inline NSVec3<T> yzw() const { return NSVec3<T>(y, z, w); }
	inline NSVec3<T> ywx() const { return NSVec3<T>(y, w, x); }
	inline NSVec3<T> ywy() const { return NSVec3<T>(y, w, y); }
	inline NSVec3<T> ywz() const { return NSVec3<T>(y, w, z); }
	inline NSVec3<T> yww() const { return NSVec3<T>(y, w, w); }
	inline NSVec3<T> zxx() const { return NSVec3<T>(z, x, x); }
	inline NSVec3<T> zxy() const { return NSVec3<T>(z, x, y); }
	inline NSVec3<T> zxz() const { return NSVec3<T>(z, x, z); }
	inline NSVec3<T> zxw() const { return NSVec3<T>(z, x, w); }
	inline NSVec3<T> zyx() const { return NSVec3<T>(z, y, x); }
	inline NSVec3<T> zyy() const { return NSVec3<T>(z, y, y); }
	inline NSVec3<T> zyz() const { return NSVec3<T>(z, y, z); }
	inline NSVec3<T> zyw() const { return NSVec3<T>(z, y, w); }
	inline NSVec3<T> zzx() const { return NSVec3<T>(z, z, x); }
	inline NSVec3<T> zzy() const { return NSVec3<T>(z, z, y); }
	inline NSVec3<T> zzz() const { return NSVec3<T>(z, z, z); }
	inline NSVec3<T> zzw() const { return NSVec3<T>(z, z, w); }
	inline NSVec3<T> zwx() const { return NSVec3<T>(z, w, x); }
	inline NSVec3<T> zwy() const { return NSVec3<T>(z, w, y); }
	inline NSVec3<T> zwz() const { return NSVec3<T>(z, w, z); }
	inline NSVec3<T> zww() const { return NSVec3<T>(z, w, w); }
	inline NSVec3<T> wxx() const { return NSVec3<T>(w, x, x); }
	inline NSVec3<T> wxy() const { return NSVec3<T>(w, x, y); }
	inline NSVec3<T> wxz() const { return NSVec3<T>(w, x, z); }
	inline NSVec3<T> wxw() const { return NSVec3<T>(w, x, w); }
	inline NSVec3<T> wyx() const { return NSVec3<T>(w, y, x); }
	inline NSVec3<T> wyy() const { return NSVec3<T>(w, y, y); }
	inline NSVec3<T> wyz() const { return NSVec3<T>(w, y, z); }
	inline NSVec3<T> wyw() const { return NSVec3<T>(w, y, w); }
	inline NSVec3<T> wzx() const { return NSVec3<T>(w, z, x); }
	inline NSVec3<T> wzy() const { return NSVec3<T>(w, z, y); }
	inline NSVec3<T> wzz() const { return NSVec3<T>(w, z, z); }
	inline NSVec3<T> wzw() const { return NSVec3<T>(w, z, w); }
	inline NSVec3<T> wwx() const { return NSVec3<T>(w, w, x); }
	inline NSVec3<T> wwy() const { return NSVec3<T>(w, w, y); }
	inline NSVec3<T> wwz() const { return NSVec3<T>(w, w, z); }
	inline NSVec3<T> www() const { return NSVec3<T>(w, w, w); }

	inline NSVec3<T> rrr() const { return NSVec3<T>(r, r, r); }
	inline NSVec3<T> rrg() const { return NSVec3<T>(r, r, g); }
	inline NSVec3<T> rrb() const { return NSVec3<T>(r, r, b); }
	inline NSVec3<T> rra() const { return NSVec3<T>(r, r, a); }
	inline NSVec3<T> rgr() const { return NSVec3<T>(r, g, r); }
	inline NSVec3<T> rgg() const { return NSVec3<T>(r, g, g); }
	inline NSVec3<T> rgb() const { return NSVec3<T>(r, g, b); }
	inline NSVec3<T> rga() const { return NSVec3<T>(r, g, a); }
	inline NSVec3<T> rbr() const { return NSVec3<T>(r, b, r); }
	inline NSVec3<T> rbg() const { return NSVec3<T>(r, b, g); }
	inline NSVec3<T> rbb() const { return NSVec3<T>(r, b, b); }
	inline NSVec3<T> rba() const { return NSVec3<T>(r, b, a); }
	inline NSVec3<T> rar() const { return NSVec3<T>(r, a, r); }
	inline NSVec3<T> rag() const { return NSVec3<T>(r, a, g); }
	inline NSVec3<T> rab() const { return NSVec3<T>(r, a, b); }
	inline NSVec3<T> raa() const { return NSVec3<T>(r, a, a); }
	inline NSVec3<T> grr() const { return NSVec3<T>(g, r, r); }
	inline NSVec3<T> grg() const { return NSVec3<T>(g, r, g); }
	inline NSVec3<T> grb() const { return NSVec3<T>(g, r, b); }
	inline NSVec3<T> gra() const { return NSVec3<T>(g, r, a); }
	inline NSVec3<T> ggr() const { return NSVec3<T>(g, g, r); }
	inline NSVec3<T> ggg() const { return NSVec3<T>(g, g, g); }
	inline NSVec3<T> ggb() const { return NSVec3<T>(g, g, b); }
	inline NSVec3<T> gga() const { return NSVec3<T>(g, g, a); }
	inline NSVec3<T> gbr() const { return NSVec3<T>(g, b, r); }
	inline NSVec3<T> gbg() const { return NSVec3<T>(g, b, g); }
	inline NSVec3<T> gbb() const { return NSVec3<T>(g, b, b); }
	inline NSVec3<T> gba() const { return NSVec3<T>(g, b, a); }
	inline NSVec3<T> gar() const { return NSVec3<T>(g, a, r); }
	inline NSVec3<T> gag() const { return NSVec3<T>(g, a, g); }
	inline NSVec3<T> gab() const { return NSVec3<T>(g, a, b); }
	inline NSVec3<T> gaa() const { return NSVec3<T>(g, a, a); }
	inline NSVec3<T> brr() const { return NSVec3<T>(b, r, r); }
	inline NSVec3<T> brg() const { return NSVec3<T>(b, r, g); }
	inline NSVec3<T> brb() const { return NSVec3<T>(b, r, b); }
	inline NSVec3<T> bra() const { return NSVec3<T>(b, r, a); }
	inline NSVec3<T> bgr() const { return NSVec3<T>(b, g, r); }
	inline NSVec3<T> bgg() const { return NSVec3<T>(b, g, g); }
	inline NSVec3<T> bgb() const { return NSVec3<T>(b, g, b); }
	inline NSVec3<T> bga() const { return NSVec3<T>(b, g, a); }
	inline NSVec3<T> bbr() const { return NSVec3<T>(b, b, r); }
	inline NSVec3<T> bbg() const { return NSVec3<T>(b, b, g); }
	inline NSVec3<T> bbb() const { return NSVec3<T>(b, b, b); }
	inline NSVec3<T> bba() const { return NSVec3<T>(b, b, a); }
	inline NSVec3<T> bar() const { return NSVec3<T>(b, a, r); }
	inline NSVec3<T> bag() const { return NSVec3<T>(b, a, g); }
	inline NSVec3<T> bab() const { return NSVec3<T>(b, a, b); }
	inline NSVec3<T> baa() const { return NSVec3<T>(b, a, a); }
	inline NSVec3<T> arr() const { return NSVec3<T>(a, r, r); }
	inline NSVec3<T> arg() const { return NSVec3<T>(a, r, g); }
	inline NSVec3<T> arb() const { return NSVec3<T>(a, r, b); }
	inline NSVec3<T> ara() const { return NSVec3<T>(a, r, a); }
	inline NSVec3<T> agr() const { return NSVec3<T>(a, g, r); }
	inline NSVec3<T> agg() const { return NSVec3<T>(a, g, g); }
	inline NSVec3<T> agb() const { return NSVec3<T>(a, g, b); }
	inline NSVec3<T> aga() const { return NSVec3<T>(a, g, a); }
	inline NSVec3<T> abr() const { return NSVec3<T>(a, b, r); }
	inline NSVec3<T> abg() const { return NSVec3<T>(a, b, g); }
	inline NSVec3<T> abb() const { return NSVec3<T>(a, b, b); }
	inline NSVec3<T> aba() const { return NSVec3<T>(a, b, a); }
	inline NSVec3<T> aar() const { return NSVec3<T>(a, a, r); }
	inline NSVec3<T> aag() const { return NSVec3<T>(a, a, g); }
	inline NSVec3<T> aab() const { return NSVec3<T>(a, a, b); }
	inline NSVec3<T> aaa() const { return NSVec3<T>(a, a, a); }

	inline NSVec3<T> sss() const { return NSVec3<T>(s, s, s); }
	inline NSVec3<T> sst() const { return NSVec3<T>(s, s, t); }
	inline NSVec3<T> ssp() const { return NSVec3<T>(s, s, p); }
	inline NSVec3<T> ssq() const { return NSVec3<T>(s, s, q); }
	inline NSVec3<T> sts() const { return NSVec3<T>(s, t, s); }
	inline NSVec3<T> stt() const { return NSVec3<T>(s, t, t); }
	inline NSVec3<T> stp() const { return NSVec3<T>(s, t, p); }
	inline NSVec3<T> stq() const { return NSVec3<T>(s, t, q); }
	inline NSVec3<T> sps() const { return NSVec3<T>(s, p, s); }
	inline NSVec3<T> spt() const { return NSVec3<T>(s, p, t); }
	inline NSVec3<T> spp() const { return NSVec3<T>(s, p, p); }
	inline NSVec3<T> spq() const { return NSVec3<T>(s, p, q); }
	inline NSVec3<T> sqs() const { return NSVec3<T>(s, q, s); }
	inline NSVec3<T> sqt() const { return NSVec3<T>(s, q, t); }
	inline NSVec3<T> sqp() const { return NSVec3<T>(s, q, p); }
	inline NSVec3<T> sqq() const { return NSVec3<T>(s, q, q); }
	inline NSVec3<T> tss() const { return NSVec3<T>(t, s, s); }
	inline NSVec3<T> tst() const { return NSVec3<T>(t, s, t); }
	inline NSVec3<T> tsp() const { return NSVec3<T>(t, s, p); }
	inline NSVec3<T> tsq() const { return NSVec3<T>(t, s, q); }
	inline NSVec3<T> tts() const { return NSVec3<T>(t, t, s); }
	inline NSVec3<T> ttt() const { return NSVec3<T>(t, t, t); }
	inline NSVec3<T> ttp() const { return NSVec3<T>(t, t, p); }
	inline NSVec3<T> ttq() const { return NSVec3<T>(t, t, q); }
	inline NSVec3<T> tps() const { return NSVec3<T>(t, p, s); }
	inline NSVec3<T> tpt() const { return NSVec3<T>(t, p, t); }
	inline NSVec3<T> tpp() const { return NSVec3<T>(t, p, p); }
	inline NSVec3<T> tpq() const { return NSVec3<T>(t, p, q); }
	inline NSVec3<T> tqs() const { return NSVec3<T>(t, q, s); }
	inline NSVec3<T> tqt() const { return NSVec3<T>(t, q, t); }
	inline NSVec3<T> tqp() const { return NSVec3<T>(t, q, p); }
	inline NSVec3<T> tqq() const { return NSVec3<T>(t, q, q); }
	inline NSVec3<T> pss() const { return NSVec3<T>(p, s, s); }
	inline NSVec3<T> pst() const { return NSVec3<T>(p, s, t); }
	inline NSVec3<T> psp() const { return NSVec3<T>(p, s, p); }
	inline NSVec3<T> psq() const { return NSVec3<T>(p, s, q); }
	inline NSVec3<T> pts() const { return NSVec3<T>(p, t, s); }
	inline NSVec3<T> ptt() const { return NSVec3<T>(p, t, t); }
	inline NSVec3<T> ptp() const { return NSVec3<T>(p, t, p); }
	inline NSVec3<T> ptq() const { return NSVec3<T>(p, t, q); }
	inline NSVec3<T> pps() const { return NSVec3<T>(p, p, s); }
	inline NSVec3<T> ppt() const { return NSVec3<T>(p, p, t); }
	inline NSVec3<T> ppp() const { return NSVec3<T>(p, p, p); }
	inline NSVec3<T> ppq() const { return NSVec3<T>(p, p, q); }
	inline NSVec3<T> pqs() const { return NSVec3<T>(p, q, s); }
	inline NSVec3<T> pqt() const { return NSVec3<T>(p, q, t); }
	inline NSVec3<T> pqp() const { return NSVec3<T>(p, q, p); }
	inline NSVec3<T> pqq() const { return NSVec3<T>(p, q, q); }
	inline NSVec3<T> qss() const { return NSVec3<T>(q, s, s); }
	inline NSVec3<T> qst() const { return NSVec3<T>(q, s, t); }
	inline NSVec3<T> qsp() const { return NSVec3<T>(q, s, p); }
	inline NSVec3<T> qsq() const { return NSVec3<T>(q, s, q); }
	inline NSVec3<T> qts() const { return NSVec3<T>(q, t, s); }
	inline NSVec3<T> qtt() const { return NSVec3<T>(q, t, t); }
	inline NSVec3<T> qtp() const { return NSVec3<T>(q, t, p); }
	inline NSVec3<T> qtq() const { return NSVec3<T>(q, t, q); }
	inline NSVec3<T> qps() const { return NSVec3<T>(q, p, s); }
	inline NSVec3<T> qpt() const { return NSVec3<T>(q, p, t); }
	inline NSVec3<T> qpp() const { return NSVec3<T>(q, p, p); }
	inline NSVec3<T> qpq() const { return NSVec3<T>(q, p, q); }
	inline NSVec3<T> qqs() const { return NSVec3<T>(q, q, s); }
	inline NSVec3<T> qqt() const { return NSVec3<T>(q, q, t); }
	inline NSVec3<T> qqp() const { return NSVec3<T>(q, q, p); }
	inline NSVec3<T> qqq() const { return NSVec3<T>(q, q, q); }

	// Vec 2 Swizzles
	inline NSVec2<T> xx() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> xy() const { return NSVec2<T>(x, y); }
	inline NSVec2<T> xz() const { return NSVec2<T>(x, z); }
	inline NSVec2<T> xw() const { return NSVec2<T>(x, w); }
	inline NSVec2<T> yx() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> yy() const { return NSVec2<T>(y, y); }
	inline NSVec2<T> yz() const { return NSVec2<T>(y, z); }
	inline NSVec2<T> yw() const { return NSVec2<T>(y, w); }
	inline NSVec2<T> zx() const { return NSVec2<T>(z, x); }
	inline NSVec2<T> zy() const { return NSVec2<T>(z, y); }
	inline NSVec2<T> zz() const { return NSVec2<T>(z, z); }
	inline NSVec2<T> zw() const { return NSVec2<T>(z, w); }
	inline NSVec2<T> wx() const { return NSVec2<T>(w, x); }
	inline NSVec2<T> wy() const { return NSVec2<T>(w, y); }
	inline NSVec2<T> wz() const { return NSVec2<T>(w, z); }
	inline NSVec2<T> ww() const { return NSVec2<T>(w, w); }

	inline NSVec2<T> rr() const { return NSVec2<T>(r, r); }
	inline NSVec2<T> rg() const { return NSVec2<T>(r, g); }
	inline NSVec2<T> rb() const { return NSVec2<T>(r, b); }
	inline NSVec2<T> ra() const { return NSVec2<T>(r, a); }
	inline NSVec2<T> gr() const { return NSVec2<T>(g, r); }
	inline NSVec2<T> gg() const { return NSVec2<T>(g, g); }
	inline NSVec2<T> gb() const { return NSVec2<T>(g, b); }
	inline NSVec2<T> ga() const { return NSVec2<T>(g, a); }
	inline NSVec2<T> br() const { return NSVec2<T>(b, r); }
	inline NSVec2<T> bg() const { return NSVec2<T>(b, g); }
	inline NSVec2<T> bb() const { return NSVec2<T>(b, b); }
	inline NSVec2<T> ba() const { return NSVec2<T>(b, a); }
	inline NSVec2<T> ar() const { return NSVec2<T>(a, r); }
	inline NSVec2<T> ag() const { return NSVec2<T>(a, g); }
	inline NSVec2<T> ab() const { return NSVec2<T>(a, b); }
	inline NSVec2<T> aa() const { return NSVec2<T>(a, a); }

	inline NSVec2<T> ss() const { return NSVec2<T>(s, s); }
	inline NSVec2<T> st() const { return NSVec2<T>(s, t); }
	inline NSVec2<T> sp() const { return NSVec2<T>(s, p); }
	inline NSVec2<T> sq() const { return NSVec2<T>(s, q); }
	inline NSVec2<T> ts() const { return NSVec2<T>(t, s); }
	inline NSVec2<T> tt() const { return NSVec2<T>(t, t); }
	inline NSVec2<T> tp() const { return NSVec2<T>(t, p); }
	inline NSVec2<T> tq() const { return NSVec2<T>(t, q); }
	inline NSVec2<T> ps() const { return NSVec2<T>(p, s); }
	inline NSVec2<T> pt() const { return NSVec2<T>(p, t); }
	inline NSVec2<T> pp() const { return NSVec2<T>(p, p); }
	inline NSVec2<T> pq() const { return NSVec2<T>(p, q); }
	inline NSVec2<T> qs() const { return NSVec2<T>(q, s); }
	inline NSVec2<T> qt() const { return NSVec2<T>(q, t); }
	inline NSVec2<T> qp() const { return NSVec2<T>(q, p); }
	inline NSVec2<T> qq() const { return NSVec2<T>(q, q); }

	union
	{
		T data[4];

		struct
		{
			T x;
			T y;
			T z;
			T w;
		};

		struct
		{
			T r;
			T g;
			T b;
			T a;
		};

		struct
		{
			T s;
			T t;
			T p;
			T q;
		};
	};
};

template <class T>
NSVec4<T> operator*(const int32_t & pLHS, const NSVec4<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec4<T> operator*(const float & pLHS, const NSVec4<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec4<T> operator*(const double & pLHS, const NSVec4<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template<class T>
NSVec4<T> operator/(const int32_t & pLHS, const NSVec4<T> & pRHS)
{
	return NSVec4<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z, pLHS / pRHS.w);
}

template <class T>
NSVec4<T> operator/(const float & pLHS, const NSVec4<T> & pRHS)
{
	return NSVec4<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z, pLHS / pRHS.w);
}

template <class T>
NSVec4<T> operator/(const double & pLHS, const NSVec4<T> & pRHS)
{
	return NSVec4<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z, pLHS / pRHS.w);
}

template <class T>
NSVec4<T> abs(const NSVec4<T> & pVec)
{
	return NSVec4<T>(pVec).abs();
}

template <class T>
NSVec4<T> axisAngle(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool rads)
{
	return NSVec4<T>().axisAngleFrom(NSQuat<T>().from(euler, order, rads), rads);
}

template <class T>
NSVec4<T> axisAngle(const NSQuat<T> & orientation, bool rads)
{
	return NSVec4<T>().axisAngleFrom(orientation, rads);
}

template <class T>
NSVec4<T> axisAngle(const NSMat3<T> & rotationMat3, bool rads)
{
	return NSVec4<T>().axisAngleFrom(NSQuat<T>().from(rotationMat3), rads);
}

template <class T>
NSVec4<T> axisAngle(const NSMat4<T> & transform, bool rads)
{
	return NSVec4<T>().axisAngleFrom(NSQuat<T>().from(transform), rads);
}

template <class T>
NSVec4<T> axisAngle(const NSVec3<T> & vec, const NSVec3<T> & toVec, bool rads)
{
	return NSVec4<T>().axisAngleFrom(NSQuat<T>().from(vec, toVec), rads);
}

template <class T>
NSVec4<T> ceil(const NSVec4<T> & pVec)
{
	NSVec4<T> ret(pVec);
	ret.ceiling();
	return ret;
}

template <class T>
NSVec4<T> clamp(const NSVec4<T> & pVec, const T & pMin, const T & pMax)
{
	NSVec4<T> ret(pVec);
	ret.clamp(pMin, pMax);
	return ret;
}

template <class T>
T distance(const NSVec4<T> & lvec, const NSVec4<T> & rvec)
{
	return lvec.distanceTo(rvec);
}

template <class T>
T dot(const NSVec4<T> & pLeft, const NSVec4<T> & pRight)
{
	return pLeft * pRight;
}

template <class T>
NSVec4<T> floor(const NSVec4<T> & pVec)
{
	NSVec4<T> ret(pVec);
	ret.floor();
	return ret;
}

template <class T>
NSVec4<T> fract(const NSVec4<T> & vec)
{
	return vec - floor(vec);
}

template <class T>
T length(const NSVec4<T> & pVec)
{
	return pVec.length();
}

template <class T, class T2>
NSVec4<T> lerp(const NSVec4<T> & lhs, const NSVec4<T> & rhs, T2 scalingFactor)
{
	NSVec4<T> ret(lhs);
	ret.lerp(rhs, scalingFactor);
	return ret;
}

template <class T>
NSVec4<T> min(const NSVec4<T> & pLeft, const NSVec4<T> & pRight)
{
	NSVec2<T> ret(pLeft);
	ret.minimize(pRight);
	return ret;
}

template <class T>
NSVec4<T> max(const NSVec4<T> & pLeft, const NSVec4<T> & pRight)
{
	NSVec2<T> ret(pLeft);
	ret.maximize(pRight);
	return ret;
}

template <class T>
NSVec4<T> normalize(const NSVec4<T> & pRHS)
{
	NSVec4<T> ret(pRHS);
	ret.normalize();
	return ret;
}

template <class T>
NSVec4<T> round(const NSVec4<T> & pVec)
{
	NSVec4<T> ret(pVec);
	ret.round();
	return ret;
}

template<class PUPer, class T>
void pup(PUPer & p, NSVec4<T> & v4, const std::string & varName)
{
	pup(p, v4.x, varName + "[0]"); pup(p, v4.y, varName + "[1]"); pup(p, v4.z, varName + "[2]"); pup(p, v4.w, varName + "[3]");
}

#endif
