#ifndef NSVEC3_H
#define NSVEC3_H

#include "nsvec2.h"

template <class T>
NSVec3<T> operator*(const int & pLHS, const NSVec3<T> & pRHS);

template <class T>
NSVec3<T> operator*(const float & pLHS, const NSVec3<T> & pRHS);

template <class T>
NSVec3<T> operator*(const double & pLHS, const NSVec3<T> & pRHS);

template <class T>
NSVec3<T> operator/(const int & pLHS, const NSVec3<T> & pRHS);

template <class T>
NSVec3<T> operator/(const float & pLHS, const NSVec3<T> & pRHS);

template <class T>
NSVec3<T> operator/(const double & pLHS, const NSVec3<T> & pRHS);

template <class T>
NSVec3<T> abs(const NSVec3<T> & pVec);

template <class T>
NSVec3<T> ceil(const NSVec3<T> & pVec);

template <class T>
NSVec3<T> clamp(const NSVec3<T> & pVec, const T & pMin, const T & pMax);

template <class T>
NSVec3<T> cross(const NSVec3<T> & pLeft, const NSVec3<T> & pRight);

template <class T>
T distance(const NSVec3<T> & lvec, const NSVec3<T> & rvec);

template <class T>
T dot(const NSVec3<T> & pLeft, const NSVec3<T> & pRight);

template <class T>
NSVec3<T> euler(const NSVec4<T> & axisAngle, typename NSVec3<T>::EulerOrder order , bool pRads = false);

template <class T>
NSVec3<T> euler(const NSQuat<T> & orientation, typename NSVec3<T>::EulerOrder order , bool rads = false);

template <class T>
NSVec3<T> euler(const NSMat3<T> & rotationMat3, typename NSVec3<T>::EulerOrder order , bool pRads = false);

template <class T>
NSVec3<T> euler(const NSMat4<T> & transform, typename NSVec3<T>::EulerOrder order , bool pRads = false);

template <class T>
NSVec3<T> euler(const NSVec3<T> & vec, const NSVec3<T> & toVec, typename NSVec3<T>::EulerOrder order , bool pRads = false);

template <class T>
NSVec3<T> floor(const NSVec3<T> & pVec);

template <class T>
NSVec3<T> fract(const NSVec3<T> & vec);

template <class T>
T length(const NSVec3<T> & pVec);

template <class T, class T2>
NSVec3<T> lerp(const NSVec3<T> & lhs, const NSVec3<T> & rhs, T2 scalingFactor);

template <class T>
NSVec3<T> min(const NSVec3<T> & pLeft, const NSVec3<T> & pRight);

template <class T>
NSVec3<T> max(const NSVec3<T> & pLeft, const NSVec3<T> & pRight);

template <class T>
NSVec3<T> normalize(const NSVec3<T> & pRHS);

template <class T>
NSVec3<T> project(const NSVec3<T> & a, const NSVec3<T> & b);

template <class T>
NSVec3<T> projectPlane(const NSVec3<T> & a, const NSVec3<T> & normal);

template <class T>
NSVec3<T> reflect(const NSVec3<T> & incident, const NSVec3<T> & normal);

template <class T>
NSVec3<T> round(const NSVec3<T> & pVec);

template <class T>
NSVec3<T> scalingVec(const NSMat3<T> & transform);

template <class T>
NSVec3<T> scalingVec(const NSMat4<T> & transform);

template <class T>
NSVec3<T> translationVec(const NSMat4<T> & transform);

template<class PUPer, class T>
void pup(PUPer & p, NSVec3<T> & v3);

template <class T>
struct NSVec3
{
	enum CoordSys
	{
		Cartesian,
		Cylindrical,
		Spherical
	};

	enum EulerOrder
	{
		XYZ,
		XZY,
		YXZ,
		YZX,
		ZXY,
		ZYX
	};

	NSVec3(const NSVec3<T> & copy) : x(copy.x), y(copy.y), z(copy.z) {}
	NSVec3(const T & val = static_cast<T>(0)) : x(val), y(val), z(val) {}
	NSVec3(const NSVec2<T> & xy, const T & z_) : x(xy.x), y(xy.y), z(z_) {}
	NSVec3(const T & x_, const NSVec2<T> & yz) : x(x_), y(yz.y), z(yz.z) {}
	NSVec3(const T & x_, const T & y_, const T & z_ = static_cast<T>(0)) : x(x_), y(y_), z(z_) {}

	NSVec3<T> & abs()
	{
		x = static_cast<T>(std::abs(x));
		y = static_cast<T>(std::abs(y));
		z = static_cast<T>(std::abs(z));
		return *this;
	}

	T angleTo(const NSVec3<T> pVec, bool pRads = false) const
	{
		T dotP = (*this) * pVec;
		T l = length() * pVec.length();

		if (l == static_cast<T>(0))
			return static_cast<T>(0);

		dotP /= l;

		if (pRads)
			return T(acos(dotP));
		else
			return T(degrees(acos(dotP)));
	}

	NSVec3<T> & ceil()
	{
		x = static_cast<T>(std::ceil(x));
		y = static_cast<T>(std::ceil(y));
		z = static_cast<T>(std::ceil(z));
		return *this;
	}

	NSVec3<T> & clamp(const T & min = static_cast<T>(0), const T & max = static_cast<T>(0))
	{
		if (x < min)
			x = min;
		if (y < min)
			y = min;
		if (z < min)
			z = min;
		if (x > max)
			x = max;
		if (y > max)
			y = max;
		if (z > max)
			z = max;
		return *this;
	}

	NSVec3<T> & cross(const NSVec3<T> & crossWith)
	{
		T tmpx = x, tmpy = y;
		x = y * crossWith.z - z * crossWith.y;
		y = z * crossWith.x - tmpx * crossWith.z;
		z = tmpx * crossWith.y - tmpy * crossWith.x;
		return *this;
	}

	NSVec3<T> cylindrical(bool pRads = false) const
	{
		NSVec3<T> ret;
		ret.x = sqrt(x*x + y*y);

		if (x == static_cast<T>(0))
		{
			ret.y = static_cast<T>(PI / 2);
			if (y == static_cast<T>(0))
				ret.y = static_cast<T>(0);
		}
		else
			ret.y = static_cast<T>(atan2(y,x));

		ret.z = z;

		if (!pRads)
			ret.y = degrees(ret.y);

		return ret;
	}

	T distanceTo(const NSVec3<T> & pVec) const
	{
		return ((pVec - *this).length());
	}

	NSVec3<T> & eulerFrom(const NSVec4<T> & axisAngle, EulerOrder order = XYZ, bool pRads = false)
	{
		return eulerFrom(NSQuat<T>().from(axisAngle, pRads), order, pRads);
	}

	NSVec3<T> & eulerFrom(const NSQuat<T> & orientation, EulerOrder order, bool rads)
	{
		return eulerFrom(NSMat3<T>().rotationFrom(orientation), order, rads);
	}

	NSVec3<T> & eulerFrom(const NSMat3<T> & rotationMat3, EulerOrder order = XYZ, bool pRads = false)
	{
		// https://github.com/mrdoob/three.js/blob/master/src/math/Euler.js
		T ep = static_cast<T>(1) - EPS;
		switch (order)
		{
		case(XYZ) :
			y = std::asin(rotationMat3[0][2]);
			if (std::abs(rotationMat3[0][2]) < ep)
			{
				x = std::atan2(-rotationMat3[1][2], rotationMat3[2][2]);
				z = std::atan2(-rotationMat3[0][1], rotationMat3[0][0]);
			}
			else
			{
				x = std::atan2(rotationMat3[2][1], rotationMat3[1][1]);
				z = 0;
			}
			break;
		case(XZY) :
			z = std::asin(rotationMat3[0][1]);
			if (std::abs(rotationMat3[0][1]) < ep)
			{
				x = std::atan2(rotationMat3[2][1], rotationMat3[1][1]);
				y = std::atan2(rotationMat3[0][2], rotationMat3[0][0]);
			}
			else
			{
				x = std::atan2(-rotationMat3[1][2], rotationMat3[2][2]);
				y = 0;
			}
			break;
		case(YXZ) :
			x = std::asin(rotationMat3[1][2]);
			if (std::abs(rotationMat3[1][2]) < ep)
			{
				y = std::atan2(rotationMat3[0][2], rotationMat3[2][2]);
				z = std::atan2(rotationMat3[1][0], rotationMat3[1][1]);
			}
			else
			{
				y = std::atan2(-rotationMat3[2][0], rotationMat3[0][0]);
				z = 0;
			}
			break;
		case(YZX) :
			z = std::asin(rotationMat3[1][0]);
			if (std::abs(rotationMat3[1][0]) < ep)
			{
				x = std::atan2(-rotationMat3[1][2], rotationMat3[1][1]);
				y = std::atan2(-rotationMat3[2][0], rotationMat3[0][0]);
			}
			else
			{
				x = 0;
				y = std::atan2(rotationMat3[0][2], rotationMat3[2][2]);
			}
			break;
		case(ZXY) :
			x = std::asin(rotationMat3[2][1]);
			if (std::abs(rotationMat3[2][1]) < ep)
			{
				y = std::atan2(-rotationMat3[2][0], rotationMat3[2][2]);
				z = std::atan2(-rotationMat3[0][1], rotationMat3[1][1]);
			}
			else
			{
				y = 0;
				z = std::atan2(rotationMat3[1][0], rotationMat3[0][0]);
			}
			break;
		case(ZYX) :
			y = std::asin(rotationMat3[2][0]);
			if (std::abs(rotationMat3[2][0]) < ep)
			{
				x = std::atan2(rotationMat3[2][1], rotationMat3[2][2]);
				z = std::atan2(rotationMat3[1][0], rotationMat3[0][0]);
			}
			else
			{
				x = 0;
				z = std::atan2(-rotationMat3[0][1], rotationMat3[1][1]);
			}
			break;
		}
		if (!pRads)
			*this = degrees(*this);
		return *this;
	}

	NSVec3<T> & eulerFrom(const NSMat4<T> & transform, EulerOrder order = XYZ, bool pRads = false)
	{
		return eulerFrom(rotationMat3(transform), order, pRads);
	}

	NSVec3<T> & eulerFrom(const NSVec3<T> & vec, const NSVec3<T> & toVec, EulerOrder order = XYZ, bool pRads = false)
	{
		return eulerFrom(NSQuat<T>().from(vec, toVec), order, pRads);
	}

	NSVec3<T> & floor()
	{
		x = static_cast<T>(std::floor(x));
		y = static_cast<T>(std::floor(y));
		z = static_cast<T>(std::floor(z));
		return *this;
	}

	NSVec3<T> & fract()
	{
		x -= static_cast<T>(std::floor(x));
		y -= static_cast<T>(std::floor(y));
		z -= static_cast<T>(std::floor(z));
		return *this;
	}

	NSVec3<T> & from(CoordSys coordSys, const NSVec3<T> & vec, bool pRads = false)
	{
		switch (coordSys)
		{
		case(Cylindrical) :
		{
			T theta = vec.y;
			if (!pRads)
				theta = radians(theta);
			x = vec.x*cos(theta);
			y = vec.x*sin(theta);
			z = vec.z;
			break;
		}
		case(Spherical) :
		{
			T theta = vec.y; T phi = vec.z;
			if (!pRads)
			{
				theta = radians(theta);
				phi = radians(phi);
			}
			x = vec.x*cos(theta)*sin(phi);
			y = vec.x*sin(theta)*sin(phi);
			z = vec.x*cos(phi);
			break;
		}
		default:
			*this = vec;
		}
		return *this;
	}

	T length() const
	{
		return static_cast<T>(sqrt(x*x + y*y + z*z));
	}

	T lengthSq() const
	{
		return x*x + y*y + z*z;
	}

	template<class T2 >
	NSVec3<T> & lerp(const NSVec3<T> & vec, const T2 & scalingFactor)
	{
		x += static_cast<T>((vec.x - x)*scalingFactor);
		y += static_cast<T>((vec.y - y)*scalingFactor);
		z += static_cast<T>((vec.z - z)*scalingFactor);
		return *this;
	}

	T min()
	{
		if (x < y)
		{
			if (x < z)
				return x;
			else
				return z;
		}
		else
		{
			if (y < z)
				return y;
			else
				return z;
		}
	}

	NSVec3<T> & minimize(const NSVec3<T> & rhs)
	{
		if (x < rhs.x)
			x = rhs.x;
		if (y < rhs.y)
			y = rhs.y;
		if (z < rhs.z)
			z = rhs.z;
		return *this;
	}

	T max()
	{
		if (x > y)
		{
			if (x > z)
				return x;
			else
				return z;
		}
		else
		{
			if (y > z)
				return y;
			else
				return z;
		}
	}

	NSVec3<T> & maximize(const NSVec3<T> & rhs)
	{
		if (x > rhs.x)
			x = rhs.x;
		if (y > rhs.y)
			y = rhs.y;
		if (z > rhs.z)
			z = rhs.z;
		return *this;
	}

	NSVec3<T> & normalize()
	{
		T l = length();
		if (::abs(l) <= static_cast<T>(EPS))
			return *this;
		return *this *= static_cast<T>(1) / l;
	}

	NSVec3<T> & projectOn(const NSVec3<T> & vec)
	{
		T denom = vec * vec;
		if (denom == static_cast<T>(0))
			return *this;
		(*this) = ((*this * vec) / denom) * vec;
		return *this;
	}

	NSVec3<T> & projectOnPlane(const NSVec3<T> & planeNormal)
	{
		NSVec3<T> aonb(*this);
		aonb.projectOn(planeNormal);
		(*this) -= aonb;
		return *this;
	}

	NSVec3<T> & reflect(const NSVec3<T> & normal)
	{

		(*this) = (*this) - (static_cast<T>(2) * (normal * *this)) * normal;
		return *this;
	}

	NSVec3<T> & round()
	{
		x = static_cast<T>(std::round(x));
		y = static_cast<T>(std::round(y));
		z = static_cast<T>(std::round(z));
		return *this;
	}

	NSVec3<T> & roundToZero()
	{
		if (::abs(x) < EPS)
			x = 0;
		if (::abs(y) < EPS)
			y = 0;
		if (::abs(z) < EPS)
			z = 0;
		return *this;
	}

	NSVec3<T> & scalingFrom(const NSMat3<T> & transform)
	{
		x = transform[0].length();
		y = transform[1].length();
		z = transform[2].length();
		return *this;
	}

	NSVec3<T> & scalingFrom(const NSMat4<T> & transform)
	{
		x = sqrt(transform[0][0] * transform[0][0] + transform[0][1] * transform[0][1] + transform[0][2] * transform[0][2]);
		y = sqrt(transform[1][0] * transform[1][0] + transform[1][1] * transform[1][1] + transform[1][2] * transform[1][2]);
		z = sqrt(transform[2][0] * transform[2][0] + transform[2][1] * transform[2][1] + transform[2][2] * transform[2][2]);
		return *this;
	}

	NSVec3<T> & translationFrom(const NSMat4<T> & transform)
	{
		return *this = transform(3).xyz();
	}

	NSVec3<T> & set(const T & pVal)
	{
		x = y = z = pVal;
		return *this;
	}

	NSVec3<T> & set(const T & pX, const T & pY, const T & pZ)
	{
		x = pX; y = pY; z = pZ;
		return *this;
	}

	NSVec3<T> & set(const NSVec2<T> & xy, const T & pZ)
	{
		x = xy.x; y = xy.y; z = pZ;
		return *this;
	}

	NSVec3<T> & set(const T & pX, const NSVec2<T> & yz)
	{
		x = pX; y = yz.x; z = yz.y;
		return *this;
	}

	NSVec3<T> & setLength(const T & len)
	{
		T l = length();

		if (l == static_cast<T>(0))
			return *this;

		T mult = len / l;
		(*this) *= mult;
		return *this;
	}

	NSVec3<T> spherical(bool pRads = false) const
	{
		NSVec3<T> ret;
		ret.x = length();

		if (x == static_cast<T>(0))
		{
			ret.y = static_cast<T>(PI / 2);
			if (y == static_cast<T>(0))
				ret.y = static_cast<T>(0);
		}
		else
			ret.y = static_cast<T>(atan2(y,x));

		if (ret.x == static_cast<T>(0))
			ret.z = static_cast<T>(0);
		else
			ret.z = acos(z / ret.x);

		if (!pRads)
		{
			ret.y = degrees(ret.y);
			ret.z = degrees(ret.z);
		}

		return ret;
	}

	std::string toString(CoordSys disp = Cartesian) const
	{
		std::ostringstream ss;
		if (disp == Cartesian)
			ss  << "[" << x << " " << y << " " << z << "]";
		else if (disp == Cylindrical)
		{
			NSVec3<T> cyl = cylindrical();
			ss << "[" << cyl.x << " " << cyl.y << " " << cyl.z << "]";
		}
		else
		{
			NSVec3<T> sph = spherical();
			ss << "[" << sph.x << " " << sph.y << " " << sph.z << "]";
		}
		return ss.str();
	}

	// overloaded operators
	NSVec3<T> operator+(const NSVec3<T> & rhs) const
	{
		return NSVec3<T>(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	NSVec3<T> operator-(const NSVec3<T> & rhs) const
	{
		return NSVec3<T>(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	T operator*(const NSVec3<T> & rhs) const // dot product
	{
		return x*rhs.x + y*rhs.y + z*rhs.z;
	}

	NSMat3<T> operator^(const NSVec3<T> & pRHS) const
	{
		NSMat3<T> ret;
		ret[0] = x * pRHS;
		ret[1] = y * pRHS;
		ret[2] = z * pRHS;
		return ret;
	}

	NSVec3<T> operator%(const NSVec3<T> & rhs) const // component wise scalar product
	{
		return NSVec3<T>(x*rhs.x, y*rhs.y, z*rhs.z);
	}

	NSVec3<T> operator/(const NSVec3<T> & rhs) const
	{
		return NSVec3<T>(x/rhs.x, y/rhs.y, z/rhs.z);
	}

	NSVec3<T> operator*(const T & rhs) const
	{
		return NSVec3<T>(x * rhs, y * rhs, z * rhs);
	}

	NSVec3<T> operator/(const T & rhs) const
	{
		return NSVec3<T>(x / rhs, y / rhs, z / rhs);
	}

	NSVec3<T> & operator=(const NSVec3<T> & rhs)
	{
		if (this == &rhs)
			return *this;
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		return *this;
	}

	NSVec3<T> operator++(int)
	{
		NSVec3<T> ret(*this);
		++(*this);
		return ret;
	}

	NSVec3<T> operator--(int)
	{
		NSVec3<T> ret(*this);
		--(*this);
		return ret;
	}

	NSVec3<T> & operator++()
	{
		++x; ++y; ++z;
		return *this;
	}

	NSVec3<T> & operator--()
	{
		--x; --y; --z;
		return *this;
	}

	NSVec3<T> & operator+=(const NSVec3<T> & rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z;
		return *this;
	}

	NSVec3<T> & operator-=(const NSVec3<T> & rhs)
	{
		x -= rhs.x; y -= rhs.y; z -= rhs.z;
		return *this;
	}

	NSVec3<T> & operator%=(const NSVec3<T> & rhs)
	{
		x *= rhs.x; y *= rhs.y; z *= rhs.z;
		return *this;
	}

	NSVec3<T> & operator/=(const NSVec3<T> & rhs)
	{
		x /= rhs.x; y /= rhs.y; z /= rhs.z;
		return *this;
	}

	NSVec3<T> & operator*=(const T & rhs)
	{
		x *= rhs; y *= rhs; z *= rhs;
		return *this;
	}

	NSVec3<T> & operator/=(const T & rhs)
	{
		x /= rhs; y /= rhs; z /= rhs;
		return *this;
	}

	bool operator==(const NSVec3<T> & rhs) const
	{
		return ((x == rhs.x) && (y == rhs.y) && (z == rhs.z));
	}

	bool operator!=(const NSVec3<T> & rhs) const
	{
		return !(*this == rhs);
	}

	bool operator==(const T & rhs) const
	{
		return ((x == rhs) && (y == rhs) && (z == rhs));
	}

	bool operator!=(const T & rhs) const
	{
		return !(*this == rhs);
	}

	const T & operator[](const uint & pVal) const
	{
		if (pVal > 2)
			throw(std::out_of_range("vec3 index out of range"));
		return data[pVal];
	}

	T & operator[](const uint & pVal)
	{
		if (pVal > 2)
			throw(std::out_of_range("vec3 index out of range"));
		return data[pVal];
	}


	// Swizzle operations
	inline NSVec3<T> xxx() const { return NSVec3<T>(x, x, x); }
	inline NSVec3<T> xxy() const { return NSVec3<T>(x, x, y); }
	inline NSVec3<T> xxz() const { return NSVec3<T>(x, x, z); }
	inline NSVec3<T> xyx() const { return NSVec3<T>(x, y, x); }
	inline NSVec3<T> xyy() const { return NSVec3<T>(x, y, y); }
	inline NSVec3<T> xzx() const { return NSVec3<T>(x, z, x); }
	inline NSVec3<T> xzy() const { return NSVec3<T>(x, z, y); }
	inline NSVec3<T> xzz() const { return NSVec3<T>(x, z, z); }

	inline NSVec3<T> yxx() const { return NSVec3<T>(y, x, x); }
	inline NSVec3<T> yxy() const { return NSVec3<T>(y, x, y); }
	inline NSVec3<T> yxz() const { return NSVec3<T>(y, x, z); }
	inline NSVec3<T> yyx() const { return NSVec3<T>(y, y, x); }
	inline NSVec3<T> yyy() const { return NSVec3<T>(y, y, y); }
	inline NSVec3<T> yyz() const { return NSVec3<T>(y, y, z); }
	inline NSVec3<T> yzx() const { return NSVec3<T>(y, z, x); }
	inline NSVec3<T> yzy() const { return NSVec3<T>(y, z, y); }
	inline NSVec3<T> yzz() const { return NSVec3<T>(y, z, z); }

	inline NSVec3<T> zxx() const { return NSVec3<T>(z, x, x); }
	inline NSVec3<T> zxy() const { return NSVec3<T>(z, x, y); }
	inline NSVec3<T> zxz() const { return NSVec3<T>(z, x, z); }
	inline NSVec3<T> zyx() const { return NSVec3<T>(z, y, x); }
	inline NSVec3<T> zyy() const { return NSVec3<T>(z, y, y); }
	inline NSVec3<T> zyz() const { return NSVec3<T>(z, y, z); }
	inline NSVec3<T> zzx() const { return NSVec3<T>(z, z, x); }
	inline NSVec3<T> zzy() const { return NSVec3<T>(z, z, y); }
	inline NSVec3<T> zzz() const { return NSVec3<T>(z, z, z); }


	inline NSVec3<T> rrr() const { return NSVec3<T>(x, x, x); }
	inline NSVec3<T> rrg() const { return NSVec3<T>(x, x, y); }
	inline NSVec3<T> rrb() const { return NSVec3<T>(x, x, z); }
	inline NSVec3<T> rgr() const { return NSVec3<T>(x, y, x); }
	inline NSVec3<T> rgg() const { return NSVec3<T>(x, y, y); }
	inline NSVec3<T> rbr() const { return NSVec3<T>(x, z, x); }
	inline NSVec3<T> rbg() const { return NSVec3<T>(x, z, y); }
	inline NSVec3<T> rbb() const { return NSVec3<T>(x, z, z); }

	inline NSVec3<T> grr() const { return NSVec3<T>(y, x, x); }
	inline NSVec3<T> grg() const { return NSVec3<T>(y, x, y); }
	inline NSVec3<T> grb() const { return NSVec3<T>(y, x, z); }
	inline NSVec3<T> ggr() const { return NSVec3<T>(y, y, x); }
	inline NSVec3<T> ggg() const { return NSVec3<T>(y, y, y); }
	inline NSVec3<T> ggb() const { return NSVec3<T>(y, y, z); }
	inline NSVec3<T> gbr() const { return NSVec3<T>(y, z, x); }
	inline NSVec3<T> gbg() const { return NSVec3<T>(y, z, y); }
	inline NSVec3<T> gbb() const { return NSVec3<T>(y, z, z); }

	inline NSVec3<T> brr() const { return NSVec3<T>(z, x, x); }
	inline NSVec3<T> brg() const { return NSVec3<T>(z, x, y); }
	inline NSVec3<T> brb() const { return NSVec3<T>(z, x, z); }
	inline NSVec3<T> bgr() const { return NSVec3<T>(z, y, x); }
	inline NSVec3<T> bgg() const { return NSVec3<T>(z, y, y); }
	inline NSVec3<T> bgb() const { return NSVec3<T>(z, y, z); }
	inline NSVec3<T> bbr() const { return NSVec3<T>(z, z, x); }
	inline NSVec3<T> bbg() const { return NSVec3<T>(z, z, y); }
	inline NSVec3<T> bbb() const { return NSVec3<T>(z, z, z); }


	inline NSVec3<T> sss() const { return NSVec3<T>(x, x, x); }
	inline NSVec3<T> sst() const { return NSVec3<T>(x, x, y); }
	inline NSVec3<T> ssp() const { return NSVec3<T>(x, x, z); }
	inline NSVec3<T> sts() const { return NSVec3<T>(x, y, x); }
	inline NSVec3<T> stt() const { return NSVec3<T>(x, y, y); }
	inline NSVec3<T> sps() const { return NSVec3<T>(x, z, x); }
	inline NSVec3<T> spt() const { return NSVec3<T>(x, z, y); }
	inline NSVec3<T> spp() const { return NSVec3<T>(x, z, z); }

	inline NSVec3<T> tss() const { return NSVec3<T>(y, x, x); }
	inline NSVec3<T> tst() const { return NSVec3<T>(y, x, y); }
	inline NSVec3<T> tsp() const { return NSVec3<T>(y, x, z); }
	inline NSVec3<T> tts() const { return NSVec3<T>(y, y, x); }
	inline NSVec3<T> ttt() const { return NSVec3<T>(y, y, y); }
	inline NSVec3<T> ttp() const { return NSVec3<T>(y, y, z); }
	inline NSVec3<T> tps() const { return NSVec3<T>(y, z, x); }
	inline NSVec3<T> tpt() const { return NSVec3<T>(y, z, y); }
	inline NSVec3<T> tpp() const { return NSVec3<T>(y, z, z); }

	inline NSVec3<T> pss() const { return NSVec3<T>(z, x, x); }
	inline NSVec3<T> pst() const { return NSVec3<T>(z, x, y); }
	inline NSVec3<T> psp() const { return NSVec3<T>(z, x, z); }
	inline NSVec3<T> pts() const { return NSVec3<T>(z, y, x); }
	inline NSVec3<T> ptt() const { return NSVec3<T>(z, y, y); }
	inline NSVec3<T> ptp() const { return NSVec3<T>(z, y, z); }
	inline NSVec3<T> pps() const { return NSVec3<T>(z, z, x); }
	inline NSVec3<T> ppt() const { return NSVec3<T>(z, z, y); }
	inline NSVec3<T> ppp() const { return NSVec3<T>(z, z, z); }


	inline NSVec2<T> xx() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> xy() const { return NSVec2<T>(x, y); }
	inline NSVec2<T> xz() const { return NSVec2<T>(x, z); }
	inline NSVec2<T> yx() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> yy() const { return NSVec2<T>(y, y); }
	inline NSVec2<T> yz() const { return NSVec2<T>(y, z); }
	inline NSVec2<T> zx() const { return NSVec2<T>(z, x); }
	inline NSVec2<T> zy() const { return NSVec2<T>(z, y); }
	inline NSVec2<T> zz() const { return NSVec2<T>(z, z); }

	inline NSVec2<T> rr() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> rg() const { return NSVec2<T>(x, y); }
	inline NSVec2<T> rb() const { return NSVec2<T>(x, z); }
	inline NSVec2<T> gr() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> gg() const { return NSVec2<T>(y, y); }
	inline NSVec2<T> gb() const { return NSVec2<T>(y, z); }
	inline NSVec2<T> br() const { return NSVec2<T>(z, x); }
	inline NSVec2<T> bg() const { return NSVec2<T>(z, y); }
	inline NSVec2<T> bb() const { return NSVec2<T>(z, z); }

	inline NSVec2<T> ss() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> st() const { return NSVec2<T>(x, y); }
	inline NSVec2<T> sp() const { return NSVec2<T>(x, z); }
	inline NSVec2<T> ts() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> tt() const { return NSVec2<T>(y, y); }
	inline NSVec2<T> tp() const { return NSVec2<T>(y, z); }
	inline NSVec2<T> ps() const { return NSVec2<T>(z, x); }
	inline NSVec2<T> pt() const { return NSVec2<T>(z, y); }
	inline NSVec2<T> pp() const { return NSVec2<T>(z, z); }


	union
	{
		T data[3];

		struct
		{
			T x;
			T y;
			T z;
		};

		struct
		{
			T r;
			T g;
			T b;
		};

		struct
		{
			T s;
			T t;
			T p;
		};
	};
};

template <class T>
NSVec3<T> operator*(const int & pLHS, const NSVec3<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec3<T> operator*(const float & pLHS, const NSVec3<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec3<T> operator*(const double & pLHS, const NSVec3<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec3<T> operator/(const int & pLHS, const NSVec3<T> & pRHS)
{
	return NSVec3<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z);
}

template <class T>
NSVec3<T> operator/(const float & pLHS, const NSVec3<T> & pRHS)
{
	return NSVec3<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z);
}

template <class T>
NSVec3<T> operator/(const double & pLHS, const NSVec3<T> & pRHS)
{
	return NSVec3<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z);
}

template <class T>
NSVec3<T> abs(const NSVec3<T> & pVec)
{
	return NSVec3<T>(pVec).abs();
}

template <class T>
NSVec3<T> ceil(const NSVec3<T> & pVec)
{
	NSVec3<T> ret(pVec);
	ret.ceiling();
	return ret;
}

template <class T>
NSVec3<T> clamp(const NSVec3<T> & pVec, const T & pMin, const T & pMax)
{
	NSVec3<T> ret(pVec);
	ret.clamp(pMin, pMax);
	return ret;
}

template <class T>
NSVec3<T> cross(const NSVec3<T> & pLeft, const NSVec3<T> & pRight)
{
	NSVec3<T> ret;
	ret.x = pLeft.y * pRight.z - pLeft.z * pRight.y;
	ret.y = pLeft.z * pRight.x - pLeft.x * pRight.z;
	ret.z = pLeft.x * pRight.y - pLeft.y * pRight.x;
	return ret;
}

template <class T>
T distance(const NSVec3<T> & lvec, const NSVec3<T> & rvec)
{
	return lvec.distanceTo(rvec);
}

template <class T>
T dot(const NSVec3<T> & pLeft, const NSVec3<T> & pRight)
{
	return pLeft * pRight;
}

template <class T>
NSVec3<T> euler(const NSVec4<T> & axisAngle, typename NSVec3<T>::EulerOrder order, bool pRads)
{
	return NSVec3<T>().eulerFrom(NSQuat<T>().from(axisAngle, pRads), order, pRads);
}

template <class T>
NSVec3<T> euler(const NSQuat<T> & orientation, typename NSVec3<T>::EulerOrder order, bool rads)
{
	return NSVec3<T>().eulerFrom(NSMat3<T>().rotationFrom(orientation), order, rads);
}

template <class T>
NSVec3<T> euler(const NSMat3<T> & rotationMat3, typename NSVec3<T>::EulerOrder order, bool pRads)
{
	return NSVec3<T>().eulerFrom(rotationMat3, order, pRads);
}

template <class T>
NSVec3<T> euler(const NSMat4<T> & transform, typename NSVec3<T>::EulerOrder order, bool pRads)
{
	return NSVec3<T>().eulerFrom(transform, order, pRads);
}

template <class T>
NSVec3<T> euler(const NSVec3<T> & vec, const NSVec3<T> & toVec, typename NSVec3<T>::EulerOrder order, bool pRads)
{
	return NSVec3<T>().eulerFrom(NSQuat<T>().from(vec, toVec), order, pRads);
}

template <class T>
NSVec3<T> floor(const NSVec3<T> & pVec)
{
	NSVec3<T> ret(pVec);
	ret.floor();
	return ret;
}

template <class T>
NSVec3<T> fract(const NSVec3<T> & vec)
{
	return vec - floor(vec);
}

template <class T>
T length(const NSVec3<T> & pVec)
{
	return pVec.length();
}

template <class T, class T2>
NSVec3<T> lerp(const NSVec3<T> & lhs, const NSVec3<T> & rhs, T2 scalingFactor)
{
	NSVec3<T> ret(lhs);
	ret.lerp(rhs, scalingFactor);
	return ret;
}

template <class T>
NSVec3<T> min(const NSVec3<T> & pLeft, const NSVec3<T> & pRight)
{
	NSVec2<T> ret(pLeft);
	ret.minimize(pRight);
	return ret;
}

template <class T>
NSVec3<T> max(const NSVec3<T> & pLeft, const NSVec3<T> & pRight)
{
	NSVec2<T> ret(pLeft);
	ret.maximize(pRight);
	return ret;
}

template <class T>
NSVec3<T> normalize(const NSVec3<T> & pRHS)
{
	NSVec3<T> ret(pRHS);
	ret.normalize();
	return ret;
}

template <class T>
NSVec3<T> project(const NSVec3<T> & a, const NSVec3<T> & b)
{
	NSVec3<T> ret(a);
	a.projectOn(b);
	return ret;
}

template <class T>
NSVec3<T> projectPlane(const NSVec3<T> & a, const NSVec3<T> & normal)
{
	NSVec3<T> ret(a);
	a.projectOnPlane(normal);
	return ret;
}

template <class T>
NSVec3<T> reflect(const NSVec3<T> & incident, const NSVec3<T> & normal)
{
	NSVec3<T> ret(incident);
	ret.reflect(normal);
	return ret;
}

template <class T>
NSVec3<T> round(const NSVec3<T> & pVec)
{
	NSVec3<T> ret(pVec);
	ret.round();
	return ret;
}

template <class T>
NSVec3<T> scalingVec(const NSMat3<T> & transform)
{
	return NSVec3<T>().scalingFrom(transform);
}

template <class T>
NSVec3<T> scalingVec(const NSMat4<T> & transform)
{
	return NSVec3<T>().scalingFrom(transform);
}

template <class T>
NSVec3<T> translationVec(const NSMat4<T> & transform)
{
	return NSVec3<T>().translationFrom(transform);
}

template<class PUPer, class T>
void pup(PUPer & p, NSVec3<T> & v3, const std::string & varName)
{
	pup(p, v3.x, varName + "[0]"); pup(p, v3.y, varName + "[1]"); pup(p, v3.z, varName + "[2]");
}

#endif
