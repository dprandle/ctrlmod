#ifndef NSVEC2_H
#define NSVEC2_H

#include <cmath>

template <class T>
struct NSVec3;

template <class T>
struct NSVec4;

template <class T>
struct nsquat;

template <class T>
struct nsmat2;

template <class T>
struct nsmat3;

template <class T>
struct nsmat4;

template <class T>
struct NSVec2;

template <class T>
NSVec2<T> operator*(const int32_t & pLHS, const NSVec2<T> & pRHS);

template <class T>
NSVec2<T> operator*(const float & pLHS, const NSVec2<T> & pRHS);

template <class T>
NSVec2<T> operator*(const double & pLHS, const NSVec2<T> & pRHS);

template <class T>
NSVec2<T> operator/(const int32_t & pLHS, const NSVec2<T> & pRHS);

template <class T>
NSVec2<T> operator/(const float & pLHS, const NSVec2<T> & pRHS);

template <class T>
NSVec2<T> operator/(const double & pLHS, const NSVec2<T> & pRHS);

template <class T>
NSVec2<T> abs(const NSVec2<T> & pVec);

template <class T>
NSVec2<T> ceil(const NSVec2<T> & pVec);

template <class T>
NSVec2<T> clamp(const NSVec2<T> & pVec, const T & pMin, const T & pMax);

template <class T>
T distance(const NSVec2<T> & lvec, const NSVec2<T> & rvec);

template <class T>
T dot(const NSVec2<T> & pLeft, const NSVec2<T> & pRight);

template <class T>
NSVec2<T> floor(const NSVec2<T> & pVec);

template <class T>
NSVec2<T> fract(const NSVec2<T> & vec);

template <class T>
T length(const NSVec2<T> & pVec);

template <class T, class T2>
NSVec2<T> lerp(const NSVec2<T> & lhs, const NSVec2<T> & rhs, T2 scalingFactor);

template <class T>
NSVec2<T> min(const NSVec2<T> & pLeft, const NSVec2<T> & pRight);

template <class T>
NSVec2<T> max(const NSVec2<T> & pLeft, const NSVec2<T> & pRight);

template <class T>
NSVec2<T> normalize(const NSVec2<T> & pRHS);

template <class T>
NSVec2<T> project(const NSVec2<T> & a, const NSVec2<T> & b);

template <class T>
NSVec2<T> project_plane(const NSVec2<T> & a, const NSVec2<T> & normal);

template <class T>
NSVec2<T> reflect(const NSVec2<T> & incident, const NSVec2<T> & normal);

template <class T>
NSVec2<T> round(const NSVec2<T> & pVec);

template <class T>
NSVec2<T> scaling2d_vec(const nsmat2<T> & transform2d);

template <class T>
NSVec2<T> scaling2d_vec(const nsmat3<T> & transform2d);

template <class T>
NSVec2<T> translation2d_vec(const nsmat3<T> & transform2d);

template<class PUPer, class T>
void pup(PUPer & p, NSVec2<T> & v2);

template <class T>
struct NSVec2
{
	NSVec2(const NSVec2<T> & pCopy) : x(pCopy.x), y(pCopy.y) {}

	NSVec2(const T & pInit=static_cast<T>(0)) : x(pInit), y(pInit) {}

	NSVec2(const T & pX,const T & pY) : x(pX), y(pY) {}

	NSVec2<T> & abs()
	{
		x = static_cast<T>(std::abs(x));
		y = static_cast<T>(std::abs(y));
		return *this;
	}

	T angle(bool pRads = false) const
	{
		return angleTo(NSVec2<T>(1, 0),pRads);
	}

	T angleTo(const NSVec2<T> & pVec, bool pRads = false) const
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

	NSVec2<T> & ceil()
	{
		x = static_cast<T>(std::ceil(x));
		y = static_cast<T>(std::ceil(y));
		return *this;
	}

	NSVec2<T> & clamp(const T & pMin = static_cast<T>(0), const T & pMax = static_cast<T>(1))
	{
		if (x < pMin)
			x = pMin;
		if (y < pMin)
			y = pMin;
		if (x > pMax)
			x = pMax;
		if (y > pMax)
			y = pMax;
		return *this;
	}

	T distanceTo(const NSVec2<T> & pVec) const
	{
		return ((pVec - *this).length());
	}

	NSVec2<T> & floor()
	{
		x = static_cast<T>(std::floor(x));
		y = static_cast<T>(std::floor(y));
		return *this;
	}

	NSVec2<T> & fract()
	{
		x -= static_cast<T>(std::floor(x));
		y -= static_cast<T>(std::floor(y));
		return *this;
	}

	T length() const
	{
		return static_cast<T>(sqrt(x*x + y*y));
	}

	T lengthSq() const
	{
		return x*x + y*y;
	}

	template<class T2>
	NSVec2<T> & lerp(const NSVec2<T> & vec, const T2 & scalingFactor)
	{
		x += static_cast<T>((vec.x - x)*scalingFactor);
		y += static_cast<T>((vec.y - y)*scalingFactor);
		return *this;
	}

	T min() const
	{
		if (x < y)
			return x;
		else
			return y;
	}

	NSVec2<T> & minimize(const NSVec2<T> & rhs)
	{
		if (x < rhs.x)
			x = rhs.x;
		if (y < rhs.y)
			y = rhs.y;
		return *this;
	}

	T max() const
	{
		if (x > y)
			return x;
		else
			return y;
	}

	NSVec2<T> & maximize(const NSVec2<T> & rhs)
	{
		if (x > rhs.x)
			x = rhs.x;
		if (y > rhs.y)
			y = rhs.y;
		return *this;
	}

	NSVec2<T> & normalize()
	{
		T l = length();
		if (l == static_cast<T>(0))
			return *this;
		return *this *= static_cast<T>(1) / l;
	}

	NSVec2<T> polar(bool pRads = false) const
	{
		return NSVec2<T>(length(), angle(pRads));
	}
	
	NSVec2<T> & projectOn(const NSVec2<T> & vec)
	{
		T denom = vec * vec;
		if (denom == static_cast<T>(0))
			return *this;
		(*this) = ((*this * vec) / denom) * vec;
		return *this;
	}

	NSVec2<T> & projectOnPlane(const NSVec2<T> & planeNormal)
	{
		NSVec2<T> aonb(*this);
		aonb.projectOn(planeNormal);
		(*this) -= aonb;
		return *this;
	}
	
	NSVec2<T> & reflect(const NSVec2<T> & normal)
	{
		(*this) = (*this) - (static_cast<T>(2) * (normal * *this)) * normal;
		return *this;
	}

	template<class T2>
	NSVec2<T> & rotate(const T2 & angle_)
	{
		T2 newangle = angle_ + static_cast<T2>(angle());
		x = length()*static_cast<T>(std::cos(angle_ + angle()));
		y = length()*static_cast<T>(std::sin(angle_ + angle()));
		return *this;
	}

	NSVec2<T> & round()
	{
		x = static_cast<T>(std::round(x));
		y = static_cast<T>(std::round(y));
		return *this;
	}

	NSVec2<T> & roundToZero()
	{
		if (::abs(x) < EPS)
			x = 0;
		if (::abs(y) < EPS)
			y = 0;
		return *this;
	}

	NSVec2<T> & scalingFrom(const nsmat2<T> & transform2d)
	{
		x = transform2d[0].length();
		y = transform2d[1].length();
		return *this;
	}

	NSVec2<T> & scalingFrom(const nsmat3<T> & transform2d)
	{
		x = sqrt(transform2d[0][0] * transform2d[0][0] + transform2d[0][1] * transform2d[0][1]);
		y = sqrt(transform2d[1][0] * transform2d[1][0] + transform2d[1][1] * transform2d[1][1]);
		return *this;
	}

	NSVec2<T> & set(const T & pVal)
	{
		x = y = pVal;
		return *this;
	}

	NSVec2<T> & set(const T & pX, const T & pY)
	{
		x = pX; y = pY;
		return *this;
	}

	NSVec2<T> & setFromPolar(const T & pMag, T angle_, bool pRads = false)
	{
		if (!pRads)
			angle_ = radians(angle_);

		x = static_cast<T>(pMag*std::cos(angle_));
		y = static_cast<T>(pMag*std::sin(angle_));
		return *this;
	}

	NSVec2<T> & setFromPolar(const NSVec2<T> & pVec, bool pRads = false)
	{
		T ang = pVec.y;

		if (!pRads)
			ang = radians(ang);

		x = static_cast<T>(pVec.x*std::cos(ang));
		y = static_cast<T>(pVec.x*std::sin(ang));
		return *this;
	}

	NSVec2<T> & setLength(const T & len)
	{
		T l = length();

		if (l == static_cast<T>(0))
			return *this;

		T mult = len / l;
		(*this) *= mult;
		return *this;
	}

	NSVec2<T> & translationFrom(const nsmat3<T> & transform2d)
	{
		return *this = transform2d(2).xy();
	}

	std::string toString(bool pPolar = false)
	{
		std::ostringstream ss;
		if (pPolar)
			ss << "[" << length() << " " << angle() << "]";
		else
			ss << "[" << x << " " << y << "]";
		return ss.str();
	}

	NSVec2<T> operator+(const NSVec2<T> & pRHS) const
	{
		return NSVec2<T>(x + pRHS.x, y + pRHS.y);
	}

	NSVec2<T> operator-(const NSVec2<T> & pRHS) const
	{
		return NSVec2<T>(x - pRHS.x, y - pRHS.y);
	}

	T operator*(const NSVec2<T> & pRHS) const
	{
		return x*pRHS.x + y*pRHS.y;
	}

	nsmat2<T> operator^(const NSVec2<T> & pRHS) const
	{
		nsmat2<T> ret;
		ret[0] = x * pRHS;
		ret[1] = y * pRHS;
		return ret;
	}

	NSVec2<T> operator%(const NSVec2<T> & pRHS) const
	{
		return NSVec2<T>(x*pRHS.x, y*pRHS.y);
	}

	NSVec2<T> operator/(const NSVec2<T> & pRHS) const
	{
		return NSVec2<T>(x / pRHS.x, y / pRHS.y);
	}

	NSVec2<T> operator*(const T & pRHS) const
	{
		return NSVec2<T>(x * pRHS, y * pRHS);
	}

	NSVec2<T> operator/(const T & pRHS) const
	{
		return NSVec2<T>(x / pRHS, y / pRHS);
	}

	NSVec2<T> & operator=(const NSVec2<T> & pRHS)
	{
		if (this == &pRHS)
			return *this;
		x = pRHS.x;
		y = pRHS.y;
		return *this;
	}

	NSVec2<T> operator++(int32_t)
	{
		NSVec2<T> ret(*this);
		++(*this);
		return ret;
	}

	NSVec2<T> operator--(int32_t)
	{
		NSVec2<T> ret(*this);
		--(*this);
		return ret;
	}

	NSVec2<T> & operator++()
	{
		++x; ++y;
		return *this;
	}

	NSVec2<T> & operator--()
	{
		--x; --y;
		return *this;
	}

	NSVec2<T> & operator+=(const NSVec2<T> & pRHS)
	{
		x += pRHS.x; y += pRHS.y;
		return *this;
	}

	NSVec2<T> & operator-=(const NSVec2<T> & pRHS)
	{
		x -= pRHS.x; y -= pRHS.y;
		return *this;
	}

	NSVec2<T> & operator%=(const NSVec2<T> & pRHS)
	{
		x *= pRHS.x; y *= pRHS.y;
		return *this;
	}

	NSVec2<T> & operator/=(const NSVec2<T> & pRHS)
	{
		x /= pRHS.x; y /= pRHS.y;
		return *this;
	}

	NSVec2<T> & operator*=(const T & pRHS)
	{
		x *= pRHS; y *= pRHS;
		return *this;
	}

	NSVec2<T> & operator/=(const T & pRHS)
	{
		x /= pRHS; y /= pRHS;
		return *this;
	}

	bool operator==(const NSVec2<T> & pRHS) const
	{
		return ((x == pRHS.x) && (y == pRHS.y));
	}

	bool operator!=(const NSVec2<T> & pRHS) const
	{
		return !(*this == pRHS);
	}

	bool operator==(const T & pRHS) const
	{
		return ((x == pRHS) && (y == pRHS));
	}

	bool operator!=(const T & pRHS) const
	{
		return !(*this == pRHS);
	}

	const T & operator[](const uint32_t & pVal) const
	{
		if (pVal > 1)
			throw(std::out_of_range("vec2 index out of range"));
		return data[pVal];
	}

	T & operator[](const uint32_t & pVal)
	{
		if (pVal > 1)
			throw(std::out_of_range("vec2 index out of range"));
		return data[pVal];
	}


	// Swizzle operations
	inline NSVec2<T> xx() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> yx() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> yy() const { return NSVec2<T>(y, y); }

	inline NSVec2<T> ss() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> ts() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> tt() const { return NSVec2<T>(y, y); }

	inline NSVec2<T> uu() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> vu() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> vv() const { return NSVec2<T>(y, y); }

	inline NSVec2<T> ww() const { return NSVec2<T>(x, x); }
	inline NSVec2<T> hw() const { return NSVec2<T>(y, x); }
	inline NSVec2<T> hh() const { return NSVec2<T>(y, y); }

	union
	{
		T data[2];

		struct 
		{
			T x;
			T y;
		};

		struct 
		{
			T w;
			T h;
		};

		struct 
		{
			T s;
			T t;
		};

		struct 
		{
			T u;
			T v;
		};
	};
};

template <class T>
NSVec2<T> operator*(const int32_t & pLHS, const NSVec2<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec2<T> operator*(const float & pLHS, const NSVec2<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec2<T> operator*(const double & pLHS, const NSVec2<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSVec2<T> operator/(const int32_t & pLHS, const NSVec2<T> & pRHS)
{
	return NSVec2<T>(pLHS / pRHS.x, pLHS / pRHS.y);
}

template <class T>
NSVec2<T> operator/(const float & pLHS, const NSVec2<T> & pRHS)
{
	return NSVec2<T>(pLHS / pRHS.x, pLHS / pRHS.y);
}

template <class T>
NSVec2<T> operator/(const double & pLHS, const NSVec2<T> & pRHS)
{
	return NSVec2<T>(pLHS / pRHS.x, pLHS / pRHS.y);
}

template <class T>
NSVec2<T> abs(const NSVec2<T> & pVec)
{
	return NSVec2<T>(pVec).abs();
}

template <class T>
NSVec2<T> ceil(const NSVec2<T> & pVec)
{
	NSVec2<T> ret(pVec);
	ret.ceiling();
	return ret;
}

template <class T>
NSVec2<T> clamp(const NSVec2<T> & pVec, const T & pMin, const T & pMax)
{
	NSVec2<T> ret(pVec);
	ret.clamp(pMin, pMax);
	return ret;
}

template <class T>
T distance(const NSVec2<T> & lvec, const NSVec2<T> & rvec)
{
	return lvec.distanceTo(rvec);
}

template <class T>
T dot(const NSVec2<T> & pLeft, const NSVec2<T> & pRight)
{
	return pLeft * pRight;
}

template <class T>
NSVec2<T> floor(const NSVec2<T> & pVec)
{
	NSVec2<T> ret(pVec);
	ret.floor();
	return ret;
}

template <class T>
NSVec2<T> fract(const NSVec2<T> & vec)
{
	return vec - floor(vec);
}

template <class T>
T length(const NSVec2<T> & pVec)
{
	return pVec.length();
}

template <class T, class T2>
NSVec2<T> lerp(const NSVec2<T> & lhs, const NSVec2<T> & rhs, T2 scalingFactor)
{
	NSVec2<T> ret(lhs);
	ret.lerp(rhs, scalingFactor);
	return ret;
}

template <class T>
NSVec2<T> min(const NSVec2<T> & pLeft, const NSVec2<T> & pRight)
{
	NSVec2<T> ret(pLeft);
	ret.minimize(pRight);
	return ret;
}

template <class T>
NSVec2<T> max(const NSVec2<T> & pLeft, const NSVec2<T> & pRight)
{
	NSVec2<T> ret(pLeft);
	ret.maximize(pRight);
	return ret;
}

template <class T>
NSVec2<T> normalize(const NSVec2<T> & pRHS)
{
	NSVec2<T> ret(pRHS);
	ret.normalize();
	return ret;
}

template <class T>
NSVec2<T> project(const NSVec2<T> & a, const NSVec2<T> & b)
{
	NSVec2<T> ret(a);
	a.projectOn(b);
	return ret;
}

template <class T>
NSVec2<T> project_plane(const NSVec2<T> & a, const NSVec2<T> & normal)
{
	NSVec2<T> ret(a);
	a.projectOnPlane(normal);
	return ret;
}

template <class T>
NSVec2<T> reflect(const NSVec2<T> & incident, const NSVec2<T> & normal)
{
	NSVec2<T> ret(incident);
	incident.reflect(normal);
	return ret;
}

template <class T>
NSVec2<T> round(const NSVec2<T> & pVec)
{
	NSVec2<T> ret(pVec);
	ret.round();
	return ret;
}

template <class T>
NSVec2<T> scaling2d_vec(const nsmat2<T> & transform2d)
{
	return NSVec2<T>().scalingFrom(transform2d);
}

template <class T>
NSVec2<T> scaling2d_vec(const nsmat3<T> & transform2d)
{
	return NSVec2<T>().scalingFrom(transform2d);
}

template <class T>
NSVec2<T> translation2d_vec(const nsmat3<T> & transform2d)
{
	return NSVec2<T>().translationFrom(transform2d);
}

template<class PUPer, class T>
void pup(PUPer & p, NSVec2<T> & v2, const std::string & varName)
{
	pup(p, v2.x, varName + "[0]"); pup(p, v2.y, varName + "[1]");
}
#endif
