#ifndef NSQUAT_H
#define NSQUAT_H

#include "nsvec4.h"

template <class T>
nsquat<T> operator*(const int32_t & pLHS, const nsquat<T> & pRHS);

template <class T>
nsquat<T> operator*(const float & pLHS, const nsquat<T> & pRHS);

template <class T>
nsquat<T> operator*(const double & pLHS, const nsquat<T> & pRHS);

template <class T>
nsquat<T> operator/(const int32_t & pLHS, const nsquat<T> & pRHS);

template <class T>
nsquat<T> operator/(const float & pLHS, const nsquat<T> & pRHS);

template <class T>
nsquat<T> operator/(const double & pLHS, const nsquat<T> & pRHS);

template<class T>
nsquat<T> orientation(const nsmat3<T> & rotationMat3);

template<class T>
nsquat<T> orientation(const NSVec4<T> & axisAngle, bool pRads = false);

template<class T>
nsquat<T> orientation(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order , bool pRads = false);

template<class T>
nsquat<T> orientation(const NSVec3<T> & vec, const NSVec3<T> & toVec);

template<class T>
nsquat<T> orientation(const nsmat4<T> & transform);

template<class T>
nsquat<T> conjugate(const nsquat<T> & quat);

template <class T>
T dot(const nsquat<T> & pLeft, const nsquat<T> & pRight);

template<class T>
nsquat<T> inverse(const nsquat<T> & quat);

template <class T>
nsquat<T> normalize(const nsquat<T> & quat);

template <class T>
nsquat<T> slerp(const nsquat<T> & lhs, const nsquat<T> & rhs, const T & scalingFactor);

template<class PUPer, class T>
void pup(PUPer & p, nsquat<T> & q4);

template <class T>
struct nsquat
{
	nsquat(const nsquat<T> & copy) : x(copy.x), y(copy.y), z(copy.z), w(copy.w) {}
	nsquat() : x(static_cast<T>(0)), y(static_cast<T>(0)), z(static_cast<T>(0)), w(static_cast<T>(1)) {}
	nsquat(const T & pX, const T & pY, const T & pZ, const T & pW) : x(pX), y(pY), z(pZ), w(pW) {}

	nsquat<T> & conjugate()
	{
		x *= static_cast<T>(-1);
		y *= static_cast<T>(-1);
		z *= static_cast<T>(-1);
		return *this;
	}

	nsquat<T> & from(const NSVec4<T> & axisAngle, bool pRads = false)
	{
		T ang = axisAngle.w;
		ang /= static_cast<T>(2);

		if (!pRads)
			ang = ::radians(ang);

		T sAng = static_cast<T>(std::sin(ang));
		x = axisAngle.x * sAng;
		y = axisAngle.y * sAng;
		z = axisAngle.z * sAng;
		w = static_cast<T>(std::cos(ang));
		return *this;
	}

	nsquat<T> & from(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool pRads = false)
	{
		T c1, c2, c3, s1, s2, s3;
		if (!pRads)
		{
			NSVec3<T> eul(::radians(euler));
			c1 = static_cast<T>(std::cos(eul.x / 2));
			c2 = static_cast<T>(std::cos(eul.y / 2));
			c3 = static_cast<T>(std::cos(eul.z / 2));
			s1 = static_cast<T>(std::sin(eul.x / 2));
			s2 = static_cast<T>(std::sin(eul.y / 2));
			s3 = static_cast<T>(std::sin(eul.z / 2));
		}
		else
		{
			c1 = static_cast<T>(std::cos(euler.x / 2));
			c2 = static_cast<T>(std::cos(euler.y / 2));
			c3 = static_cast<T>(std::cos(euler.z / 2));
			s1 = static_cast<T>(std::sin(euler.x / 2));
			s2 = static_cast<T>(std::sin(euler.y / 2));
			s3 = static_cast<T>(std::sin(euler.z / 2));
		}

		switch (order)
		{
		case(NSVec3<T>::XYZ) :
		{
			x = s1 * c2 * c3 + c1 * s2 * s3;
			y = c1 * s2 * c3 - s1 * c2 * s3;
			z = c1 * c2 * s3 + s1 * s2 * c3;
			w = c1 * c2 * c3 - s1 * s2 * s3;
			break;
		}
		case(NSVec3<T>::XZY) :
		{
			x = s1 * c2 * c3 - c1 * s2 * s3;
			y = c1 * s2 * c3 - s1 * c2 * s3;
			z = c1 * c2 * s3 + s1 * s2 * c3;
			w = c1 * c2 * c3 + s1 * s2 * s3;
			break;
		}
		case(NSVec3<T>::YXZ) :
		{
			x = s1 * c2 * c3 + c1 * s2 * s3;
			y = c1 * s2 * c3 - s1 * c2 * s3;
			z = c1 * c2 * s3 - s1 * s2 * c3;
			w = c1 * c2 * c3 + s1 * s2 * s3;
			break;
		}
		case(NSVec3<T>::YZX) :
		{
			x = s1 * c2 * c3 + c1 * s2 * s3;
			y = c1 * s2 * c3 + s1 * c2 * s3;
			z = c1 * c2 * s3 - s1 * s2 * c3;
			w = c1 * c2 * c3 - s1 * s2 * s3;
			break;
		}
		case(NSVec3<T>::ZXY) :
		{
			x = s1 * c2 * c3 - c1 * s2 * s3;
			y = c1 * s2 * c3 + s1 * c2 * s3;
			z = c1 * c2 * s3 + s1 * s2 * c3;
			w = c1 * c2 * c3 - s1 * s2 * s3;
			break;
		}
		case(NSVec3<T>::ZYX) :
		{
			x = s1 * c2 * c3 - c1 * s2 * s3;
			y = c1 * s2 * c3 + s1 * c2 * s3;
			z = c1 * c2 * s3 - s1 * s2 * c3;
			w = c1 * c2 * c3 + s1 * s2 * s3;
			break;
		}
		}
		return *this;
	}

	nsquat<T> & from(const nsmat3<T> & rotationMat3)
	{
		// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm

		T tr = rotationMat3[0][0] + rotationMat3[1][1] + rotationMat3[2][2], s;

		if (tr > 0)
		{
			s = sqrt(tr + 1.0) * 2;
			w = 0.25 * s;
			x = (rotationMat3[2][1] - rotationMat3[1][2]) / s;
			y = (rotationMat3[0][2] - rotationMat3[2][0]) / s;
			z = (rotationMat3[1][0] - rotationMat3[0][1]) / s;
		}
		else if ((rotationMat3[0][0] > rotationMat3[1][1])&(rotationMat3[0][0] > rotationMat3[2][2]))
		{
			s = sqrt(1.0 + rotationMat3[0][0] - rotationMat3[1][1] - rotationMat3[2][2]) * 2;
			w = (rotationMat3[2][1] - rotationMat3[1][2]) / s;
			x = 0.25 * s;
			y = (rotationMat3[0][1] + rotationMat3[1][0]) / s;
			z = (rotationMat3[0][2] + rotationMat3[2][0]) / s;
		}
		else if (rotationMat3[1][1] > rotationMat3[2][2])
		{
			s = sqrt(1.0 + rotationMat3[1][1] - rotationMat3[0][0] - rotationMat3[2][2]) * 2;
			w = (rotationMat3[0][2] - rotationMat3[2][0]) / s;
			x = (rotationMat3[0][1] + rotationMat3[1][0]) / s;
			y = 0.25 * s;
			z = (rotationMat3[1][2] + rotationMat3[2][1]) / s;
		}
		else
		{
			s = sqrt(1.0 + rotationMat3[2][2] - rotationMat3[0][0] - rotationMat3[1][1]) * 2;
			w = (rotationMat3[1][0] - rotationMat3[0][1]) / s;
			x = (rotationMat3[0][2] + rotationMat3[2][0]) / s;
			y = (rotationMat3[1][2] + rotationMat3[2][1]) / s;
			z = 0.25 * s;
		}
		return *this;
	}

	nsquat<T> & from(const nsmat4<T> & transform)
	{
		return *this;
	}

	nsquat<T> & from(const NSVec3<T> & vec, const NSVec3<T> & toVec)
	{
		/* http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm */
		T real = static_cast<T>(1) + vec * toVec;
		NSVec3<T> imag = cross(vec, toVec);
		if (real < EPS)
		{
			w = 0; x = -vec.z; y = vec.y; z = vec.x;
			return (*this).normalize();
		}
		w = real; x = imag.x; y = imag.y; z = imag.z;
		return (*this).normalize();
	}

	nsquat<T> & invert()
	{
		return conjugate().normalize();
	}

	T length() const
	{
		return static_cast<T>(sqrt(x*x + y*y + z*z + w*w));
	}
	
	T lengthSq() const
	{
		return x*x + y*y + z*z + w*w;
	}

	nsquat<T> & normalize()
	{
		T l = length();
		if (l == static_cast<T>(0))
			return setIdentity();
		return *this *= static_cast<T>(1) / l;
	}

	NSVec3<T> right() const
	{
		nsmat3<T> rot = rotationMat3(*this);
		return rot.transpose().right();
		//return NSVec3<T>(1.0f - 2.0f*y*y - 2.0f*z*z, 2.0f*x*y - 2.0f*w*z, 2.0f*x*z + 2.0f*w*y);
	}

	nsquat<T> & roundToZero()
	{
		if (abs(x) < EPS)
			x = 0;
		if (abs(y) < EPS)
			y = 0;
		if (abs(x) < EPS)
			z = 0;
		if (abs(w) < EPS)
			w = 0;
		return *this;
	}

	nsquat<T> & set(const T & pX, const T & pY, const T & pZ, const T & pW)
	{
		x = pX; y = pY; z = pZ; w = pW;
		return *this;
	}

	nsquat<T> & setIdentity()
	{
		x = static_cast<T>(0); y = static_cast<T>(0); z = static_cast<T>(0); z = static_cast<T>(1);
		return *this;
	}

	nsquat<T> & slerp(const nsquat<T> & second, const T & scalingFactor)
	{
		T cosHalfTheta = dot(*this, second);

		if (cosHalfTheta < 0)
		{
			*this = second * static_cast<T>(-1.0f);
			cosHalfTheta = -cosHalfTheta;
		}
		else
		{
			*this = second;
		}

		// make sure we dont go out of acos domain
		if (cosHalfTheta >= static_cast<T>(1))
			return *this;

		// Calculate more expensive values.
		T halfTheta = static_cast<T>(std::acos(cosHalfTheta));
		T sinHalfTheta = static_cast<T>(sqrt(static_cast<T>(1) - cosHalfTheta*cosHalfTheta));

		// if sin of theta = 0 (or something close) then just
		// take  point exactly between the two quats
		if (abs(sinHalfTheta) < EPS)
			return *this = static_cast<T>(0.5) * (*this + second);

		T ratioA = static_cast<T>(std::sin((static_cast<T>(1) - scalingFactor) * halfTheta)) / sinHalfTheta;
		T ratioB = static_cast<T>(std::sin(scalingFactor * halfTheta)) / sinHalfTheta;
		*this = *this * ratioA + second * ratioB;
		return normalize();
	}

	NSVec3<T> target() const
	{
		nsmat3<T> rot = rotationMat3(*this);
		return rot.transpose().target();
		//return NSVec3<T>(2.0f*x*z - 2.0f*w*y, 2.0f*y*z + 2.0f*w*x, 1.0f - 2.0f*x*x - 2.0f*y*y);
	}

	std::string toString()
	{
		std::ostringstream ss;
		ss << "[" << w << " " << x << "i " << y << "j " << z << "k]";
		return ss.str();
	}

	NSVec3<T> up() const
	{
		nsmat3<T> rot = rotationMat3(*this);
		return rot.transpose().up();
		//return NSVec3<T>(2.0f*x*y + 2.0f*w*z, 1.0f - 2.0f*x*x - 2.0f*z*z, 2.0f*y*z - 2.0f*w*x);
	}

	// overloaded operators
	nsquat<T> operator+(const nsquat<T> & rhs) const
	{
		return nsquat<T>(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
	}

	nsquat<T> operator-(const nsquat<T> & rhs) const
	{
		return nsquat<T>(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
	}

	nsquat<T> operator*(const nsquat<T> & rhs) const
	{
		nsquat<T> ret;
		ret.x = w*rhs.x + x*rhs.w + y*rhs.z - z*rhs.y;
		ret.y = w*rhs.y - x*rhs.z + y*rhs.w + z*rhs.x;
		ret.z = w*rhs.z + x*rhs.y - y*rhs.x + z*rhs.w;
		ret.w = w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z;
		return ret;
	}

	nsquat<T> operator/(const nsquat<T> & rhs) const
	{
		return *this * inverse(rhs);
	}

	nsquat<T> operator%(const nsquat<T> & rhs) const // component wise scalar product
	{
		return nsquat<T>(x*rhs.x, y*rhs.y, z*rhs.z, w*rhs.w);
	}

	NSVec3<T> operator*(const NSVec3<T> & rhs) const
	{
		NSVec3<T> ret;
		// quat * vec3
		T ix = w * rhs.x + y * rhs.z - z * rhs.y;
		T iy = w * rhs.y + z * rhs.x - x * rhs.z;
		T iz = w * rhs.z + x * rhs.y - y * rhs.x;
		T iw = -x * rhs.x - y * rhs.y - z * rhs.z;
		// vec3 * inverse(quat)
		ret.x = ix * w + iw * -x + iy * -z - iz * -y;
		ret.y = iy * w + iw * -y + iz * -x - ix * -z;
		ret.z = iz * w + iw * -z + ix * -y - iy * -x;
		return ret;
	}

	nsquat<T> operator*(const T & rhs) const
	{
		return nsquat<T>(x * rhs, y * rhs, z * rhs, w * rhs);
	}

	nsquat<T> operator/(const T & rhs) const
	{
		return nsquat<T>(x / rhs, y / rhs, z / rhs, w / rhs);
	}

	nsquat<T> & operator=(const nsquat<T> & rhs)
	{
		if (this == &rhs)
			return *this;
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
		return *this;
	}

	nsquat<T> operator++(int32_t)
	{
		nsquat<T> ret(*this);
		++(*this);
		return ret;
	}

	nsquat<T> operator--(int32_t)
	{
		nsquat<T> ret(*this);
		--(*this);
		return ret;
	}

	nsquat<T> & operator++()
	{
		++x; ++y; ++z; ++w;
		return *this;
	}

	nsquat<T> & operator--()
	{
		--x; --y; --z; --w;
		return *this;
	}

	nsquat<T> & operator+=(const nsquat<T> & rhs)
	{
		x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w;
		return *this;
	}

	nsquat<T> & operator-=(const nsquat<T> & rhs)
	{
		x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w;
		return *this;
	}

	nsquat<T> & operator*=(const nsquat<T> & rhs)
	{
		return *this = *this * rhs;
	}

	nsquat<T> & operator/=(const nsquat<T> & rhs)
	{
		return *this = *this * inverse(rhs);
	}

	nsquat<T> & operator%=(const nsquat<T> & rhs)
	{
		x *= rhs.x; y *= rhs.y; z *= rhs.z; w *= rhs.w;
		return *this;
	}

	nsquat<T> & operator*=(const T & rhs)
	{
		x *= rhs; y *= rhs; z *= rhs; w *= rhs;
		return *this;
	}

	nsquat<T> & operator/=(const T & rhs)
	{
		x /= rhs; y /= rhs; z /= rhs; w /= rhs;
		return *this;
	}

	bool operator==(const nsquat<T> & rhs) const
	{
		return ((x == rhs.x) && (y == rhs.y) && (z == rhs.z) && (w == rhs.w));
	}

	bool operator!=(const nsquat<T> & rhs) const
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
			throw(std::out_of_range("quat index out of range"));
		return data[pVal];
	}

	T & operator[](const uint32_t & pVal)
	{
		if (pVal > 3)
			throw(std::out_of_range("quat index out of range"));
		return data[pVal];
	}

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
			T b;
			T c;
			T d;
			T a;
		};
	};
};

template <class T>
nsquat<T> operator*(const int32_t & pLHS, const nsquat<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
nsquat<T> operator*(const float & pLHS, const nsquat<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
nsquat<T> operator*(const double & pLHS, const nsquat<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
nsquat<T> operator/(const int32_t & pLHS, const nsquat<T> & pRHS)
{
	return nsquat<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z, pLHS / pRHS.w);
}

template <class T>
nsquat<T> operator/(const float & pLHS, const nsquat<T> & pRHS)
{
	return nsquat<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z, pLHS / pRHS.w);
}

template <class T>
nsquat<T> operator/(const double & pLHS, const nsquat<T> & pRHS)
{
	return nsquat<T>(pLHS / pRHS.x, pLHS / pRHS.y, pLHS / pRHS.z, pLHS / pRHS.w);
}

template <class T>
T dot(const nsquat<T> & left, const nsquat<T> & right)
{
	return left.a*right.a + left.b*right.b + left.c*right.c + left.d*right.d;
}

template<class T>
nsquat<T> conjugate(const nsquat<T> & quat)
{
	return nsquat<T>(quat).conjugate();
}

template<class T>
nsquat<T> inverse(const nsquat<T> & quat)
{
	return nsquat<T>(quat).invert();
}

template <class T>
nsquat<T> normalize(const nsquat<T> & quat)
{
	return nsquat<T>(quat).normalize();
}

template<class T>
nsquat<T> orientation(const nsmat3<T> & rotationMat3)
{
	return nsquat<T>().from(rotationMat3);
}

template<class T>
nsquat<T> orientation(const NSVec4<T> & axisAngle, bool pRads)
{
	return nsquat<T>().from(axisAngle, pRads);
}

template<class T>
nsquat<T> orientation(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool pRads)
{
	return nsquat<T>().from(euler,order,pRads);
}

template<class T>
nsquat<T> orientation(const NSVec3<T> & vec, const NSVec3<T> & toVec)
{
	return nsquat<T>().from(vec, toVec);
}

template<class T>
nsquat<T> orientation(const nsmat4<T> & transform)
{
	return nsquat<T>().from(transform);
}

template <class T>
nsquat<T> slerp(const nsquat<T> & lhs, const nsquat<T> & rhs, const T & scalingFactor)
{
	return nsquat<T>(lhs).slerp(rhs, scalingFactor);
}

template<class PUPer, class T>
void pup(PUPer & p, nsquat<T> & q4, const std::string & varName)
{
	pup(p, q4.x, varName + ".x"); pup(p, q4.y, varName + ".y"); pup(p, q4.z, varName + ".z"); pup(p, q4.w, varName + ".w");
}

#endif
