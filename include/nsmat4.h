#ifndef NSMAT4_H
#define NSMAT4_H

#include "nsmat3.h"

template <class T>
NSMat4<T> operator*(const int & pLHS, const NSMat4<T> & pRHS);

template <class T>
NSMat4<T> operator*(const float & pLHS, const NSMat4<T> & pRHS);

template <class T>
NSMat4<T> operator*(const double & pLHS, const NSMat4<T> & pRHS);

template<class T>
NSMat4<T> operator/(const int & pLHS, const NSMat4<T> & pRHS);

template <class T>
NSMat4<T> operator/(const float & pLHS, const NSMat4<T> & pRHS);

template <class T>
NSMat4<T> operator/(const double & pLHS, const NSMat4<T> & pRHS);

template <class T>
NSVec3<T> operator*(const NSVec3<T> & lhs, const NSMat4<T> & rhs);

template <class T>
NSVec3<T> operator/(const NSVec3<T> & lhs, const NSMat4<T> & rhs);

template <class T>
NSMat4<T> operator%(const NSVec3<T> & lhs, const NSMat4<T> & rhs);

//
template <class T>
T determinant(const NSMat4<T> & mat);

//
template <class T>
NSMat4<T> inverse(const NSMat4<T> mat);

template <class T>
NSMat4<T> ortho(const T & left, const T & right, const T & top, const T & bottom, const T & near, const T & far);

template <class T>
NSMat4<T> perspective(const T & fovAngle, const T & aspectRatio, const T & zNear, const T & zFar);

template<class T>
NSMat4<T> rotationMat4(const NSVec4<T> & axisAngle, bool rads = false);

template<class T>
NSMat4<T> rotationMat4(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool rads = false);

template<class T>
NSMat4<T> rotationMat4(const NSQuat<T> & orientation);

template<class T>
NSMat4<T> rotationMat4(const NSVec3<T> & vec, const NSVec3<T> & toVec);

template<class T>
NSMat4<T> rotationMat4(const NSMat4<T> & transform);

template<class T>
NSMat4<T> scalingMat4(const NSVec3<T> & scale);

template<class T>
NSMat4<T> scalingMat4(const NSMat3<T> & transform);

template<class T>
NSMat4<T> scalingMat4(const NSMat4<T> & transform);

template <class T>
NSMat4<T> transpose(const NSMat4<T> mat);

template<class T>
NSMat4<T> translationMat4(const NSVec3<T> & pos);

template<class T>
NSMat4<T> translationMat4(const NSVec4<T> & posw);

template<class T>
NSMat4<T> translationMat4(const NSMat4<T> & transform);

template<class PUPer, class T>
void pup(PUPer & p, NSMat4<T> & m4);

template <class T>
struct NSMat4
{
	//
	NSMat4()
	{
		setIdentity();
	}

	//
	NSMat4(const NSMat4 & copy)
	{
		set(copy.data[0], copy.data[1], copy.data[2], copy.data[3]);
	}

	NSMat4(const T & val)
	{
		set(val);
	}

	//
	NSMat4(
		const T & a, const T & b, const T & c, const T & d,
		const T & e, const T & f, const T & g, const T & h, 
		const T & i, const T & j, const T & k, const T & l,
		const T & m, const T & n, const T & o, const T & p)
	{
		set(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p);
	}

	//
	NSMat4(const NSVec4<T> & row1, const NSVec4<T> & row2, const NSVec4<T> & row3, const NSVec4<T> & row4)
	{
		set(row1, row2, row3, row4);
	}

	//
	NSMat4(const NSMat3<T> & basis)
	{
		set(basis);
		data[0][3] = static_cast<T>(0); data[1][3] = static_cast<T>(0); data[2][3] = static_cast<T>(0);
		data[3][0] = static_cast<T>(0); data[3][1] = static_cast<T>(0); data[3][2] = static_cast<T>(0); data[3][3] = static_cast<T>(1);
	}

	//
	NSMat3<T> basis() const
	{
		return NSMat3<T>(
			data[0][0], data[0][1], data[0][2],
			data[1][0], data[1][1], data[1][2],
			data[2][0], data[2][1], data[2][2]);
	}

	T * dataPtr()
	{
		return &data[0][0];
	}

	T determinant() const
	{
		return
			data[0][3]*data[1][2]*data[2][1]*data[3][0] - data[0][2]*data[1][3]*data[2][1]*data[3][0] - data[0][3]*data[1][1]*data[2][2]*data[3][0] + data[0][1]*data[1][3]*data[2][2]*data[3][0] +
			data[0][2]*data[1][1]*data[2][3]*data[3][0] - data[0][1]*data[1][2]*data[2][3]*data[3][0] - data[0][3]*data[1][2]*data[2][0]*data[3][1] + data[0][2]*data[1][3]*data[2][0]*data[3][1] +
			data[0][3]*data[1][0]*data[2][2]*data[3][1] - data[0][0]*data[1][3]*data[2][2]*data[3][1] - data[0][2]*data[1][0]*data[2][3]*data[3][1] + data[0][0]*data[1][2]*data[2][3]*data[3][1] +
			data[0][3]*data[1][1]*data[2][0]*data[3][2] - data[0][1]*data[1][3]*data[2][0]*data[3][2] - data[0][3]*data[1][0]*data[2][1]*data[3][2] + data[0][0]*data[1][3]*data[2][1]*data[3][2] +
			data[0][1]*data[1][0]*data[2][3]*data[3][2] - data[0][0]*data[1][1]*data[2][3]*data[3][2] - data[0][2]*data[1][1]*data[2][0]*data[3][3] + data[0][1]*data[1][2]*data[2][0]*data[3][3] +
			data[0][2]*data[1][0]*data[2][1]*data[3][3] - data[0][0]*data[1][2]*data[2][1]*data[3][3] - data[0][1]*data[1][0]*data[2][2]*data[3][3] + data[0][0]*data[1][1]*data[2][2]*data[3][3];
	}

	NSMat4<T> & invert()
	{
		T det = determinant();

		if (det == static_cast<T>(0))
			return set(0); // indicate failure

		NSMat4 ret;
		ret[0][0] = data[1][2]*data[2][3]*data[3][1] - data[1][3]*data[2][2]*data[3][1] + data[1][3]*data[2][1]*data[3][2] - data[1][1]*data[2][3]*data[3][2] - data[1][2]*data[2][1]*data[3][3] + data[1][1]*data[2][2]*data[3][3];
		ret[0][1] = data[0][3]*data[2][2]*data[3][1] - data[0][2]*data[2][3]*data[3][1] - data[0][3]*data[2][1]*data[3][2] + data[0][1]*data[2][3]*data[3][2] + data[0][2]*data[2][1]*data[3][3] - data[0][1]*data[2][2]*data[3][3];
		ret[0][2] = data[0][2]*data[1][3]*data[3][1] - data[0][3]*data[1][2]*data[3][1] + data[0][3]*data[1][1]*data[3][2] - data[0][1]*data[1][3]*data[3][2] - data[0][2]*data[1][1]*data[3][3] + data[0][1]*data[1][2]*data[3][3];
		ret[0][3] = data[0][3]*data[1][2]*data[2][1] - data[0][2]*data[1][3]*data[2][1] - data[0][3]*data[1][1]*data[2][2] + data[0][1]*data[1][3]*data[2][2] + data[0][2]*data[1][1]*data[2][3] - data[0][1]*data[1][2]*data[2][3];
		ret[1][0] = data[1][3]*data[2][2]*data[3][0] - data[1][2]*data[2][3]*data[3][0] - data[1][3]*data[2][0]*data[3][2] + data[1][0]*data[2][3]*data[3][2] + data[1][2]*data[2][0]*data[3][3] - data[1][0]*data[2][2]*data[3][3];
		ret[1][1] = data[0][2]*data[2][3]*data[3][0] - data[0][3]*data[2][2]*data[3][0] + data[0][3]*data[2][0]*data[3][2] - data[0][0]*data[2][3]*data[3][2] - data[0][2]*data[2][0]*data[3][3] + data[0][0]*data[2][2]*data[3][3];
		ret[1][2] = data[0][3]*data[1][2]*data[3][0] - data[0][2]*data[1][3]*data[3][0] - data[0][3]*data[1][0]*data[3][2] + data[0][0]*data[1][3]*data[3][2] + data[0][2]*data[1][0]*data[3][3] - data[0][0]*data[1][2]*data[3][3];
		ret[1][3] = data[0][2]*data[1][3]*data[2][0] - data[0][3]*data[1][2]*data[2][0] + data[0][3]*data[1][0]*data[2][2] - data[0][0]*data[1][3]*data[2][2] - data[0][2]*data[1][0]*data[2][3] + data[0][0]*data[1][2]*data[2][3];
		ret[2][0] = data[1][1]*data[2][3]*data[3][0] - data[1][3]*data[2][1]*data[3][0] + data[1][3]*data[2][0]*data[3][1] - data[1][0]*data[2][3]*data[3][1] - data[1][1]*data[2][0]*data[3][3] + data[1][0]*data[2][1]*data[3][3];
		ret[2][1] = data[0][3]*data[2][1]*data[3][0] - data[0][1]*data[2][3]*data[3][0] - data[0][3]*data[2][0]*data[3][1] + data[0][0]*data[2][3]*data[3][1] + data[0][1]*data[2][0]*data[3][3] - data[0][0]*data[2][1]*data[3][3];
		ret[2][2] = data[0][1]*data[1][3]*data[3][0] - data[0][3]*data[1][1]*data[3][0] + data[0][3]*data[1][0]*data[3][1] - data[0][0]*data[1][3]*data[3][1] - data[0][1]*data[1][0]*data[3][3] + data[0][0]*data[1][1]*data[3][3];
		ret[2][3] = data[0][3]*data[1][1]*data[2][0] - data[0][1]*data[1][3]*data[2][0] - data[0][3]*data[1][0]*data[2][1] + data[0][0]*data[1][3]*data[2][1] + data[0][1]*data[1][0]*data[2][3] - data[0][0]*data[1][1]*data[2][3];
		ret[3][0] = data[1][2]*data[2][1]*data[3][0] - data[1][1]*data[2][2]*data[3][0] - data[1][2]*data[2][0]*data[3][1] + data[1][0]*data[2][2]*data[3][1] + data[1][1]*data[2][0]*data[3][2] - data[1][0]*data[2][1]*data[3][2];
		ret[3][1] = data[0][1]*data[2][2]*data[3][0] - data[0][2]*data[2][1]*data[3][0] + data[0][2]*data[2][0]*data[3][1] - data[0][0]*data[2][2]*data[3][1] - data[0][1]*data[2][0]*data[3][2] + data[0][0]*data[2][1]*data[3][2];
		ret[3][2] = data[0][2]*data[1][1]*data[3][0] - data[0][1]*data[1][2]*data[3][0] - data[0][2]*data[1][0]*data[3][1] + data[0][0]*data[1][2]*data[3][1] + data[0][1]*data[1][0]*data[3][2] - data[0][0]*data[1][1]*data[3][2];
		ret[3][3] = data[0][1]*data[1][2]*data[2][0] - data[0][2]*data[1][1]*data[2][0] + data[0][2]*data[1][0]*data[2][1] - data[0][0]*data[1][2]*data[2][1] - data[0][1]*data[1][0]*data[2][2] + data[0][0]*data[1][1]*data[2][2];

		ret /= determinant();
		return (*this = ret);
	}

	NSMat4<T> & orthoFrom(const T & left, const T & right, const T & top, const T & bottom, const T & near, const T & far)
	{
		T w = right - left;
		T h = top - bottom;
		T p = far - near;

		T x = (right + left) / w;
		T y = (top + bottom) / h;
		T z = (far + near) / p;

		setIdentity();
		setColumn(3, -x, -y, -z, 1);
		data[0][0] = static_cast<T>(2) / w;
		data[1][1] = static_cast<T>(2) / h;
		data[2][2] = static_cast<T>(-2) / p;
		return *this;
	}

	NSMat4<T> & perspectiveFrom(const T & fovAngle, const T & aspectRatio, const T & zNear, const T & zFar)
	{
		T zRange = zNear - zFar;
		T usefulNum = static_cast<T>(tan(radians(fovAngle) / static_cast<T>(2)));
		setIdentity();
		data[0][0] = static_cast<T>(1) / (aspectRatio * usefulNum);
		data[1][1] = static_cast<T>(1) / usefulNum;
		data[2][2] = (-zNear - zFar) / zRange;
		data[2][3] = static_cast<T>(2) * zFar * zNear / zRange;
		data[3][2] = static_cast<T>(1);
		data[3][3] = static_cast<T>(0);

		return *this;
	}

	NSVec3<T> right() const
	{
		return normalize(NSVec3<T>(data[0].x, data[0].y, data[0].z));
	}

	NSMat4<T> & rotationFrom(const NSVec4<T> & axisAngle, bool rads = false)
	{
		// http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm

		T angle = axisAngle.a;

		if (!rads)
			angle = radians(angle);

		T c = static_cast<T>(std::cos(angle));
		T s = static_cast<T>(std::sin(angle));
		T t = static_cast<T>(1) - c;

		data[0][0] = c + axisAngle.x * axisAngle.x * t;
		data[1][1] = c + axisAngle.y * axisAngle.y * t;
		data[2][2] = c + axisAngle.z * axisAngle.z * t;

		T tmp1 = axisAngle.x * axisAngle.y * t;
		T tmp2 = axisAngle.z * s;
		data[1][0] = tmp1 + tmp2;
		data[0][1] = tmp1 - tmp2;

		tmp1 = axisAngle.x * axisAngle.z * t;
		tmp2 = axisAngle.y * s;
		data[2][0] = tmp1 - tmp2;
		data[0][2] = tmp1 + tmp2;

		tmp1 = axisAngle.y * axisAngle.z * t;
		tmp2 = axisAngle.x * s;
		data[2][1] = tmp1 + tmp2;
		data[1][2] = tmp1 - tmp2;

		data[0][3] = static_cast<T>(0); data[1][3] = static_cast<T>(0); data[2][3] = static_cast<T>(0);
		data[3][0] = static_cast<T>(0); data[3][1] = static_cast<T>(0); data[3][2] = static_cast<T>(0); data[3][3] = static_cast<T>(1);

		return *this;
	}

	NSMat4<T> & rotationFrom(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order , bool rads = false)
	{
		T cb, sb, ch, sh, ca, sa;
		if (!rads)
		{
			NSVec3<T> eulRads(radians(euler));
			cb = static_cast<T>(std::cos(eulRads.x));
			sb = static_cast<T>(std::sin(eulRads.x));
			ch = static_cast<T>(std::cos(eulRads.y));
			sh = static_cast<T>(std::sin(eulRads.y));
			ca = static_cast<T>(std::cos(eulRads.z));
			sa = static_cast<T>(std::sin(eulRads.z));
		}
		else
		{
			cb = static_cast<T>(std::cos(euler.x));
			sb = static_cast<T>(std::sin(euler.x));
			ch = static_cast<T>(std::cos(euler.y));
			sh = static_cast<T>(std::sin(euler.y));
			ca = static_cast<T>(std::cos(euler.z));
			sa = static_cast<T>(std::sin(euler.z));
		}

		switch (order)
		{
		case(NSVec3<T>::XYZ) :
		{
			data[0][0] = ch * ca;
			data[0][1] = -ch * sa;
			data[0][2] = sh;

			data[1][0] = cb * sa + sb * ca * sh;
			data[1][1] = cb * ca - sb * sa * sh;
			data[1][2] = -sb * ch;

			data[2][0] = sb * sa - cb * ca * sh;
			data[2][1] = sb * ca + cb * sa * sh;
			data[2][2] = cb * ch;
			break;
		}
		case(NSVec3<T>::XZY) :
		{
			data[0][0] = ch * ca;
			data[0][1] = -sa;
			data[0][2] = sh * ca;

			data[1][0] = cb * ch * sa + sb * sh;
			data[1][1] = cb * ca;
			data[1][2] = cb * sh * sa - sb * ch;

			data[2][0] = sb * ch * sa - cb * sh;
			data[2][1] = sb * ca;
			data[2][2] = sb * sh * sa + cb * ch;
			break;
		}
		case(NSVec3<T>::YXZ) :
		{
			data[0][0] = ch * ca + sh * sa * sb;
			data[0][1] = sh * ca * sb - ch * sa;
			data[0][2] = cb * sh;

			data[1][0] = cb * sa;
			data[1][1] = cb * ca;
			data[1][2] = -sb;

			data[2][0] = ch * sa * sb - sh * ca;
			data[2][1] = sh * sa + ch * ca * sb;
			data[2][2] = cb * ch;
			break;
		}
		case(NSVec3<T>::YZX) :
		{
			data[0][0] = ch * ca;
			data[0][1] = sb * sh - cb * ch * sa;
			data[0][2] = sb * ch * sa + cb * sh;

			data[1][0] = sa;
			data[1][1] = cb * ca;
			data[1][2] = -sb * ca;

			data[2][0] = -sh * ca;
			data[2][1] = cb * sh * sa + sb * ch;
			data[2][2] = cb * ch - sb * sh * sa;
			break;
		}
		case(NSVec3<T>::ZXY) :
		{
			data[0][0] = ch * ca - sh * sa * sb;
			data[0][1] = -cb * sa;
			data[0][2] = sh * ca + ch * sa * sb;

			data[1][0] = ch * sa + sh * ca * sb;
			data[1][1] = cb * ca;
			data[1][2] = sh * sa - ch * ca * sb;

			data[2][0] = -cb * sh;
			data[2][1] = sb;
			data[2][2] = cb * ch;
			break;
		}
		case(NSVec3<T>::ZYX) :
		{
			data[0][0] = ch * ca;
			data[0][1] = sb * ca * sh - cb * sa;
			data[0][2] = cb * ca * sh + sb * sa;

			data[1][0] = ch * sa;
			data[1][1] = sb * sa * sh + cb * ca;
			data[1][2] = cb * sa * sh - sb * ca;

			data[2][0] = -sh;
			data[2][1] = sb * ch;
			data[2][2] = cb * ch;
			break;
		}
		}

		data[0][3] = static_cast<T>(0); data[1][3] = static_cast<T>(0); data[2][3] = static_cast<T>(0);
		data[3][0] = static_cast<T>(0); data[3][1] = static_cast<T>(0); data[3][2] = static_cast<T>(0); data[3][3] = static_cast<T>(1);
		return roundToZero();
	}

	NSMat4<T> & rotationFrom(const NSQuat<T> & orientation)
	{
		T x2 = orientation.x + orientation.x, y2 = orientation.y + orientation.y, z2 = orientation.z + orientation.z;
		T xx = orientation.x * x2, xy = orientation.x * y2, xz = orientation.x * z2;
		T yy = orientation.y * y2, yz = orientation.y * z2, zz = orientation.z * z2;
		T wx = orientation.w * x2, wy = orientation.w * y2, wz = orientation.w * z2;

		data[0][0] = 1 - (yy + zz);
		data[0][1] = xy - wz;
		data[0][2] = xz + wy;

		data[1][0] = xy + wz;
		data[1][1] = 1 - (xx + zz);
		data[1][2] = yz - wx;

		data[2][0] = xz - wy;
		data[2][1] = yz + wx;
		data[2][2] = 1 - (xx + yy);

		data[0][3] = static_cast<T>(0); data[1][3] = static_cast<T>(0); data[2][3] = static_cast<T>(0);
		data[3][0] = static_cast<T>(0); data[3][1] = static_cast<T>(0); data[3][2] = static_cast<T>(0); data[3][3] = static_cast<T>(1);
		return *this;
	}

	NSMat4<T> & rotationFrom(const NSMat3<T> & transform)
	{
		set(transform);
		data[0][3] = static_cast<T>(0); data[1][3] = static_cast<T>(0); data[2][3] = static_cast<T>(0);
		data[3][0] = static_cast<T>(0); data[3][1] = static_cast<T>(0); data[3][2] = static_cast<T>(0); data[3][3] = static_cast<T>(1);
		data[0].normalize();
		data[1].normalize();
		data[2].normalize();
		return *this;
	}

	NSMat4<T> & rotationFrom(const NSMat4<T> & transform)
	{
		*this = transform;

		// Set remaining components to identity
		data[0][3] = static_cast<T>(0); data[1][3] = static_cast<T>(0); data[2][3] = static_cast<T>(0);
		data[3][0] = static_cast<T>(0); data[3][1] = static_cast<T>(0); data[3][2] = static_cast<T>(0); data[3][3] = static_cast<T>(1);

		data[0].normalize();
		data[1].normalize();
		data[2].normalize();
		return *this;
	}

	NSMat4<T> & rotationFrom(const NSVec3<T> & vec, const NSVec3<T> & toVec)
	{
		/* http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm */
		NSVec3<T> vs = cross(vec, toVec);
		T ca = vec * toVec;
		NSVec3<T> v = normalize(vs);
		NSVec3<T> vt = v * (static_cast<T>(1) - ca);

		data[0][0] = vt.x * v.x + ca;
		data[1][1] = vt.y * v.y + ca;
		data[2][2] = vt.z * v.z + ca;
		vt.x *= v.y;
		vt.z *= v.x;
		vt.y *= v.z;
		data[0][1] = vt.x - vs.z;
		data[0][2] = vt.z + vs.y;
		data[1][0] = vt.x + vs.z;
		data[1][2] = vt.y - vs.x;
		data[2][0] = vt.z - vs.y;
		data[2][1] = vt.y + vs.x;

		// Set remaining components
		data[0][3] = static_cast<T>(0); data[1][3] = static_cast<T>(0); data[2][3] = static_cast<T>(0);
		data[3][0] = static_cast<T>(0); data[3][1] = static_cast<T>(0); data[3][2] = static_cast<T>(0); data[3][3] = static_cast<T>(1);
		return *this;
	}

	NSMat4<T> & roundToZero()
	{
		data[0].roundToZero();
		data[1].roundToZero();
		data[2].roundToZero();
		data[3].roundToZero();
		return *this;
	}

	NSMat4<T> & scalingFrom(const NSVec3<T> & scale)
	{
		setIdentity();
		data[0][0] = scale.x; data[1][1] = scale.y; data[2][2] = scale.z;
		return *this;
	}

	NSMat4<T> & scalingFrom(const NSMat3<T> & transform)
	{
		NSVec3<T> scalingVec(transform[0].length(), transform[1].length(), transform[2].length());
		return scalingFrom(scalingVec);
	}

	NSMat4<T> & scalingFrom(const NSMat4<T> & transform)
	{
		NSVec3<T> scalingVec;
		scalingVec.x = sqrt(transform[0][0] * transform[0][0] + transform[0][1] * transform[0][1] + transform[0][2] * transform[0][2]);
		scalingVec.y = sqrt(transform[1][0] * transform[1][0] + transform[1][1] * transform[1][1] + transform[1][2] * transform[1][2]);
		scalingVec.z = sqrt(transform[2][0] * transform[2][0] + transform[2][1] * transform[2][1] + transform[2][2] * transform[2][2]);
		return scalingFrom(scalingVec);
	}

	NSMat4<T> & set(const T & val)
	{
		data[0].set(val);
		data[1].set(val);
		data[2].set(val);
		data[3].set(val);
		return *this;
	}

	NSMat4<T> & set(
		const T & a, const T & b, const T & c, const T & d,
		const T & e, const T & f, const T & g, const T & h,
		const T & i, const T & j, const T & k, const T & l,
		const T & m, const T & n, const T & o, const T & p)
	{
		data[0][0] = a; data[0][1] = b; data[0][2] = c; data[0][3] = d; 
		data[1][0] = e; data[1][1] = f; data[1][2] = g; data[1][3] = h;
		data[2][0] = i; data[2][1] = j; data[2][2] = k; data[2][3] = l;
		data[3][0] = m; data[3][1] = n; data[3][2] = o; data[3][3] = p;
		return *this;
	}

	NSMat4<T> & set(
		const NSVec4<T> & row1, 
		const NSVec4<T> & row2, 
		const NSVec4<T> & row3, 
		const NSVec4<T> & row4)
	{
		data[0] = row1;
		data[1] = row2;
		data[2] = row3;
		data[3] = row4;
		return *this;
	}

	NSMat4<T> & set(const NSMat3<T> & basis)
	{
		data[0][0] = basis[0][0]; data[0][1] = basis[0][1]; data[0][2] = basis[0][2];
		data[1][0] = basis[1][0]; data[1][1] = basis[1][1]; data[1][2] = basis[1][2];
		data[2][0] = basis[2][0]; data[2][1] = basis[2][1]; data[2][2] = basis[2][2];
		return *this;
	}

	NSMat4<T> & setColumn(const uint & i, const T & x, const T & y, const T & z, const T & w)
	{
		(*this)[0][i] = x; (*this)[1][i] = y; (*this)[2][i] = z; (*this)[3][i] = w;
		return *this;
	}

	NSMat4<T> & setColumn(const uint & i, const NSVec4<T> & col)
	{
		(*this)[0][i] = col.x; (*this)[1][i] = col.y; (*this)[2][i] = col.z; (*this)[3][i] = col.w;
		return *this;
	}

	//
	NSMat4<T> & setIdentity()
	{
		data[0].set(static_cast<T>(1), static_cast<T>(0), static_cast<T>(0), static_cast<T>(0));
		data[1].set(static_cast<T>(0), static_cast<T>(1), static_cast<T>(0), static_cast<T>(0));
		data[2].set(static_cast<T>(0), static_cast<T>(0), static_cast<T>(1), static_cast<T>(0));
		data[3].set(static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(1));
		return *this;
	}

	NSVec3<T> target() const
	{
		return normalize(NSVec3<T>(data[2].x,data[2].y,data[2].z));
	}

	NSMat4<T> & translationFrom(const NSVec3<T> & pos)
	{
		return setColumn(3, pos.x, pos.y, pos.z, static_cast<T>(1));
	}

	NSMat4<T> & translationFrom(const NSVec4<T> & posw)
	{
		return setColumn(3, posw.x, posw.y, posw.z, static_cast<T>(1));
	}

	NSMat4<T> & translationFrom(const NSMat4<T> & transform)
	{
		*this = transform;
		data[3][3] = static_cast<T>(1);
		return set(NSMat3<T>());
	}

	NSMat4<T> & transpose()
	{
		T tmp;
		tmp = data[1][0]; data[1][0] = data[0][1]; data[0][1] = tmp;
		tmp = data[2][0]; data[2][0] = data[0][2]; data[0][2] = tmp;
		tmp = data[2][1]; data[2][1] = data[1][2]; data[1][2] = tmp;

		tmp = data[3][0]; data[3][0] = data[0][3]; data[0][3] = tmp;
		tmp = data[3][1]; data[3][1] = data[1][3]; data[1][3] = tmp;
		tmp = data[3][2]; data[3][2] = data[2][3]; data[2][3] = tmp;
		return *this;
	}

	std::string toString(bool newline = true) const
	{
		std::ostringstream ss;
		std::string lc = ";";

		if (newline)
			lc += '\n';

		ss << "[" << data[0][0] << " " << data[0][1] << " " << data[0][2] << " " << data[0][3] << lc;
		ss << " " << data[1][0] << " " << data[1][1] << " " << data[1][2] << " " << data[1][3] << lc;
		ss << " " << data[2][0] << " " << data[2][1] << " " << data[2][2] << " " << data[2][3] << lc;
		ss << " " << data[3][0] << " " << data[3][1] << " " << data[3][2] << " " << data[3][3] << "]";
		return ss.str();
	}

	NSVec3<T> up() const
	{
		return normalize(NSVec3<T>(data[1].x, data[1].y, data[1].z));
	}

	//
	NSMat4<T> operator*(const NSMat4<T> & rhs) const
	{
		NSMat4<T> ret;
		ret[0][0] = data[0][0] * rhs[0][0] + data[0][1] * rhs[1][0] + data[0][2] * rhs[2][0] + data[0][3] * rhs[3][0];
		ret[0][1] = data[0][0] * rhs[0][1] + data[0][1] * rhs[1][1] + data[0][2] * rhs[2][1] + data[0][3] * rhs[3][1];
		ret[0][2] = data[0][0] * rhs[0][2] + data[0][1] * rhs[1][2] + data[0][2] * rhs[2][2] + data[0][3] * rhs[3][2];
		ret[0][3] = data[0][0] * rhs[0][3] + data[0][1] * rhs[1][3] + data[0][2] * rhs[2][3] + data[0][3] * rhs[3][3];

		ret[1][0] = data[1][0] * rhs[0][0] + data[1][1] * rhs[1][0] + data[1][2] * rhs[2][0] + data[1][3] * rhs[3][0];
		ret[1][1] = data[1][0] * rhs[0][1] + data[1][1] * rhs[1][1] + data[1][2] * rhs[2][1] + data[1][3] * rhs[3][1];
		ret[1][2] = data[1][0] * rhs[0][2] + data[1][1] * rhs[1][2] + data[1][2] * rhs[2][2] + data[1][3] * rhs[3][2];
		ret[1][3] = data[1][0] * rhs[0][3] + data[1][1] * rhs[1][3] + data[1][2] * rhs[2][3] + data[1][3] * rhs[3][3];

		ret[2][0] = data[2][0] * rhs[0][0] + data[2][1] * rhs[1][0] + data[2][2] * rhs[2][0] + data[2][3] * rhs[3][0];
		ret[2][1] = data[2][0] * rhs[0][1] + data[2][1] * rhs[1][1] + data[2][2] * rhs[2][1] + data[2][3] * rhs[3][1];
		ret[2][2] = data[2][0] * rhs[0][2] + data[2][1] * rhs[1][2] + data[2][2] * rhs[2][2] + data[2][3] * rhs[3][2];
		ret[2][3] = data[2][0] * rhs[0][3] + data[2][1] * rhs[1][3] + data[2][2] * rhs[2][3] + data[2][3] * rhs[3][3];

		ret[3][0] = data[3][0] * rhs[0][0] + data[3][1] * rhs[1][0] + data[3][2] * rhs[2][0] + data[3][3] * rhs[3][0];
		ret[3][1] = data[3][0] * rhs[0][1] + data[3][1] * rhs[1][1] + data[3][2] * rhs[2][1] + data[3][3] * rhs[3][1];
		ret[3][2] = data[3][0] * rhs[0][2] + data[3][1] * rhs[1][2] + data[3][2] * rhs[2][2] + data[3][3] * rhs[3][2];
		ret[3][3] = data[3][0] * rhs[0][3] + data[3][1] * rhs[1][3] + data[3][2] * rhs[2][3] + data[3][3] * rhs[3][3];
		return ret;
	}

	NSMat4<T> operator/(const NSMat4<T> & rhs) const
	{
		return *this * inverse(rhs);
	}

	NSMat4<T> operator%(const NSMat4<T> & rhs) const
	{
		NSMat4<T> ret;
		ret.data[0] = data[0] % rhs.data[0];
		ret.data[1] = data[1] % rhs.data[1];
		ret.data[2] = data[2] % rhs.data[2];
		ret.data[3] = data[3] % rhs.data[3];
		return ret;
	}

	NSVec4<T> operator*(const NSVec4<T> & rhs) const
	{
		return NSVec4<T>(data[0] * rhs, data[1] * rhs, data[2] * rhs, data[3] * rhs);
	}

	NSMat4<T> operator%(const NSVec4<T> & rhs) const
	{
		return NSMat4<T>(data[0] % rhs, data[1] % rhs, data[2] % rhs, data[3] % rhs);
	}

	NSMat4<T> operator/(const NSVec4<T> & rhs) const
	{
		NSMat4<T> ret;
		ret.data[0] = data[0] / rhs[0];
		ret.data[1] = data[1] / rhs[1];
		ret.data[2] = data[2] / rhs[2];
		ret.data[3] = data[3] / rhs[3];
		return ret;
	}

	NSMat4<T> operator*(const T & rhs) const
	{
		NSMat4<T> ret;
		ret.data[0] = data[0] * rhs;
		ret.data[1] = data[1] * rhs;
		ret.data[2] = data[2] * rhs;
		ret.data[3] = data[3] * rhs;
		return ret;
	}

	NSMat4<T> operator/(const T & rhs) const
	{
		NSMat4<T> ret;
		ret.data[0] = data[0] / rhs;
		ret.data[1] = data[1] / rhs;
		ret.data[2] = data[2] / rhs;
		ret.data[3] = data[3] / rhs;
		return ret;
	}

	NSMat4<T> operator+(const NSMat4<T> & rhs) const
	{
		NSMat4<T> ret;
		ret.data[0] = data[0] + rhs.data[0];
		ret.data[1] = data[1] + rhs.data[1];
		ret.data[2] = data[2] + rhs.data[2];
		ret.data[3] = data[3] + rhs.data[3];
		return ret;
	}

	NSMat4<T> operator-(const NSMat4<T> & rhs) const
	{
		NSMat4<T> ret;
		ret.data[0] = data[0] - rhs.data[0];
		ret.data[1] = data[1] - rhs.data[1];
		ret.data[2] = data[2] - rhs.data[2];
		ret.data[3] = data[3] - rhs.data[3];
		return ret;
	}

	bool operator==(const NSMat4<T> & rhs) const
	{
		return (data[0] == rhs.data[0] && data[1] == rhs.data[1] && data[2] == rhs.data[2] && data[3] == rhs.data[3]);
	}

	bool operator!=(const NSMat4<T> & rhs) const
	{
		return !(*this == rhs);
	}

	NSMat4<T> & operator=(const NSMat4<T> & rhs)
	{
		if (this == &rhs)
			return *this;

		data[0] = rhs.data[0];
		data[1] = rhs.data[1];
		data[2] = rhs.data[2];
		data[3] = rhs.data[3];
		return *this;
	}

	NSMat4<T> & operator*=(const NSMat4<T> & rhs)
	{
		*this = *this * rhs;
		return *this;
	}

	NSMat4<T> & operator/=(const NSMat4<T> & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	NSMat4<T> & operator%=(const NSMat4<T> & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	NSMat4<T> & operator%=(const NSVec4<T> & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	NSMat4<T> & operator/=(const NSVec4<T> & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	NSMat4<T> & operator*=(const T & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	NSMat4<T> & operator/=(const T & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	NSMat4<T> operator++(int)
	{
		NSMat4<T> ret(*this);
		++(*this);
		return ret;
	}

	NSMat4<T> operator--(int)
	{
		NSMat4<T> ret(*this);
		--(*this);
		return ret;
	}

	NSMat4<T> & operator++()
	{
		++data[0]; ++data[1]; ++data[2]; ++data[3];
		return *this;
	}

	NSMat4<T> & operator--()
	{
		--data[0]; --data[1]; --data[2]; --data[3];
		return *this;
	}

	NSMat4<T> & operator+=(const NSMat4<T> & rhs)
	{
		*this = *this + rhs;
		return *this;
	}

	NSMat4<T> & operator-=(const NSMat4<T> & rhs)
	{
		*this = *this - rhs;
		return *this;
	}

	//
	const NSVec4<T> & operator[](const uint & pVal) const
	{
		if (pVal > 3)
			throw(std::out_of_range("mat4 index out of range"));
		return data[pVal];
	}

	//
	NSVec4<T> & operator[](const uint & pVal)
	{
		if (pVal > 3)
			throw(std::out_of_range("mat4 index out of range"));
		return data[pVal];
	}

	NSVec4<T> operator()(const uint & pVal) const
	{
		return NSVec4<T>(data[0][pVal], data[1][pVal], data[2][pVal], data[3][pVal]);
	}

private:
	NSVec4<T> data[4];
};

template <class T>
NSMat4<T> operator*(const int & pLHS, const NSMat4<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSMat4<T> operator*(const float & pLHS, const NSMat4<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
NSMat4<T> operator*(const double & pLHS, const NSMat4<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template<class T>
NSMat4<T> operator/(const int & pLHS, const NSMat4<T> & pRHS)
{
	return NSMat4<T>(pLHS / pRHS[0], pLHS / pRHS[1], pLHS / pRHS[2]);
}

template <class T>
NSMat4<T> operator/(const float & pLHS, const NSMat4<T> & pRHS)
{
	return NSMat4<T>(pLHS / pRHS[0], pLHS / pRHS[1], pLHS / pRHS[2]);
}

template <class T>
NSMat4<T> operator/(const double & pLHS, const NSMat4<T> & pRHS)
{
	return NSMat4<T>(pLHS / pRHS[0], pLHS / pRHS[1], pLHS / pRHS[2]);
}

template <class T>
NSVec3<T> operator*(const NSVec3<T> & lhs, const NSMat4<T> & rhs)
{
	return NSVec3<T>(
		lhs[0] * rhs[0][0] + lhs[1] * rhs[1][0] + lhs[2] * rhs[2][0] + lhs[3] * rhs[3][0],
		lhs[0] * rhs[0][1] + lhs[1] * rhs[1][1] + lhs[2] * rhs[2][1] + lhs[3] * rhs[3][1],
		lhs[0] * rhs[0][2] + lhs[1] * rhs[1][2] + lhs[2] * rhs[2][2] + lhs[3] * rhs[3][2]);
}

template <class T>
NSVec3<T> operator/(const NSVec3<T> & lhs, const NSMat4<T> & rhs)
{
	return lhs * inverse(rhs);
}

template <class T>
NSMat4<T> operator%(const NSVec3<T> & lhs, const NSMat4<T> & rhs)
{
	return NSMat4<T>(rhs[0] * lhs[0], rhs[1] * lhs[1], rhs[2] * lhs[2], rhs[3] * lhs[3]);
}

template <class T>
T determinant(const NSMat4<T> & mat)
{
	return mat.determinant();
}

template <class T>
NSMat4<T> ortho(const T & left, const T & right, const T & top, const T & bottom, const T & near, const T & far)
{
	return NSMat4<T>().orthoFrom(left, right, top, bottom, near, far);
}

template <class T>
NSMat4<T> perspective(const T & fovAngle, const T & aspectRatio, const T & zNear, const T & zFar)
{
	return NSMat4<T>().perspectiveFrom(fovAngle, aspectRatio, zNear, zFar);
}

template<class T>
NSMat4<T> rotationMat4(const NSVec4<T> & axisAngle, bool rads)
{
	return NSMat4<T>().rotationFrom(axisAngle, rads);
}

template<class T>
NSMat4<T> rotationMat4(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool rads)
{
	return NSMat4<T>().rotationFrom(euler, order, rads);
}

template<class T>
NSMat4<T> rotationMat4(const NSQuat<T> & orientation)
{
	return NSMat4<T>().rotationFrom(orientation);
}

template<class T>
NSMat4<T> rotationMat4(const NSVec3<T> & vec, const NSVec3<T> & toVec)
{
	return NSMat4<T>().rotationFrom(vec, toVec);
}

template<class T>
NSMat4<T> rotationMat4(const NSMat3<T> & transform)
{
	return NSMat4<T>().rotationFrom(transform);
}

template<class T>
NSMat4<T> rotationMat4(const NSMat4<T> & transform)
{
	return NSMat4<T>().rotationFrom(transform);
}

template<class T>
NSMat4<T> scalingMat4(const NSVec3<T> & scale)
{
	return NSMat4<T>().scalingFrom(scale);
}

template<class T>
NSMat4<T> scalingMat4(const NSMat3<T> & transform)
{
	return NSMat4<T>().scalingFrom(transform);
}

template<class T>
NSMat4<T> scalingMat4(const NSMat4<T> & transform)
{
	return NSMat4<T>().scalingFrom(transform);
}

template<class T>
NSMat4<T> translationMat4(const NSVec3<T> & pos)
{
	return NSMat4<T>().translationFrom(pos);
}

template<class T>
NSMat4<T> translationMat4(const NSVec4<T> & posw)
{
	return NSMat4<T>().translationFrom(posw);
}

template<class T>
NSMat4<T> translationMat4(const NSMat4<T> & transform)
{
	return NSMat4<T>().translationFrom(transform);
}

template <class T>
NSMat4<T> transpose(NSMat4<T> mat)
{
	return mat.transpose();
}

template <class T>
NSMat4<T> inverse(NSMat4<T> mat)
{
	return mat.invert();
}

template<class PUPer, class T>
void pup(PUPer & p, NSMat4<T> & m4, const std::string & varName)
{
	pup(p, m4[0], varName + "[0]"); pup(p, m4[1], varName + "[1]"); pup(p, m4[2], varName + "[2]"); pup(p, m4[3], varName + "[3]");
}

#endif
