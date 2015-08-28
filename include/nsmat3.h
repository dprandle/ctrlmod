#ifndef NSMAT3_H
#define NSMAT3_H

#include "nsmat2.h"

template <class T>
NSVec3<T> operator*(const NSVec3<T> & lhs, const nsmat3<T> & rhs);

template <class T>
NSVec3<T> operator/(const NSVec3<T> & lhs, const nsmat3<T> & rhs);

template <class T>
nsmat3<T> operator%(const NSVec3<T> & lhs, const nsmat3<T> & rhs);

template <class T>
nsmat3<T> operator*(const int32_t & pLHS, const nsmat3<T> & pRHS);

template <class T>
nsmat3<T> operator*(const float & pLHS, const nsmat3<T> & pRHS);

template <class T>
nsmat3<T> operator*(const double & pLHS, const nsmat3<T> & pRHS);

template<class T>
nsmat3<T> operator/(const int32_t & pLHS, const nsmat3<T> & pRHS);

template <class T>
nsmat3<T> operator/(const float & pLHS, const nsmat3<T> & pRHS);

template <class T>
nsmat3<T> operator/(const double & pLHS, const nsmat3<T> & pRHS);

template <class T>
T determinant(const nsmat3<T> & mat);

template <class T>
nsmat3<T> rotation2d_mat3(const T angle, bool rads = false);

template <class T>
nsmat3<T> rotation2d_mat3(const nsmat3<T> & transform2d);

template <class T>
nsmat3<T> rotation2d_mat3(const nsmat2<T> & transform2d);

template<class T>
nsmat3<T> rotation_mat3(const NSVec4<T> & axisAngle, bool rads = false);

template<class T>
nsmat3<T> rotation_mat3(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order , bool rads = false);

template<class T>
nsmat3<T> rotation_mat3(const nsquat<T> & orientation);

template<class T>
nsmat3<T> rotation_mat3(const NSVec3<T> & vec, const NSVec3<T> & toVec);

template<class T>
nsmat3<T> rotation_mat3(const nsmat4<T> & transform);

template<class T>
nsmat3<T> scaling2d_mat3(const NSVec2<T> & scale);

template<class T>
nsmat3<T> scaling2d_mat3(const nsmat2<T> & transform2d);

template<class T>
nsmat3<T> scaling2d_mat3(const nsmat3<T> & transform2d);

template <class T>
nsmat3<T> scaling_mat3(const NSVec3<T> & scale);

template <class T>
nsmat3<T> scaling_mat3(const nsmat3<T> & transform);

template <class T>
nsmat3<T> scaling_mat3(const nsmat4<T> & transform);

template <class T>
nsmat3<T> translation2d_mat3(const NSVec3<T> & v3);

template <class T>
nsmat3<T> translation2d_mat3(const NSVec2<T> & v2);

template <class T>
nsmat3<T> transpose(const nsmat3<T> mat);

template <class T>
nsmat3<T> inverse(const nsmat3<T> mat);

template<class PUPer, class T>
void pup(PUPer & p, nsmat3<T> & m3);

template <class T>
struct nsmat3
{
	nsmat3()
	{
		setIdentity();
	}

	nsmat3(const T & val)
	{
		set(val);
	}

	nsmat3(const nsmat3 & copy)
	{
		set(copy.data[0], copy.data[1], copy.data[2]);
	}

	nsmat3(
		const T & a, const T & b, const T & c,
		const T & d, const T & e, const T & f,
		const T & g, const T & h, const T & i)
	{
		set(a, b, c, d, e, f, g, h, i);
	}

	nsmat3(const NSVec3<T> & row1, const NSVec3<T> & row2, const NSVec3<T> & row3)
	{
		set(row1, row2, row3);
	}

	nsmat3(const nsmat2<T> & basis)
	{
		set(basis);
		data[0][2] = static_cast<T>(0); data[1][2] = static_cast<T>(0);
		data[2][0] = static_cast<T>(0); data[2][1] = static_cast<T>(0); data[2][2] = static_cast<T>(1);
	}

	nsmat2<T> basis() const
	{
		return nsmat2<T>(data[0][0], data[0][1], data[1][0], data[1][1]);
	}

	T * dataPtr()
	{
		return &data[0][0];
	}

	T determinant() const
	{
		return
			data[0][0] * data[1][1] * data[2][2] - data[0][0] * data[1][2] * data[2][1] +
			data[0][1] * data[1][2] * data[2][0] - data[0][1] * data[1][0] * data[2][2] +
			data[0][2] * data[1][0] * data[2][1] - data[0][2] * data[1][1] * data[2][0];
	}

	nsmat3<T> & invert()
	{
		T det = determinant();

		if (det == static_cast<T>(0))
			return set(0);

		nsmat3 ret;
		ret[0].set(
			data[1][1] * data[2][2] - data[1][2] * data[2][1],
			data[0][2] * data[2][1] - data[0][1] * data[2][2],
			data[0][1] * data[1][2] - data[0][2] * data[1][1]);

		ret[1].set(
			data[1][2] * data[2][0] - data[1][0] * data[2][2],
			data[0][0] * data[2][2] - data[0][2] * data[2][0],
			data[0][2] * data[1][0] - data[0][0] * data[1][2]);

		ret[2].set(
			data[1][0] * data[2][1] - data[1][1] * data[2][0],
			data[0][1] * data[2][0] - data[0][0] * data[2][1],
			data[0][0] * data[1][1] - data[0][1] * data[1][0]);

		ret /= determinant();
		return (*this = ret);
	}

	NSVec3<T> right() const
	{
		return normalize(data[0]);
	}

	nsmat3<T> & rotation2dFrom(const T angle, bool rads = false)
	{
		if (!rads)
			angle = radians(angle);

		data[0][0] = static_cast<T>(std::cos(angle)); data[0][1] = static_cast<T>(std::sin(angle));
		data[1][0] = static_cast<T>(-std::sin(angle)); data[1][1] = static_cast<T>(std::cos(angle));
		data[0][2] = data[1][2] = data[2][0] = data[2][1] = static_cast<T>(0); data[2][2] = static_cast<T>(1);
		return *this;
	}

	nsmat3<T> & rotation2dFrom(const nsmat3<T> & transform2d)
	{
		*this = transform2d;
		data[0][2] = data[1][2] = data[2][0] = data[2][1] = static_cast<T>(0); data[2][2] = static_cast<T>(1);
		data[0].normalize();
		data[1].normalize();
		return *this;
	}

	nsmat3<T> & rotation2dFrom(const nsmat2<T> & transform2d)
	{
		set(transform2d);
		data[0][2] = data[1][2] = data[2][0] = data[2][1] = static_cast<T>(0); data[2][2] = static_cast<T>(1);
		data[0].normalize();
		data[1].normalize();
		return *this;
	}

	nsmat3<T> & rotationFrom(const NSVec4<T> & axisAngle, bool rads = false)
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

		return *this;
	}

	nsmat3<T> & rotationFrom(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order , bool rads = false)
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
		return roundToZero();
	}

	nsmat3<T> & rotationFrom(const nsquat<T> & orientation)
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
		return *this;
	}

	nsmat3<T> & rotationFrom(const nsmat3<T> & transform)
	{
		*this = transform;
		data[0].normalize();
		data[1].normalize();
		data[2].normalize();
		return *this;
	}

	nsmat3<T> & rotationFrom(const nsmat4<T> & transform)
	{
		*this = transform.basis();
		data[0].normalize();
		data[1].normalize();
		data[2].normalize();
		return *this;
	}

	nsmat3<T> & rotationFrom(const NSVec3<T> & vec, const NSVec3<T> & toVec)
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
		return *this;
	}

	nsmat3<T> & roundToZero()
	{
		data[0].roundToZero();
		data[1].roundToZero();
		data[2].roundToZero();
		return *this;
	}

	nsmat3<T> & scaling2dFrom(const NSVec2<T> & scale)
	{
		setIdentity();
		data[0][0] = scale.x; data[1][1] = scale.y;
		return *this;
	}

	nsmat3<T> & scaling2dFrom(const nsmat2<T> & transform2d)
	{
		NSVec2<T> scalingVec(transform2d[0].length(), transform2d[1].length());
		return scalingFrom(scalingVec);
	}

	nsmat3<T> & scaling2dFrom(const nsmat3<T> & transform2d)
	{
		NSVec2<T> scalingVec;
		scalingVec.x = sqrt(transform2d[0][0] * transform2d[0][0] + transform2d[0][1] * transform2d[0][1]);
		scalingVec.y = sqrt(transform2d[1][0] * transform2d[1][0] + transform2d[1][1] * transform2d[1][1]);
		return scalingFrom(scalingVec);
	}

	nsmat3<T> & scalingFrom(const NSVec3<T> & scale)
	{
		setIdentity();
		data[0][0] = scale.x; data[1][1] = scale.y; data[2][2] = scale.z;
		return *this;
	}

	nsmat3<T> & scalingFrom(const nsmat3<T> & transform)
	{
		NSVec3<T> scalingVec(transform[0].length(), transform[1].length(), transform[2].length());		
		return scalingFrom(scalingVec);
	}

	nsmat3<T> & scalingFrom(const nsmat4<T> & transform)
	{
		NSVec3<T> scalingVec;
		scalingVec.x = sqrt(transform[0][0] * transform[0][0] + transform[0][1] * transform[0][1] + transform[0][2] * transform[0][2]);
		scalingVec.y = sqrt(transform[1][0] * transform[1][0] + transform[1][1] * transform[1][1] + transform[1][2] * transform[1][2]);
		scalingVec.z = sqrt(transform[2][0] * transform[2][0] + transform[2][1] * transform[2][1] + transform[2][2] * transform[2][2]);
		return scalingFrom(scalingVec);
	}

	nsmat3<T> & set(
		const T & a, const T & b, const T & c,
		const T & d, const T & e, const T & f,
		const T & g, const T & h, const T & i)
	{
		data[0][0] = a; data[0][1] = b; data[0][2] = c;
		data[1][0] = d; data[1][1] = e; data[1][2] = f;
		data[2][0] = g; data[2][1] = h; data[2][2] = i;
		return *this;
	}

	nsmat3<T> & set(const T & val)
	{
		data[0].set(val);
		data[1].set(val);
		data[2].set(val);
		return *this;
	}

	nsmat3<T> & set(const NSVec3<T> & row1, const NSVec3<T> & row2, const NSVec3<T> & row3)
	{
		data[0] = row1;
		data[1] = row2;
		data[2] = row3;
		return *this;
	}

	nsmat3<T> & set(const nsmat2<T> & basis)
	{
		data[0][0] = basis[0][0]; data[0][1] = basis[0][1];
		data[1][0] = basis[1][0]; data[1][1] = basis[1][1];
		return *this;
	}

	nsmat3<T> & setColumn(const uint32_t & i, const T & x, const T & y,const T & z)
	{
		(*this)[0][i] = x; (*this)[1][i] = y; (*this)[2][i] = z;
		return *this;
	}

	nsmat3<T> & setColumn(const uint32_t & i, const NSVec3<T> & col)
	{
		(*this)[0][i] = col.x; (*this)[1][i] = col.y; (*this)[2][i] = col.z;
		return *this;
	}

	nsmat3<T> & setIdentity()
	{
		data[0].set(static_cast<T>(1), static_cast<T>(0), static_cast<T>(0));
		data[1].set(static_cast<T>(0), static_cast<T>(1), static_cast<T>(0));
		data[2].set(static_cast<T>(0), static_cast<T>(0), static_cast<T>(1));
		return *this;
	}

	NSVec3<T> target() const
	{
		return normalize(data[2]);
	}

	nsmat3<T> & translation2dFrom(const NSVec3<T> & v3)
	{
		setIdentity();
		return setColumn(2, v3.x, v3.y, 1);
	}

	nsmat3<T> & translation2dFrom(const NSVec2<T> & v2)
	{
		setIdentity();
		return setColumn(2, v2.x, v2.y, 1);
	}

	nsmat3<T> & transpose()
	{
		T tmp;
		tmp = data[1][0]; data[1][0] = data[0][1]; data[0][1] = tmp;
		tmp = data[2][0]; data[2][0] = data[0][2]; data[0][2] = tmp;
		tmp = data[2][1]; data[2][1] = data[1][2]; data[1][2] = tmp;
		return *this;
	}

	std::string toString(bool newline=true) const
	{
		std::stringstream ss;
		std::string lc = ";";

		if (newline)
			lc += '\n';

		ss << "[" << data[0][0] << " " << data[0][1] << " " << data[0][2] << lc;
		ss << " " << data[1][0] << " " << data[1][1] << " " << data[1][2] << lc;
		ss << " " << data[2][0] << " " << data[2][1] << " " << data[2][2] << "]";
		return ss.str();
	}

	NSVec3<T> up() const
	{
		return normalize(data[1]);
	}

	// Overloaded operators
	nsmat3<T> operator*(const nsmat3<T> & rhs) const
	{
		nsmat3<T> ret;
		
		ret.data[0][0] = data[0][0] * rhs.data[0][0] + data[0][1] * rhs.data[1][0] + data[0][2] * rhs.data[2][0];
		ret.data[0][1] = data[0][0] * rhs.data[0][1] + data[0][1] * rhs.data[1][1] + data[0][2] * rhs.data[2][1];
		ret.data[0][2] = data[0][0] * rhs.data[0][2] + data[0][1] * rhs.data[1][2] + data[0][2] * rhs.data[2][2];

		ret.data[1][0] = data[1][0] * rhs.data[0][0] + data[1][1] * rhs.data[1][0] + data[1][2] * rhs.data[2][0];
		ret.data[1][1] = data[1][0] * rhs.data[0][1] + data[1][1] * rhs.data[1][1] + data[1][2] * rhs.data[2][1];
		ret.data[1][2] = data[1][0] * rhs.data[0][2] + data[1][1] * rhs.data[1][2] + data[1][2] * rhs.data[2][2];

		ret.data[2][0] = data[2][0] * rhs.data[0][0] + data[2][1] * rhs.data[1][0] + data[2][2] * rhs.data[2][0];
		ret.data[2][1] = data[2][0] * rhs.data[0][1] + data[2][1] * rhs.data[1][1] + data[2][2] * rhs.data[2][1];
		ret.data[2][2] = data[2][0] * rhs.data[0][2] + data[2][1] * rhs.data[1][2] + data[2][2] * rhs.data[2][2];

		return ret;
	}

	nsmat3<T> operator/(const nsmat3<T> & rhs) const
	{
		return *this * inverse(rhs);
	}

	nsmat3<T> operator%(const nsmat3<T> & rhs) const
	{
		nsmat3<T> ret;
		ret.data[0] = data[0] % rhs.data[0];
		ret.data[1] = data[1] % rhs.data[1];
		ret.data[2] = data[2] % rhs.data[2];
		return ret;
	}

	NSVec3<T> operator*(const NSVec3<T> & rhs) const
	{
		return NSVec3<T>(data[0] * rhs, data[1] * rhs, data[2] * rhs);
	}

	nsmat3<T> operator%(const NSVec3<T> & rhs) const
	{
		return nsmat3<T>(data[0] % rhs, data[1] % rhs, data[2] % rhs);
	}

	nsmat3<T> operator/(const NSVec3<T> & rhs) const
	{
		nsmat3<T> ret;
		ret.data[0] = data[0] / rhs[0];
		ret.data[1] = data[1] / rhs[1];
		ret.data[2] = data[2] / rhs[2];
		return ret;
	}

	nsmat3<T> operator*(const T & rhs) const
	{
		nsmat3<T> ret;
		ret.data[0] = data[0] * rhs;
		ret.data[1] = data[1] * rhs;
		ret.data[2] = data[2] * rhs;
		return ret;
	}

	nsmat3<T> operator/(const T & rhs) const
	{
		nsmat3<T> ret;
		ret.data[0] = data[0] / rhs;
		ret.data[1] = data[1] / rhs;
		ret.data[2] = data[2] / rhs;
		return ret;
	}
	
	nsmat3<T> operator+(const nsmat3<T> & rhs) const
	{
		nsmat3<T> ret;
		ret.data[0] = data[0] + rhs.data[0];
		ret.data[1] = data[1] + rhs.data[1];
		ret.data[2] = data[2] + rhs.data[2];
		return ret;
	}

	nsmat3<T> operator-(const nsmat3<T> & rhs) const
	{
		nsmat3<T> ret;
		ret.data[0] = data[0] - rhs.data[0];
		ret.data[1] = data[1] - rhs.data[1];
		ret.data[2] = data[2] - rhs.data[2];
		return ret;
	}

	bool operator==(const nsmat3<T> & rhs) const
	{
		return (data[0] == rhs.data[0] && data[1] == rhs.data[1] && data[2] == rhs.data[2]);
	}

	bool operator!=(const nsmat3<T> & rhs) const
	{
		return !(*this == rhs);
	}

	nsmat3<T> & operator=(const nsmat3<T> & rhs)
	{
		if (this == &rhs)
			return *this;

		data[0] = rhs.data[0];
		data[1] = rhs.data[1];
		data[2] = rhs.data[2];
		return *this;
	}

	nsmat3<T> & operator*=(const nsmat3<T> & rhs)
	{
		*this = *this * rhs;
		return *this;
	}

	nsmat3<T> & operator/=(const nsmat3<T> & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	nsmat3<T> & operator%=(const nsmat3<T> & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	nsmat3<T> & operator%=(const NSVec3<T> & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	nsmat3<T> & operator/=(const NSVec3<T> & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	nsmat3<T> & operator*=(const T & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	nsmat3<T> & operator/=(const T & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	nsmat3<T> operator++(int32_t)
	{
		nsmat3<T> ret(*this);
		++(*this);
		return ret;
	}

	nsmat3<T> operator--(int32_t)
	{
		nsmat3<T> ret(*this);
		--(*this);
		return ret;
	}

	nsmat3<T> & operator++()
	{
		++data[0]; ++data[1]; ++data[2];
		return *this;
	}

	nsmat3<T> & operator--()
	{
		--data[0]; --data[1]; --data[2];
		return *this;
	}

	nsmat3<T> & operator+=(const nsmat3<T> & rhs)
	{
		*this = *this + rhs;
		return *this;
	}

	nsmat3<T> & operator-=(const nsmat3<T> & rhs)
	{
		*this = *this - rhs;
		return *this;
	}

	const NSVec3<T> & operator[](const uint32_t & pVal) const
	{
		if (pVal > 2)
			throw(std::out_of_range("mat3 index out of range"));
		return data[pVal];
	}

	NSVec3<T> & operator[](const uint32_t & pVal)
	{
		if (pVal > 2)
			throw(std::out_of_range("mat3 index out of range"));
		return data[pVal];
	}

	NSVec3<T> operator()(uint32_t pVal) const
	{
		return NSVec3<T>(data[0][pVal], data[1][pVal], data[2][pVal]);
	}

private:
	NSVec3<T> data[3];
};

template <class T>
nsmat3<T> operator*(const int32_t & pLHS, const nsmat3<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
nsmat3<T> operator*(const float & pLHS, const nsmat3<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
nsmat3<T> operator*(const double & pLHS, const nsmat3<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template<class T>
nsmat3<T> operator/(const int32_t & pLHS, const nsmat3<T> & pRHS)
{
	return nsmat3<T>(pLHS / pRHS[0], pLHS / pRHS[1], pLHS / pRHS[2]);
}

template <class T>
nsmat3<T> operator/(const float & pLHS, const nsmat3<T> & pRHS)
{
	return nsmat3<T>(pLHS / pRHS[0], pLHS / pRHS[1], pLHS / pRHS[2]);
}

template <class T>
nsmat3<T> operator/(const double & pLHS, const nsmat3<T> & pRHS)
{
	return nsmat3<T>(pLHS / pRHS[0], pLHS / pRHS[1], pLHS / pRHS[2]);
}

template <class T>
NSVec3<T> operator*(const NSVec3<T> & lhs, const nsmat3<T> & rhs)
{
	return NSVec3<T>(
		lhs[0] * rhs[0][0] + lhs[1] * rhs[1][0] + lhs[2] * rhs[2][0],
		lhs[0] * rhs[0][1] + lhs[1] * rhs[1][1] + lhs[2] * rhs[2][1],
		lhs[0] * rhs[0][2] + lhs[1] * rhs[1][2] + lhs[2] * rhs[2][2]);
}

template <class T>
NSVec3<T> operator/(const NSVec3<T> & lhs, const nsmat3<T> & rhs)
{
	return lhs * inverse(rhs);
}

template <class T>
nsmat3<T> operator%(const NSVec3<T> & lhs, const nsmat3<T> & rhs)
{
	return nsmat3<T>(rhs[0] * lhs[0], rhs[1] * lhs[1], rhs[2] * lhs[2]);
}

template <class T>
T determinant(const nsmat3<T> & mat)
{
	return mat.determinant();
}

template <class T>
nsmat3<T> rotation2d_mat3(const T angle, bool rads)
{
	return nsmat3<T>().rotation2dFrom(angle, rads);
}

template <class T>
nsmat3<T> rotation2d_mat3(const nsmat3<T> & transform2d)
{
	return nsmat3<T>().rotation2dFrom(transform2d);
}

template <class T>
nsmat3<T> rotation2d_mat3(const nsmat2<T> & transform2d)
{
	return nsmat3<T>().rotation2dFrom(transform2d);
}

template<class T>
nsmat3<T> rotation_mat3(const NSVec4<T> & axisAngle, bool rads)
{
	return nsmat3<T>().rotationFrom(axisAngle, rads);
}

template<class T>
nsmat3<T> rotation_mat3(const NSVec3<T> & euler, typename NSVec3<T>::EulerOrder order, bool rads)
{
	return nsmat3<T>().rotationFrom(euler,order, rads);
}

template<class T>
nsmat3<T> rotation_mat3(const nsquat<T> & orientation)
{
	return nsmat3<T>().rotationFrom(orientation);
}

template<class T>
nsmat3<T> rotation_mat3(const NSVec3<T> & vec, const NSVec3<T> & toVec)
{
	return nsmat3<T>().rotationFrom(vec, toVec);
}

template<class T>
nsmat3<T> rotation_mat3(const nsmat4<T> & transform)
{
	return nsmat3<T>().rotationFrom(transform);
}

template<class T>
nsmat3<T> rotationMat3(const nsmat3<T> & transform)
{
	return nsmat3<T>().rotationFrom(transform);
}

template<class T>
nsmat3<T> scaling2d_mat3(const NSVec2<T> & scale)
{
	return nsmat3<T>().scaling2dFrom(scale);
}

template<class T>
nsmat3<T> scaling2d_mat3(const nsmat2<T> & transform2d)
{
	return nsmat3<T>().scaling2dFrom(transform2d);
}

template<class T>
nsmat3<T> scaling2d_mat3(const nsmat3<T> & transform2d)
{
	return nsmat3<T>().scaling2dFrom(transform2d);
}

template <class T>
nsmat3<T> scaling_mat3(const NSVec3<T> & scale)
{
	return nsmat3<T>().scalingFrom(scale);
}

template <class T>
nsmat3<T> scaling_mat3(const nsmat3<T> & transform)
{
	return nsmat3<T>().scalingFrom(transform);
}

template <class T>
nsmat3<T> scaling_mat3(const nsmat4<T> & transform)
{
	return nsmat3<T>().scalingFrom(transform);
}

template <class T>
nsmat3<T> translation2d_mat3(const NSVec3<T> & v3)
{
	return nsmat3<T>().translation2dFrom(v3);
}

template <class T>
nsmat3<T> translation2d_mat3(const NSVec2<T> & v2)
{
	return nsmat3<T>().translation2dFrom(v2);
}

template <class T>
nsmat3<T> transpose(nsmat3<T> mat)
{
	return mat.transpose();
}

template <class T>
nsmat3<T> inverse(nsmat3<T> mat)
{
	return mat.invert();
}

template<class PUPer, class T>
void pup(PUPer & p, nsmat3<T> & m3, const std::string & varName)
{
	pup(p, m3[0], varName + "[0]"); pup(p, m3[1], varName + "[1]"); pup(p, m3[2], varName + "[2]");
}

#endif
