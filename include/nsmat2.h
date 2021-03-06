#ifndef NSMAT2_H
#define NSMAT2_H

#include "nsquat.h"

template <class T>
nsmat2<T> operator*(const int32_t & pLHS, const nsmat2<T> & pRHS);

template <class T>
nsmat2<T> operator*(const float & pLHS, const nsmat2<T> & pRHS);

template <class T>
nsmat2<T> operator*(const double & pLHS, const nsmat2<T> & pRHS);

template<class T>
nsmat2<T> operator/(const int32_t & pLHS, const nsmat2<T> & pRHS);

template <class T>
nsmat2<T> operator/(const float & pLHS, const nsmat2<T> & pRHS);

template <class T>
nsmat2<T> operator/(const double & pLHS, const nsmat2<T> & pRHS);

template <class T>
NSVec2<T> operator*(const NSVec2<T> & lhs, const nsmat2<T> & rhs);

template <class T>
NSVec2<T> operator/(const NSVec2<T> & lhs, const nsmat2<T> & rhs);

template <class T>
nsmat2<T> operator%(const NSVec2<T> & lhs, const nsmat2<T> & rhs);

template <class T>
T determinant(const nsmat2<T> & mat);

template <class T>
nsmat2<T> rotation2d_mat2(const T & angle, bool rads = false);

template <class T>
nsmat2<T> rotation2d_mat2(const nsmat3<T> & transform2d);

template <class T>
nsmat2<T> rotation2d_mat2(const nsmat2<T> & transform2d);

template<class T>
nsmat2<T> scaling2d_mat2(const NSVec2<T> & scale);

template<class T>
nsmat2<T> scaling2d_mat2(const nsmat2<T> & transform2d);

template<class T>
nsmat2<T> scaling2d_mat2(const nsmat3<T> & transform2d);

template <class T>
nsmat2<T> transpose(const nsmat2<T> mat);

template <class T>
nsmat2<T> inverse(const nsmat2<T> mat);

template<class PUPer, class T>
void pup(PUPer & p, nsmat2<T> & m2);


template <class T>
struct nsmat2
{
	nsmat2()
	{
		setIdentity();
	}

	nsmat2(const T & val)
	{
		set(val);
	}

	nsmat2(const nsmat2 & copy)
	{
		set(copy.data[0], copy.data[1]);
	}

	nsmat2(const T & a, const T & b, const T & c, const T & d)
	{
		set(a, b, c, d);
	}

	nsmat2(const NSVec2<T> & row1, const NSVec2<T> & row2)
	{
		set(row1, row2);
	}

	T * dataPtr()
	{
		return &data[0][0];
	}

	T determinant() const
	{
		return data[0].x*data[1].y - data[1].x*data[0].y;
	}

	nsmat2<T> & invert()
	{
		T det = determinant();

		if (det == static_cast<T>(0))
			return set(0);

		T tmp = data[0][0];
		data[0][0] = data[1][1]/det;
		data[0][1] = -data[0][1]/det;
		data[1][0] = -data[1][0]/det;
		data[1][1] = tmp/det;
		return *this;
	}

	nsmat2<T> & rotationFrom(T angle, bool rads = false)
	{
		if (!rads)
			angle = radians(angle);

		data[0][0] = static_cast<T>(std::cos(angle)); data[0][1] = static_cast<T>(std::sin(angle));
		data[1][0] = static_cast<T>(-std::sin(angle)); data[1][1] = static_cast<T>(std::cos(angle));
		return *this;
	}

	nsmat2<T> & roundToZero()
	{
		data[0].roundToZero();
		data[1].roundToZero();
		return *this;
	}

	nsmat2<T> & scalingFrom(const NSVec2<T> & scale)
	{
		data[0][0] = scale.x; data[1][1] = scale.y;
		data[0][1] = data[1][0] = static_cast<T>(0);
		return *this;
	}

	nsmat2<T> & scalingFrom(const nsmat2<T> & transform2d)
	{
		NSVec2<T> scalingVec(transform2d[0].length(), transform2d[1].length());
		return scalingFrom(scalingVec);
	}

	nsmat2<T> & scalingFrom(const nsmat3<T> & transform2d)
	{
		NSVec2<T> scalingVec;
		scalingVec.x = sqrt(transform2d[0][0] * transform2d[0][0] + transform2d[0][1] * transform2d[0][1]);
		scalingVec.y = sqrt(transform2d[1][0] * transform2d[1][0] + transform2d[1][1] * transform2d[1][1]);
		return scalingFrom(scalingVec);
	}

	nsmat2<T> & set(const T & val)
	{
		data[0].x = val; data[0].y = val; data[1].x = val; data[1].y = val;
		return *this;
	}

	nsmat2<T> & set(const T & a, const T & b, const T & c, const T & d)
	{
		data[0].x = a; data[0].y = b; data[1].x = c; data[1].y = d;
		return *this;
	}

	nsmat2<T> & set(const NSVec2<T> & row1, const NSVec2<T> & row2)
	{
		data[0] = row1;
		data[1] = row2;
		return *this;
	}

	nsmat2<T> & setColumn(const uint32_t & i, const T & x, const T & y)
	{
		(*this)[0][i] = x; (*this)[1][i] = y;
		return *this;
	}

	nsmat2<T> & setColumn(const uint32_t & i, const NSVec2<T> & col)
	{
		(*this)[i][0] = col.x; (*this)[i][1] = col.y;
		return *this;
	}

	nsmat2<T> & setIdentity()
	{
		data[0].set(static_cast<T>(1), static_cast<T>(0));
		data[1].set(static_cast<T>(0), static_cast<T>(1));
		return *this;
	}

	nsmat2<T> & transpose()
	{
		T tmp = data[1][0];
		data[1][0] = data[0][1]; data[0][1] = tmp;
		return *this;
	}

	std::string toString(bool newline = true) const
	{
		std::stringstream ss;
		std::string lc = ";";

		if (newline)
			lc += '\n';

		ss << "[" << data[0][0] << " " << data[0][1] << lc;
		ss << " " << data[1][0] << " " << data[1][1] << "]";
		return ss.str();
	}

	// Overloaded operators
	nsmat2<T> operator*(const nsmat2<T> & rhs) const
	{
		nsmat2<T> ret;
		ret.data[0][0] = data[0][0] * rhs.data[0][0] + data[0][1] * rhs.data[1][0];
		ret.data[0][1] = data[0][0] * rhs.data[0][1] + data[0][1] * rhs.data[1][1];
		ret.data[1][0] = data[1][0] * rhs.data[0][0] + data[1][1] * rhs.data[1][0];
		ret.data[1][1] = data[1][0] * rhs.data[0][1] + data[1][1] * rhs.data[1][1];
		return ret;
	}

	nsmat2<T> operator/(const nsmat2<T> & rhs) const
	{
		return *this * inverse(rhs);
	}

	nsmat2<T> operator%(const nsmat2<T> & rhs) const
	{
		nsmat2<T> ret;
		ret.data[0] = data[0] % rhs.data[0];
		ret.data[1] = data[1] % rhs.data[1];
		return ret;
	}

	NSVec2<T> operator*(const NSVec2<T> & rhs) const
	{
		return NSVec2<T>(data[0] * rhs, data[1] * rhs);
	}

	nsmat2<T> operator%(const NSVec2<T> & rhs) const
	{
		return nsmat2<T>(data[0] % rhs, data[1] % rhs);
	}

	nsmat2<T> operator/(const NSVec2<T> & rhs) const
	{
		nsmat2<T> ret;
		ret.data[0] = data[0] / rhs[0];
		ret.data[1] = data[1] / rhs[1];
		return ret;
	}

	nsmat2<T> operator*(const T & rhs) const
	{
		nsmat2<T> ret;
		ret.data[0] = data[0] * rhs;
		ret.data[1] = data[1] * rhs;
		return ret;
	}

	nsmat2<T> operator/(const T & rhs) const
	{
		nsmat2<T> ret;
		ret.data[0] = data[0] / rhs;
		ret.data[1] = data[1] / rhs;
		return ret;
	}

	nsmat2<T> operator+(const nsmat2<T> & rhs) const
	{
		nsmat2<T> ret;
		ret.data[0] = data[0] + rhs.data[0];
		ret.data[1] = data[1] + rhs.data[1];
		return ret;
	}

	nsmat2<T> operator-(const nsmat2<T> & rhs) const
	{
		nsmat2<T> ret;
		ret.data[0] = data[0] - rhs.data[0];
		ret.data[1] = data[1] - rhs.data[1];
		return ret;
	}

	bool operator==(const nsmat2<T> & rhs) const
	{
		return (data[0] == rhs.data[0] && data[1] == rhs.data[1]);
	}

	bool operator!=(const nsmat2<T> & rhs) const
	{
		return !(*this == rhs);
	}

	nsmat2<T> & operator=(const nsmat2<T> & rhs)
	{
		if (this == &rhs)
			return *this;

		data[0] = rhs.data[0];
		data[1] = rhs.data[1];
		return *this;
	}

	nsmat2<T> & operator*=(const nsmat2<T> & rhs)
	{
		*this = *this * rhs;
		return *this;
	}

	nsmat2<T> & operator%=(const nsmat2<T> & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	nsmat2<T> & operator/=(const nsmat2<T> & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	nsmat2<T> & operator%=(const NSVec2<T> & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	nsmat2<T> & operator/=(const NSVec2<T> & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	nsmat2<T> & operator*=(const T & rhs)
	{
		*this = *this % rhs;
		return *this;
	}

	nsmat2<T> & operator/=(const T & rhs)
	{
		*this = *this / rhs;
		return *this;
	}

	nsmat2<T> operator++(int32_t)
	{
		nsmat2<T> ret(*this);
		++(*this);
		return ret;
	}

	nsmat2<T> operator--(int32_t)
	{
		nsmat2<T> ret(*this);
		--(*this);
		return ret;
	}

	nsmat2<T> & operator++()
	{
		++data[0]; ++data[1];
		return *this;
	}

	nsmat2<T> & operator--()
	{
		--data[0]; --data[1];
		return *this;
	}

	nsmat2<T> & operator+=(const nsmat2<T> & rhs)
	{
		*this = *this + rhs;
		return *this;
	}

	nsmat2<T> & operator-=(const nsmat2<T> & rhs)
	{
		*this = *this - rhs;
		return *this;
	}

	const NSVec2<T> & operator[](const uint32_t & pVal) const
	{
		if (pVal > 1)
			throw(std::out_of_range("mat2 index out of range"));
		return data[pVal];
	}

	NSVec2<T> & operator[](const uint32_t & pVal)
	{
		if (pVal > 1)
			throw(std::out_of_range("mat2 index out of range"));
		return data[pVal];
	}

	NSVec2<T> operator()(const uint32_t & pVal)
	{
		return NSVec2<T>(data[0][pVal], data[1][pVal]);
	}

private:
	NSVec2<T> data[2];
};

template <class T>
nsmat2<T> operator*(const int32_t & pLHS, const nsmat2<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
nsmat2<T> operator*(const float & pLHS, const nsmat2<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template <class T>
nsmat2<T> operator*(const double & pLHS, const nsmat2<T> & pRHS)
{
	return pRHS * static_cast<T>(pLHS);
}

template<class T>
nsmat2<T> operator/(const int32_t & pLHS, const nsmat2<T> & pRHS)
{
	return nsmat2<T>(pLHS / pRHS[0], pLHS / pRHS[1]);
}

template <class T>
nsmat2<T> operator/(const float & pLHS, const nsmat2<T> & pRHS)
{
	return nsmat2<T>(pLHS / pRHS[0], pLHS / pRHS[1]);
}

template <class T>
nsmat2<T> operator/(const double & pLHS, const nsmat2<T> & pRHS)
{
	return nsmat2<T>(pLHS / pRHS[0], pLHS / pRHS[1]);
}

template <class T>
NSVec2<T> operator*(const NSVec2<T> & lhs, const nsmat2<T> & rhs)
{
	return NSVec2<T>(
		lhs[0] * rhs[0][0] + lhs[1] * rhs[1][0],
		lhs[0] * rhs[0][1] + lhs[1] * rhs[1][1]);
}

template <class T>
NSVec2<T> operator/(const NSVec2<T> & lhs, const nsmat2<T> & rhs)
{
	return lhs * inverse(rhs);
}

template <class T>
nsmat2<T> operator%(const NSVec2<T> & lhs, const nsmat2<T> & rhs)
{
	return nsmat2<T>(rhs[0] * lhs[0],rhs[1] * lhs[1]);
}

template <class T>
T determinant(const nsmat2<T> & mat)
{
	return mat.determinant();
}

template <class T>
nsmat2<T> rotation2d_mat2(const T & angle, bool rads)
{
	return nsmat2<T>().rotationFrom(angle, rads);
}

template <class T>
nsmat2<T> rotation2d_mat2(const nsmat3<T> & transform2d)
{
	return nsmat2<T>().rotationFrom(transform2d);
}

template <class T>
nsmat2<T> rotation2d_mat2(const nsmat2<T> & transform2d)
{
	return nsmat2<T>().rotationFrom(transform2d);
}

template<class T>
nsmat2<T> scaling2d_mat2(const NSVec2<T> & scale)
{
	return nsmat2<T>().scalingFrom(scale);
}

template<class T>
nsmat2<T> scaling2d_mat2(const nsmat2<T> & transform2d)
{
	return nsmat2<T>().scalingFrom(transform2d);
}

template<class T>
nsmat2<T> scaling2d_mat2(const nsmat3<T> & transform2d)
{
	return nsmat2<T>().scalingFrom(transform2d);
}

template <class T>
nsmat2<T> transpose(nsmat2<T> mat)
{
	return mat.transpose();
}

template <class T>
nsmat2<T> inverse(nsmat2<T> mat)
{
	return mat.invert();
}

template<class PUPer, class T>
void pup(PUPer & p, nsmat2<T> & m2, const std::string & varName)
{
	pup(p, m2[0], varName + "[0]"); pup(p, m2[1], varName + "[1]");
}

#endif
