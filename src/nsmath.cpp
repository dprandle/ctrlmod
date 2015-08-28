#include <nsmath.h>

float clampf(float pVal, const float & pMin, const float & pMax)
{
	if (pVal < pMin)
		pVal = pMin;
	if (pVal > pMax)
		pVal = pMax;
	return pVal;
}

double clamp(double pVal, const double & pMin, const double & pMax)
{
	if (pVal < pMin)
		pVal = pMin;
	if (pVal > pMax)
		pVal = pMax;
	return pVal;
}

float fractf(const float & num)
{
	float flr = std::floor(num);
	return num - flr;
}

double fract(const double & num)
{
	double flr = std::floor(num);
	return num - flr;
}

float lerp(float low, float high, float middle)
{
	return (middle - low) / (high - low);
}


double lerp(double low, double high, double middle)
{
	return (middle - low) / (high - low);
}

float lerp(int32_t low, int32_t high, int32_t middle)
{	
	return float(middle - low) / float(high - low);
}

float lerp(uint32_t low, uint32_t high, uint32_t middle)
{
	return float(middle - low) / float(high - low);
}

#define FLOAT_EPS 0.00001

/*!
Calculates the box given a set of vertices.. if no vertices are given then will set everything to zero.
*/
NSBoundingBox::NSBoundingBox(
	const std::vector<fvec3> & pVertices
	)
{
	calculate(pVertices);
}

/*!
Find the min and max of a set of vertices and use that to make the bounding box.
*/
void NSBoundingBox::calculate(
	const std::vector<fvec3> & pVertices,
	const fmat4 & pTransform
	)
{
	clear();
	extend(pVertices, pTransform);
}

/*!
Returns the center of the box or the center of a face of the box if pFace is specified as something
other than None.
*/
fvec3 NSBoundingBox::center(const Face & pFace)
{
	fvec3 center;
	switch (pFace)
	{
	case (None) :
		center.set(dx() / 2.0f, dy() / 2.0f, dz() / 2.0f);
		break;
	case (Bottom) :
		center.set(dx() / 2.0f, dy() / 2.0f, mMin.z);
		break;
	case (Top) :
		center.set(dx() / 2.0f, dy() / 2.0f, mMax.z);
		break;
	case (Left) :
		center.set(mMin.x, dy() / 2.0f, dz() / 2.0f);
		break;
	case (Right) :
		center.set(mMax.x, dy() / 2.0f, dz() / 2.0f);
		break;
	case (Back) :
		center.set(dx() / 2.0f, mMin.y, dz() / 2.0f);
		break;
	case (Front) :
		center.set(dx() / 2.0f, mMax.y, dz() / 2.0f);
		break;
	}
	return center;
}

/*!
Clears the verts and the min/max to 0.
*/
void NSBoundingBox::clear()
{
	mMin = fvec3();
	mMax = fvec3();
	for (uint32_t i = 0; i < 8; ++i)
		mVerts[i] = fvec3();
}

/*!
Length of box in x direction.
*/
float NSBoundingBox::dx()
{
	return mMax.x - mMin.x;
}

/*!
Length of box in y direction.
*/
float NSBoundingBox::dy()
{
	return mMax.y - mMin.y;
}

/*!
Length of box in z direction.
*/
float NSBoundingBox::dz()
{
	return mMax.z - mMin.z;
}

void NSBoundingBox::extend(
	const std::vector<fvec3> & pVertices,
	const fmat4 & pTransform
	)
{
	for (uint32_t i = 0; i < pVertices.size(); ++i)
	{
		fvec3 tVert = (pTransform * fvec4(pVertices[i], 1.0f)).xyz();
		// Find maximum of each dimension
		if (tVert.x > mMax.x)
			mMax.x = tVert.x;
		if (tVert.y > mMax.y)
			mMax.y = tVert.y;
		if (tVert.z > mMax.z)
			mMax.z = tVert.z;

		// Find minimum of each dimension
		if (tVert.x < mMin.x)
			mMin.x = tVert.x;
		if (tVert.y < mMin.y)
			mMin.y = tVert.y;
		if (tVert.z < mMin.z)
			mMin.z = tVert.z;
	}
	_updateVerts();
}

/*!
Set the min and max - will update the verts based on this new min and max.
*/
void NSBoundingBox::set(
	const fvec3 & pMin,
	const fvec3 pMax
	)
{
	mMin = pMin;
	mMax = pMax;
	_updateVerts();
}

/*!
The volume in whatever units the world is represented in. The cartesian coordinate x = 1, y = 1, z = 1 would
represent a point that is 1 unit away from each axis and 1.41 units away from the origin.
*/
float NSBoundingBox::volume()
{
	return (dx() * dy() * dz());
}

void NSBoundingBox::_updateVerts()
{
	mVerts[0] = mMin;
	mVerts[1] = fvec3(mMax.x, mMin.y, mMin.z);
	mVerts[2] = fvec3(mMin.x, mMax.y, mMin.z);
	mVerts[3] = fvec3(mMax.x, mMax.y, mMin.z);
	mVerts[4] = fvec3(mMin.x, mMin.y, mMax.z);
	mVerts[5] = fvec3(mMax.x, mMin.y, mMax.z);
	mVerts[6] = fvec3(mMin.x, mMax.y, mMax.z);
	mVerts[7] = mMax;
}

float random_float(float pHigh, float pLow)
{
	return pLow + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (pHigh - pLow)));
}
