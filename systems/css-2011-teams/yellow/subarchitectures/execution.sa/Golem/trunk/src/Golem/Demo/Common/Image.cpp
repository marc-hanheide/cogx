/** @file Image3.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/Image.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const Image2::Element Image2::ElementMin = numeric_const<U8>::MIN;
const Image2::Element Image2::ElementMax = numeric_const<U8>::MAX;

//------------------------------------------------------------------------------

void Image2::getLimits(Vec2 &min, Vec2 &max, const Mat23 &trn, const Image2 &image) const {
	min.set(numeric_const<Real>::MAX), max.set(numeric_const<Real>::MIN);
	Vec2 upper(image.grid.v1*image.iend.n1, image.grid.v2*image.iend.n2);
	for (U8 i = 0; i < 4; i++) {
		Vec2 edge(i & 0x1 ? upper.v1 : REAL_ZERO, i & 0x2 ? upper.v2 : REAL_ZERO);
		trn.multiply(edge, edge);
		
		if (min.v1 > edge.v1)
			min.v1 = edge.v1;
		if (min.v2 > edge.v2)
			min.v2 = edge.v2;
		if (max.v1 < edge.v1)
			max.v1 = edge.v1;
		if (max.v2 < edge.v2)
			max.v2 = edge.v2;
	}
}

bool Image2::getIntersection(Idx2 &begin, Idx2 &end, const Image2 &image) const {
	if (isEmpty() || image.isEmpty())
		return false;

	Mat23 trn;
	trn.setInverseRT(pose);
	trn.multiply(image.pose, trn);

	Vec2 min, max;
	getLimits(min, max, trn, image);

	begin.n1 = std::max((Index)ibegin.n1, (Index)Math::floor(min.v1/grid.v1));
	begin.n2 = std::max((Index)ibegin.n2, (Index)Math::floor(min.v2/grid.v2));
	end.n1 = std::min((Index)iend.n1, (Index)Math::ceil(max.v1/grid.v1) + 1);
	end.n2 = std::min((Index)iend.n2, (Index)Math::ceil(max.v2/grid.v2) + 1);

	return begin < end;
}

bool Image2::setIntersection(const Image2 &image) {
	if (image.isEmpty())
		return false;

	Vec2 min, max;
	getLimits(min, max, image.pose, image);

	Desc desc;

	desc.dimensions.n1 = (Index)Math::ceil((max.v1 - min.v1)/image.grid.v1) + 1;
	desc.dimensions.n2 = (Index)Math::ceil((max.v2 - min.v2)/image.grid.v2) + 1;
	
	desc.grid = image.grid;

	desc.pose.R.setId();
	desc.pose.p.set(
		min.v1 - REAL_HALF*(desc.grid.v1*(desc.dimensions.n1 - 1) - (max.v1 - min.v1)),
		min.v2 - REAL_HALF*(desc.grid.v2*(desc.dimensions.n2 - 1) - (max.v2 - min.v2))
	);

	return create(desc);
}

//------------------------------------------------------------------------------

bool Image2::interpolateNearestNeighbour(const Idx2 &begin, const Idx2 &end, const Image2 &image) {
	if (isEmpty() || image.isEmpty())
		return false;

	Mat23 trn;
	trn.setInverseRT(image.pose);
	trn.multiply(pose, trn);
	
	Idx2 i;
	for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
		for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++) {
			Vec2 p(grid.v1*i.n1, grid.v2*i.n2);
			trn.multiply(p, p);
			
			Idx2 j;
			j.n1 = (Index)Math::round(p.v1/image.grid.v1);
			j.n2 = (Index)Math::round(p.v2/image.grid.v2);
			
			(*this)(i) =  j < image.end() ? image(j) : Image2::ElementMin;
		}

	return true;
}
	
//------------------------------------------------------------------------------

const Image3::Element Image3::ElementMin = numeric_const<U8>::MIN;
const Image3::Element Image3::ElementMax = numeric_const<U8>::MAX;

//------------------------------------------------------------------------------

void Image3::getLimits(Vec3 &min, Vec3 &max, const Mat34 &trn, const Image3 &image) const {
	min.set(numeric_const<Real>::MAX), max.set(numeric_const<Real>::MIN);
	Vec3 upper(image.grid.v1*image.iend.n1, image.grid.v2*image.iend.n2, image.grid.v3*image.iend.n3);
	for (U8 i = 0; i < 8; i++) {
		Vec3 edge(i & 0x1 ? upper.v1 : REAL_ZERO, i & 0x2 ? upper.v2 : REAL_ZERO, i & 0x4 ? upper.v3 : REAL_ZERO);
		trn.multiply(edge, edge);
		
		if (min.v1 > edge.v1)
			min.v1 = edge.v1;
		if (min.v2 > edge.v2)
			min.v2 = edge.v2;
		if (min.v3 > edge.v3)
			min.v3 = edge.v3;
		if (max.v1 < edge.v1)
			max.v1 = edge.v1;
		if (max.v2 < edge.v2)
			max.v2 = edge.v2;
		if (max.v3 < edge.v3)
			max.v3 = edge.v3;
	}
}

bool Image3::getIntersection(Idx3 &begin, Idx3 &end, const Image3 &image) const {
	if (isEmpty() || image.isEmpty())
		return false;

	Mat34 trn;
	trn.setInverseRT(pose);
	trn.multiply(image.pose, trn);

	Vec3 min, max;
	getLimits(min, max, trn, image);

	begin.n1 = std::max((Index)ibegin.n1, (Index)Math::floor(min.v1/grid.v1));
	begin.n2 = std::max((Index)ibegin.n2, (Index)Math::floor(min.v2/grid.v2));
	begin.n3 = std::max((Index)ibegin.n3, (Index)Math::floor(min.v3/grid.v3));
	end.n1 = std::min((Index)iend.n1, (Index)Math::ceil(max.v1/grid.v1) + 1);
	end.n2 = std::min((Index)iend.n2, (Index)Math::ceil(max.v2/grid.v2) + 1);
	end.n3 = std::min((Index)iend.n3, (Index)Math::ceil(max.v3/grid.v3) + 1);

	return begin < end;
}

bool Image3::setIntersection(const Image3 &image) {
	if (image.isEmpty())
		return false;

	Vec3 min, max;
	getLimits(min, max, image.pose, image);

	Desc desc;

	desc.dimensions.n1 = (Index)Math::ceil((max.v1 - min.v1)/image.grid.v1) + 1;
	desc.dimensions.n2 = (Index)Math::ceil((max.v2 - min.v2)/image.grid.v2) + 1;
	desc.dimensions.n3 = (Index)Math::ceil((max.v3 - min.v3)/image.grid.v3) + 1;
	
	desc.grid = image.grid;

	desc.pose.R.setId();
	desc.pose.p.set(
		min.v1 - REAL_HALF*(desc.grid.v1*(desc.dimensions.n1 - 1) - (max.v1 - min.v1)),
		min.v2 - REAL_HALF*(desc.grid.v2*(desc.dimensions.n2 - 1) - (max.v2 - min.v2)),
		min.v3 - REAL_HALF*(desc.grid.v3*(desc.dimensions.n3 - 1) - (max.v3 - min.v3))
	);

	return create(desc);
}

//------------------------------------------------------------------------------

bool Image3::interpolateNearestNeighbour(const Idx3 &begin, const Idx3 &end, const Image3 &image) {
	if (isEmpty() || image.isEmpty())
		return false;

	Mat34 trn;
	trn.setInverseRT(image.pose);
	trn.multiply(pose, trn);
	
	Idx3 i;
	for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
		for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++)
			for (i.n3 = begin.n3; i.n3 < end.n3; i.n3++) {
				Vec3 p(grid.v1*i.n1, grid.v2*i.n2, grid.v3*i.n3);
				trn.multiply(p, p);
				
				Idx3 j;
				j.n1 = (Index)Math::round(p.v1/image.grid.v1);
				j.n2 = (Index)Math::round(p.v2/image.grid.v2);
				j.n3 = (Index)Math::round(p.v3/image.grid.v3);
				
				(*this)(i) =  j < image.end() ? image(j) : Image3::ElementMin;
			}

	return true;
}
	
//------------------------------------------------------------------------------
