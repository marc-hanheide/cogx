/** @file Rotations.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/Rotations.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const std::string Rotations::Header::NAME = "Rotations";

Rotations::Rotations(golem::Embodiment &embodiment) :
	embodiment(embodiment),
	context(embodiment.getContext()),
	rand(context.getRandSeed())
{}

bool Rotations::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgRotations(Message::LEVEL_CRIT, "Rotations::create(): Invalid description");
	
	symmetryAxis = desc.symmetryAxis;
	degeneracy = desc.degeneracy;
	
	if (desc.rotationsPath.empty() || !load(desc.rotationsPath))
		generate(desc.numOfRotations);
	if (!desc.rotationsPath.empty())
		(void)store(desc.rotationsPath);
	
	return true;
}

void Rotations::next(Real *t, U32 n) const {
	Real m;
	
	do {
		m = 0;
		for (U32 i = 0; i < n; i++)
			m += Math::sqr(t[i] = rand.nextUniform(-REAL_ONE, +REAL_ONE));
		m = Math::sqrt(m);
	} while (m > REAL_ONE - REAL_EPS || m < REAL_EPS);
	
	m = REAL_ONE/m;
	for (U32 i = 0; i < n; i++)
		t[i] *= m;
}

bool Rotations::get(Quat &q, const Vec3& a) const {
	Vec3 axis;
	axis.cross(symmetryAxis, a);
	
	if (axis.magnitude() < REAL_EPS)
		return false;

	axis.normalise();
	Real angle = Math::acos(symmetryAxis.dot(a));
	q.fromAngleAxis(angle, axis);

	return true;
}

bool Rotations::get(Vec3& a, const Quat &q) const {
	q.multiply(a, symmetryAxis);
	return true;
}

void Rotations::generate(U32 numOfRotations) {
	context.getMessageStream()->write(Message::LEVEL_INFO, "Rotations::generate(): generating %d data items...", numOfRotations);
	
	rotations.clear();
	
	for (U32 i = 0; i < numOfRotations;) {
		Rotation rotation;
		
		if (i == 0) {
			rotation.setId();
		}
		else if (degeneracy) {
			Vec3 a;
			next(&a.v1, 3);
			if (!get(rotation, a))
				continue;
		}
		else {
			//Vec3 a;
			rotation.next(rand);
			//if (!get(a, rotation))
			//	continue;
		}

		rotations.push_back(rotation);
		
		i++;
	}
}

bool Rotations::load(const std::string& rotationsPath) {
	FileReadStream stream(rotationsPath.c_str());

	Header header;
	header.load(stream);
	if (!header.isValid()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Rotations::load(): invalid file header");
		return false;
	}
	
	symmetryAxis = header.symmetryAxis;
	degeneracy = header.degeneracy;

	context.getMessageStream()->write(Message::LEVEL_INFO, "Rotations::load(): loading %d data items...", header.items);
	
	rotations.clear();
	stream.read(rotations, rotations.begin());

	return true;
}

bool Rotations::store(const std::string& rotationsPath) const {
	FileWriteStream stream(rotationsPath.c_str());
	
	Header header;
	header.setToDefault();
	header.symmetryAxis = symmetryAxis;
	header.degeneracy = degeneracy;
	header.items = (U32)rotations.size();
	header.store(stream);
	
	stream.write(rotations.begin(), rotations.end());
	
	return true;
}

//------------------------------------------------------------------------------
