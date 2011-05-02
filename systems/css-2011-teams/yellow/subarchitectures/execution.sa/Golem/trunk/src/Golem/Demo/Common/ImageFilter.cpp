/** @file ImageFilter.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/ImageFilter.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

ImageFilter2::ImageFilter2(golem::Embodiment &embodiment) :
	embodiment(embodiment), context(embodiment.getContext())
{}

bool ImageFilter2::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgImageFilter(Message::LEVEL_CRIT, "ImageFilter2::create(): Invalid description");
	
	radius = desc.radius;
	diameter.set(2*radius.n1 + 1, 2*radius.n2 + 1);
	
	envelopSymmetry = desc.envelopSymmetry;

	numOfRotations = desc.numOfRotations;
	rotationDelta = REAL_2_PI/Real(numOfRotations);

	sigma = desc.sigma;
	sigmaFac.set(-REAL_HALF/(sigma.v1*sigma.v1), -REAL_HALF/(sigma.v2*sigma.v2));
	C = REAL_ONE/(REAL_2_PI * sigma.v1 * sigma.v2);
	//C = REAL_ONE;
	
	templates = true;//desc.templates;// HACK
	
	if (templates && !createTemplates())
		throw MsgImageFilter(Message::LEVEL_CRIT, "ImageFilter2::create(): unable to create filter templates");

	return true;
}

bool ImageFilter2::createTemplates() {
	Idx2 i;
	
	// envelop
	envelopes.resize(envelopSymmetry ? 1 : numOfRotations);
	weights.resize(envelopSymmetry ? 1 : numOfRotations);
	for (U32 j = 0; j < envelopes.size(); j++) {
		Data& envelop = envelopes[j];
		if (!envelop.create(diameter))
			return false;

		const Mat22 rotationInv(getRotationInv(j));
		Real weight = Real(0.0);

		for (i.n1 = 0; i.n1 < diameter.n1; i.n1++)
			for (i.n2 = 0; i.n2 < diameter.n2; i.n2++) {
				Vec2 location(Real(i.n1 - radius.n1), Real(i.n2 - radius.n2));
				rotationInv.multiply(location, location);
				weight += envelop(i) = getEnvelop(location);
			}

		if (Math::equals(weight, REAL_ZERO, REAL_EPS))
			return false;
		weights[j] = weight;
	}

	// carrier
	carriers.resize(numOfRotations);
	for (U32 j = 0; j < carriers.size(); j++) {
		Data& envelop = envelopes[envelopSymmetry ? 0 : j];
		Data& carrier = carriers[j];
		if (!carrier.create(diameter))
			return false;

		const Mat22 rotationInv(getRotationInv(j));

		for (i.n1 = 0; i.n1 < diameter.n1; i.n1++)
			for (i.n2 = 0; i.n2 < diameter.n2; i.n2++) {
				Vec2 location(Real(i.n1 - radius.n1), Real(i.n2 - radius.n2));
				rotationInv.multiply(location, location);
				carrier(i) = envelop(i)*getCarrier(location);
			}
	}

	return true;
}

void ImageFilter2::response(Response::Seq& response, const Idx2& location, const Image2& image) const {
	const Idx2 begin(
		std::max(image.begin().n1, location.n1 - radius.n1),
		std::max(image.begin().n2, location.n2 - radius.n2)
	);
	const Idx2 end(
		std::min(image.end().n1, location.n1 + radius.n1 + 1),
		std::min(image.end().n2, location.n2 + radius.n2 + 1)
	);
	const Idx2 offset(
		radius.n1 - location.n1,
		radius.n2 - location.n2
	);
	Idx2 i;

	response.resize(numOfRotations);
	Response* responses = &response.front();

	// envelopes data
	const U32 numOfEnvelopes = envelopes.size();
	for (U32 k = 0; k < numOfEnvelopes; k++)
		responses[k].envelop = Real(0.0);

	// find image mean values for all envelopes
	const Data* envelopes = &this->envelopes.front();
	for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
		for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++) {
			const Real img = image(i);
			const Index j = envelopes[0].getIndex(i.n1 + offset.n1, i.n2 + offset.n2);

			for (U32 k = 0; k < numOfEnvelopes; k++)
				responses[k].envelop += envelopes[k][j]*img;
		}

	// normalise
	for (U32 k = 0; k < numOfEnvelopes; k++)
		responses[k].envelop /= this->weights[k];
	for (U32 k = numOfEnvelopes; k < numOfRotations; k++)
		responses[k].envelop = responses[0].envelop;

	// carriers data
	for (U32 k = 0; k < numOfRotations; k++)
		responses[k].filter = Real(0.0);

	// find filter energy for all rotations
	const Data* carriers = &this->carriers.front();
	for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
		for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++) {
			const Real img = image(i);
			const Index j = carriers[0].getIndex(i.n1 + offset.n1, i.n2 + offset.n2);

			for (U32 k = 0; k < numOfRotations; k++)
				responses[k].filter += carriers[k][j]*(img - responses[k].envelop);
		}
}

//------------------------------------------------------------------------------

GaborFilter2::GaborFilter2(golem::Embodiment &embodiment) :
	ImageFilter2(embodiment)
{}

bool GaborFilter2::create(const Desc &desc) {
	lambda = desc.lambda;
	phi = desc.phi;
	D = REAL_2_PI/lambda;
	
	return ImageFilter2::create(desc);
}

//------------------------------------------------------------------------------

ImageFilter3::ImageFilter3(golem::Embodiment &embodiment) :
	embodiment(embodiment), context(embodiment.getContext())
{}

bool ImageFilter3::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgImageFilter(Message::LEVEL_CRIT, "ImageFilter3::create(): Invalid description");
	
	radius = desc.radius;
	diameter.set(2*radius.n1 + 1, 2*radius.n2 + 1, 2*radius.n3 + 1);
	
	rotations = desc.rotationsDesc->create(embodiment); // throws

	numOfRotations = rotations->getRotations().size();
	envelopSymmetry = desc.envelopSymmetry;
	carrierDegeneracy = rotations->isDegenerated();

	sigma = desc.sigma;
	sigmaFac.set(-REAL_HALF/(sigma.v1*sigma.v1), -REAL_HALF/(sigma.v2*sigma.v2), -REAL_HALF/(sigma.v3*sigma.v3));
	C = REAL_ONE/(Math::pow(REAL_2_PI, Real(3.0)/Real(2.0)) * sigma.v1 * sigma.v2 * sigma.v3);
	//C = REAL_ONE;
	
	templates = true;//desc.templates;// HACK
	
	if (templates && !createTemplates())
		throw MsgImageFilter(Message::LEVEL_CRIT, "ImageFilter3::create(): unable to create filter templates");

	return true;
}

bool ImageFilter3::createTemplates() {
	Idx3 i;
	
	// envelop
	envelopes.resize(envelopSymmetry ? 1 : numOfRotations);
	weights.resize(envelopSymmetry ? 1 : numOfRotations);
	for (U32 j = 0; j < envelopes.size(); j++) {
		Data& envelop = envelopes[j];
		if (!envelop.create(diameter))
			return false;

		const Rotation& rotation = rotations->getRotations()[j];
		Real weight = Real(0.0);

		for (i.n1 = 0; i.n1 < diameter.n1; i.n1++)
			for (i.n2 = 0; i.n2 < diameter.n2; i.n2++)
				for (i.n3 = 0; i.n3 < diameter.n3; i.n3++) {
					Vec3 location(Real(i.n1 - radius.n1), Real(i.n2 - radius.n2), Real(i.n3 - radius.n3));
					rotation.multiplyByInverse(location, location);
					weight += envelop(i) = getEnvelop(location);
				}

		if (Math::equals(weight, REAL_ZERO, REAL_EPS))
			return false;
		weights[j] = weight;
	}

	// carrier
	carriers.resize(numOfRotations);
	for (U32 j = 0; j < carriers.size(); j++) {
		Data& envelop = envelopes[envelopSymmetry ? 0 : j];
		Data& carrier = carriers[j];
		if (!carrier.create(diameter))
			return false;

		const Rotation& rotation = rotations->getRotations()[j];

		for (i.n1 = 0; i.n1 < diameter.n1; i.n1++)
			for (i.n2 = 0; i.n2 < diameter.n2; i.n2++)
				for (i.n3 = 0; i.n3 < diameter.n3; i.n3++) {
					Vec3 location(Real(i.n1 - radius.n1), Real(i.n2 - radius.n2), Real(i.n3 - radius.n3));
					rotation.multiplyByInverse(location, location);
					carrier(i) = envelop(i)*getCarrier(location);
				}
	}

	return true;
}

void ImageFilter3::response(Response::Seq& response, const Idx3& location, const Image3& image) const {
	//const golem::Rotations::Rotation::Seq& rotations = getRotations();
	const Idx3 begin(
		std::max(image.begin().n1, location.n1 - radius.n1),
		std::max(image.begin().n2, location.n2 - radius.n2),
		std::max(image.begin().n3, location.n3 - radius.n3)
	);
	const Idx3 end(
		std::min(image.end().n1, location.n1 + radius.n1 + 1),
		std::min(image.end().n2, location.n2 + radius.n2 + 1),
		std::min(image.end().n3, location.n3 + radius.n3 + 1)
	);
	const Idx3 offset(
		radius.n1 - location.n1,
		radius.n2 - location.n2,
		radius.n3 - location.n3
	);
	Idx3 i;

	response.resize(numOfRotations);
	Response* responses = &response.front();
	
	// envelopes data
	const U32 numOfEnvelopes = envelopes.size();
	for (U32 k = 0; k < numOfEnvelopes; k++)
		responses[k].envelop = Real(0.0);

	// find image mean values for all envelopes
	const Data* envelopes = &this->envelopes.front();
	for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
		for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++)
			for (i.n3 = begin.n3; i.n3 < end.n3; i.n3++) {
				const Real img = image(i);
				const Index j = envelopes[0].getIndex(i.n1 + offset.n1, i.n2 + offset.n2, i.n3 + offset.n3);

				for (U32 k = 0; k < numOfEnvelopes; k++)
					responses[k].envelop += envelopes[k][j]*img;
			}

	// normalise
	for (U32 k = 0; k < numOfEnvelopes; k++)
		responses[k].envelop /= this->weights[k];
	for (U32 k = numOfEnvelopes; k < numOfRotations; k++)
		responses[k].envelop = responses[0].envelop;

	// carriers data
	for (U32 k = 0; k < numOfRotations; k++)
		responses[k].filter = Real(0.0);

	// find filter energy for all rotations
	const Data* carriers = &this->carriers.front();
	for (i.n1 = begin.n1; i.n1 < end.n1; i.n1++)
		for (i.n2 = begin.n2; i.n2 < end.n2; i.n2++)
			for (i.n3 = begin.n3; i.n3 < end.n3; i.n3++) {
				const Real img = image(i);
				const Index j = carriers[0].getIndex(i.n1 + offset.n1, i.n2 + offset.n2, i.n3 + offset.n3);

				for (U32 k = 0; k < numOfRotations; k++)
					responses[k].filter += carriers[k][j]*(img - responses[k].envelop);
			}
}

//------------------------------------------------------------------------------

GaborFilter3::GaborFilter3(golem::Embodiment &embodiment) :
	ImageFilter3(embodiment)
{}

bool GaborFilter3::create(const Desc &desc) {
	lambda = desc.lambda;
	phi = desc.phi;
	D = REAL_2_PI/lambda;
	
	return ImageFilter3::create(desc);
}

//------------------------------------------------------------------------------
