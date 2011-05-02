/** @file Recognition.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Math/Quat.h>
#include <Golem/Demo/Common/Recognition.h>
#include <Golem/Demo/Common/Msg.h>
#include <algorithm>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool RealisationRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;
	if (desc.realisations == NULL) // nothing to render
		return true;
	const U32 numOfRealisations = std::min(desc.numOfRealisations, (U32)desc.realisations->size());
	if (numOfRealisations == 0)
		return true;

	// clear memory buffer
	reset();

	const Realisation* const* ppRealisation = &desc.realisations->front();
	for (U32 i = 0; i < numOfRealisations; i++) {
		addRealisation(NULL, ppRealisation[i], desc, desc.depth);
	}

	return true;
}

void RealisationRenderer::addRealisation(const Realisation* pUpper, const Realisation* pLower, const Desc& desc, U32 depth) {
	const Real shift1 = Real(depth)*desc.shift;
	const Real shift2 = Real(depth + 1)*desc.shift;
	Vec3 lower, upper, rotation3;
	
	Vec2 rotation2(REAL_ZERO, desc.rotation);
	Mat22 m(pLower->getPose().a);
	m.multiply(rotation2, rotation2);

	lower.set(pLower->getPose().p.v1, pLower->getPose().p.v2, shift1);
	rotation3.set(lower.v1 + rotation2.v1, lower.v2 + rotation2.v2, lower.v3); 
	desc.pose.multiply(lower, lower);
	desc.pose.multiply(rotation3, rotation3);
	this->addPoint(lower, desc.nodeColour);
	this->addLine(lower, rotation3, desc.rotationColour);
	
	if (pUpper != NULL) {
		upper.set(pUpper->getPose().p.v1, pUpper->getPose().p.v2, shift2);
		desc.pose.multiply(upper, upper);
		this->addLine(lower, upper, desc.edgeColour);
	}

	if (--depth <= 0)
		return;

	const Realisation::PtrSeq& parts = pLower->getParts();
	for (Realisation::PtrSeq::const_iterator i = parts.begin(); i != parts.end(); i++)
		addRealisation(pLower, *i, desc, depth);
}

//------------------------------------------------------------------------------

Recognition::Recognition(Embodiment &embodiment) :
	Base(embodiment),
	rand(context.getRandSeed())
{
}

Recognition::~Recognition() {
}

bool Recognition::create(const Recognition::Desc& desc) {
	Base::create(desc); // throws

	layers.clear();
	Layer* pLower = NULL;
	for (Layer::Desc::Seq::const_iterator i = desc.layersDesc.begin(); i != desc.layersDesc.end(); i++) {
		Layer::Ptr pLayer = (*i)->create(embodiment, pLower); // throws
		layers.push_back(pLayer);
		pLower = pLayer.get();
	}

	realisationRendererDesc = desc.realisationRendererDesc;
	sort = desc.sort;

	return true;
}

bool Recognition::process(RecognitionOut &out, const RecognitionInp &inp, MSecTmU32 timeOut) {
	if (inp.image == NULL || inp.image->isEmpty()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Recognition::process(): no specified image");
		return false;
	}

	// check if 2D image has to be re-initialised
	const Image2::Idx2 dimensions2(inp.image->getDimensions().n1, inp.image->getDimensions().n2); // ignore 3rd Z-dimension
	if (image.isEmpty() || dimensions2 != image.getDimensions()) {
		Image2::Desc desc;
		desc.dimensions = dimensions2;
		if (!image.create(desc)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Recognition::process(): unable to create input image"	);
			return false;
		}
	}
	
	const Vec2 grid2(inp.image->getGrid().v1, inp.image->getGrid().v2); // ignore 3rd Z-dimension
	image.setGrid(grid2); // set grid
	//image.setPose(...); // ID pose

	// create 2D image from the first layer (Z==0) in XY plane of the 3D image
	Image3::Idx3 begin3 = inp.image->begin(), end3 = inp.image->end(), i3;
	for (i3.n1 = begin3.n1; i3.n1 < end3.n1; i3.n1++)
		for (i3.n2 = begin3.n2; i3.n2 < end3.n2; i3.n2++) {
			i3.n3 = 0; // ignore 3rd Z-dimension
			Real img = image(Image2::Idx2(i3.n1, i3.n2)) = (*inp.image)(i3); // copy image
		}

	// process image
	if (!layers.front()->process(&image))
		return false;

	out.realisations.clear();
	const Layer::RealisationArr& realisationArr = layers.back()->getRealisationArr();
	Layer::RealisationArr::Idx2 begin2 = realisationArr.begin(), end2 = realisationArr.end(), i2;
	for (i2.n1 = begin2.n1; i2.n1 < end2.n1; i2.n1++)
		for (i2.n2 = begin2.n2; i2.n2 < end2.n2; i2.n2++) {
			const Realisation::Seq& realisationSeq = realisationArr(i2);
			
			for (Realisation::Seq::const_iterator j = realisationSeq.begin(); j != realisationSeq.end(); j++) {
				out.realisations.push_back(&*j);
			}
		}

	// sort
	if (sort)
		std::sort(out.realisations.begin(), out.realisations.end(), RealisationCmpr());

	//if (out.realisations.size() > 0)
	//	context.getMessageStream()->write(Message::LEVEL_DEBUG, "size = %d, energy = %f, %f, %f",
	//	out.realisations.size(), out.realisations.front()->getEnergy(), out.realisations[out.realisations.size()/2]->getEnergy(), out.realisations.back()->getEnergy()));

	// prepare renderer object description
	realisationRendererDesc.realisations = &out.realisations;
	realisationRendererDesc.pose = inp.image->getPose();
	realisationRendererDesc.depth = (U32)layers.size();

	// draw data
	CriticalSectionWrapper csw(cs);
	realisationRenderer.create(realisationRendererDesc);
	
	return true;
}

void Recognition::render() {
	// postprocess() and render() are always called from the same thread 
	CriticalSectionWrapper csw(cs);
	
	realisationRenderer.render();
}

void Recognition::keyboardHandler(unsigned char key, int x, int y) {
}

//------------------------------------------------------------------------------
