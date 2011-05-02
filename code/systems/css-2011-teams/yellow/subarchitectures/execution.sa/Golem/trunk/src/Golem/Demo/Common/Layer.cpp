/** @file Layer.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/Layer.h>
#include <Golem/Demo/Common/Msg.h>
#include <algorithm>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool Model::create(const Model::Desc& desc) {
	if (!desc.isValid())
		throw MsgLayer(Message::LEVEL_CRIT, "Model::create(): Invalid description");
	
	if (desc.filterDesc == NULL) {
		if (layer.isFirst())
			throw MsgLayer(Message::LEVEL_CRIT, "Model::create(): first layer must contain image filters");

		parts = desc.parts;
		for (Part::Seq::iterator i = parts.begin(); i != parts.end(); i++)
			i->init();

		setScale(REAL_ONE);
	}
	else {
		filter = desc.filterDesc->create(layer.getEmbodiment()); // throws
	}

	threshold = desc.threshold;
	norm = desc.norm;

	return true;
}

void Model::release() {
}

//------------------------------------------------------------------------------

Layer::Layer(golem::Embodiment &embodiment, Layer* pLower) :
	embodiment(embodiment), context(embodiment.getContext()), pLower(pLower)
{
}

Layer::~Layer() {
}

bool Layer::create(const Layer::Desc& desc) {
	if (!desc.isValid())
		throw MsgLayer(Message::LEVEL_CRIT, "Layer::create(): Invalid description");

	pUpper = NULL;
	if (pLower == NULL) {
		pFirst = this;
		sharedData.reset(new SharedData);
		sharedData->modelCounter = 0;
		
		index = 0;
		scale = REAL_ONE; // first layer is always the size of image
		scaleTotal = REAL_ONE;
	}
	else {
		pFirst = pLower->pFirst;
		pLower->pUpper = this;
		sharedData = pLower->sharedData;

		index = pLower->index + 1;
		scale = desc.scale;
		scaleTotal = scale*pLower->scaleTotal;
	}

	numOfRotations = desc.numOfRotations;
	rotationDelta = REAL_2_PI/Real(numOfRotations);

	numOfRealisations = desc.numOfRealisations;
	
	modelList.clear();
	for (Model::Desc::Seq::const_iterator i = desc.modelsDesc.begin(); i != desc.modelsDesc.end(); i++)
		create(**i); // throws

	return true;
}

Model* Layer::create(const Model::Desc& desc) {
	Model::Ptr pModel(desc.create(*this)); // throws

	pModel->sig = desc.sig == 0 ? ++sharedData->modelCounter : desc.sig;

	if (!sharedData->modelMap.insert(std::pair<ModelMap::key_type, ModelMap::mapped_type>(pModel->getSig(), pModel.get())).second)
		throw MsgLayer(Message::LEVEL_CRIT, "Layer::create(): model signature is not unique");
	
	modelList.push_back(ModelList::Pair(pModel.get(), pModel));
	return pModel.get();
}

void Layer::remove(Model& model) {
	if (!modelList.contains(&model))
		return;
	
	sharedData->modelMap.erase(model.getSig());
	
	modelList.erase(&model);
	model.release();
}

//------------------------------------------------------------------------------

void Layer::run() {
	ParallelsData data;
	
	// for all pixel positions in multiple threads
	for (;;) {
		{
			CriticalSectionWrapper csw(this->parallelsCs);
			if (this->parallelsIndex >= realisationArr.getNumOfElements())
				return;
			data.index = this->parallelsIndex++;
		}

		// setup parallels data
		realisationArr.getIndex(data.idx2, data.index);
		realisationArr.getPositionLocal(data.position, data.idx2); // local frame
		data.realisationRank.clear();
		data.realisationRank.reserve(numOfRealisations);
		data.responseSeq.resize(0);
		
		// for all models for a given position
		for (ModelList::const_iterator i = modelList.begin(); i != modelList.end(); i++) {
			const Model* pModel = *i;
			if (!processModel(pModel, data))
				return;
		}

		// reserve resources
		RealisationArr::Element& realisations = realisationArr[data.index];
		const size_t n = data.realisationRank.size();
		if (realisations.capacity() < n)
			realisations.reserve(n);
		realisations.resize(0);
		
		// copy realisations according model energy
		for (Realisation::Rank::iterator i = data.realisationRank.begin(); i != data.realisationRank.end(); i++) {
			Realisation &realisation = i->second;
			// compute signature
			realisation.computeSig();
			// add to the layer
			realisations.push_back(realisation);
		}
	}
}

//------------------------------------------------------------------------------

bool Layer::process(const Image2* pImage) {
	// proces layers up in hierarchy
	for (Layer* pCurrent = this->pFirst; pCurrent != NULL; pCurrent = pCurrent->pUpper)
		if (!pCurrent->processLayer(pImage))
			return false;

	return true;
}

bool Layer::processLayer(const Image2* pImage) {
	// layer can be created only from image
	if (pImage == NULL || pImage->isEmpty()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processLayer(): no specified image in layer %d", this->getIndex());
		return false;
	}
	this->pImage = pImage;

	// check if realisations array has to be re-initialised
	const RealisationArr::Idx2 dimensions(
		(RealisationArr::Index)Math::round(scaleTotal*pImage->getDimensions().n1),
		(RealisationArr::Index)Math::round(scaleTotal*pImage->getDimensions().n2)
	);
	if (realisationArr.isEmpty() || dimensions != realisationArr.getDimensions()) {
		RealisationArr::Desc desc;
		desc.dimensions = dimensions;
		if (!realisationArr.create(desc)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processLayer(): unable to create realisations array");
			return false;
		}
	}

	// update global pose and grid of the realisations array
	const Vec2 grid(
		pImage->getGrid().v1*Real(pImage->getDimensions().n1)/Real(realisationArr.getDimensions().n1),
		pImage->getGrid().v2*Real(pImage->getDimensions().n2)/Real(realisationArr.getDimensions().n2)
	);
	realisationArr.setGrid(grid);
	realisationArr.setPose(pImage->getPose());
	
	// layer cannot be processed without models
	if (modelList.empty()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processLayer(): no specified models in layer %d", this->getIndex());
		return false;
	}
	
	// setup model scale
	const Real scale = grid.magnitude()/Math::sqrt(Real(2.0));
	for (ModelList::const_iterator i = modelList.begin(); i != modelList.end(); i++)
		(*i)->setScale(scale);
	
	// Setup parallels
	Parallels *parallels = context.getParallels();
	if (parallels == NULL) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processLayer(): Parallels has not been initialised");
		return false;
	}
	
	this->parallelsIndex = 0;
	const U32 numOfThreads = std::max(
		(U32)1, std::min((U32)realisationArr.getNumOfElements(), parallels->getNumOfThreads())
	);

	// run parallels' threads
	for (U32 i = 0; i < numOfThreads; i++)
		parallels->startJob(this);

	// wait for completion
	if (!parallels->joinJobs()) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processLayer(): Parallels time out in layer %d", this->getIndex());
		return false;
	}

	return true;
}

bool Layer::processModel(const Model* pModel, ParallelsData& data) const {
	// create realisation template
	Realisation realisationTemplate;
	// assign model
	realisationTemplate.model = pModel;
	
	const ImageFilter2* filter = pModel->getFilter();
	if (filter != NULL) {
		// image filter
		
		// find response for all orientations
		Image2::Idx2 idx2; // index in the image
		pImage->findIndexLocal(idx2, data.position); // local frame
		filter->response(data.responseSeq, idx2, *pImage);
		
		const U32 numOfRotations = std::min(filter->getNumOfRotations(), (U32)data.responseSeq.size());
		for (U32 i = 0; i < numOfRotations; i++) {
			Response& response = data.responseSeq[i];

			// compute realisation energy
			realisationTemplate.energy = pModel->norm*response.filter;

			// reject realisations of low energies
			if (realisationTemplate.energy <= pModel->threshold)
				continue;
			
			// update pose
			realisationTemplate.pose.set(Math::normaliseRad(filter->getRotation(i)), data.position);

			// add realisation
			data.realisationRank.insert(realisationTemplate.energy, realisationTemplate);
		}
	}
	else {
		// composition of parts

		// model parts
		const Part::Seq &parts = pModel->getParts();
		const U32 numOfParts = (U32)parts.size();
		
		// this should never happen, but... (see Model::create())
		if (numOfParts <= 0) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processModel(): no model parts to process"	);
			return false;
		}
		
		// do not reallocate memory if it is not necessary
		data.realisationClassRankSeq.resize(std::max((U32)data.realisationClassRankSeq.size(), numOfParts));

		// set maximum number of models (from the lower layer) for each part
		for (U32 j = 0; j < numOfParts; j++) {
			Realisation::ClassRank& rank = data.realisationClassRankSeq[j];
			const Part& part = parts[j];
			rank.reserve(part.numOfModels);
		}
		
		// this should never happen, but... (see Model::create())
		if (pLower == NULL) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processModel(): first layer cannot process compositions");
			return false;
		}

		// lower layer realisation array
		const RealisationArr& lowerRealisationArr = pLower->getRealisationArr();

		// for all rotations for a given model
		for (U32 i = 0; i < numOfRotations; i++) {
			// model pose in the image frame
			const Pose2 modelPose(getRotation(i), data.position);
			const Mat23 modelPoseMat(modelPose.a, modelPose.p);

			// for all parts of a given model
			for (U32 j = 0; j < numOfParts; j++) {
				Realisation::ClassRank& rank = data.realisationClassRankSeq[j];
				rank.clear();

				const Part& part = parts[j];
				// part pose in the image frame (not the model frame)
				Pose2 partPose;
				partPose.a = Math::normaliseRad(modelPose.a + part.localPose.a);
				modelPoseMat.multiply(partPose.p, part.localPose.p);

				// find region of the lower layer
				RealisationArr::Idx2 begin, end;
				lowerRealisationArr.findRangeLocal(begin, end, partPose.p, part.linVar);

				// search for parts
				// partEnergy = submodelEnergy * C * exp(- Clin * dr^2 - Cang * da^2)
				RealisationArr::Idx2 k;
				for (k.n1 = begin.n1; k.n1 < end.n1; k.n1++)
					for (k.n2 = begin.n2; k.n2 < end.n2; k.n2++) {
						const RealisationArr::Index lowerIndex = lowerRealisationArr.getIndex(k);
						const Realisation::Seq& lowerRealisations = lowerRealisationArr[lowerIndex];
						if (lowerRealisations.empty())
							continue;

						Vec2 r;
						lowerRealisationArr.getPositionLocal(r, k);
						r.subtract(r, partPose.p);
						const Real dr2 = r.magnitudeSquared();
						const Real dr = Math::sqrt(dr2);
						
						// check if location lies beyond range
						if (dr > part.linVar)
							continue;

						// C1 = - Clin * dr^2
						const Real C1 = -part.Clin*dr2;
						
						for (Realisation::Seq::const_iterator l = lowerRealisations.begin(); l != lowerRealisations.end(); l++) {
							const Realisation& lowerRealisation = *l;
							const Real da2 = Math::sqr(Math::diffRadNormalised(lowerRealisation.pose.a, partPose.a));
							const Real C2 = -part.Cang*da2;
							const Real energy = part.C*lowerRealisation.energy*Math::exp(C1 + C2);

							// check if the part energy is above threshold
							if (energy <= part.threshold)
								continue;

							//rank.insert(energy, lowerRealisation.sig, &lowerRealisation);
							rank.insert(energy, lowerRealisation.getModel()->getSig(), &lowerRealisation);
						}
					}
			}

			// data.realisationClassRankSeq[] contains all potential parts of a realisation of given model
			// which energy is above required part energy threshold.
			// Each part 'p_i' contains 'pm_i' lower layer models (no larger than 'numOfModels' for a given 'p_i')
			// For 'n' parts there can be 'pm_1*pm_2*...*pm_n' different realisations created.
			// Their energies can be checked in a recursive way:
			
			realisationTemplate.parts.resize(numOfParts);
			realisationTemplate.pose = modelPose;

			if (!processRealisation(0, REAL_ZERO, realisationTemplate, data)) {
				context.getMessageStream()->write(Message::LEVEL_ERROR, "Layer::processModel(): unable to process realisation"	);
				return false;
			}
		}
	}

	return true;
}

bool Layer::processRealisation(U32 index, Real energy, Realisation& realisation, ParallelsData& data) const {
	const Realisation::ClassRank& rank = data.realisationClassRankSeq[index];
	const Model& model = *realisation.model;
	const U32 end = (U32)realisation.parts.size();
	
	// current realisation part
	Realisation::PtrSeq::reference part = realisation.parts[index];

	// increment part index
	index++;

	for (Realisation::ClassRank::const_iterator i = rank.begin(); i != rank.end(); i++) {
		part = i->second.second; // pointer to the lower model
		const Real partEnergy = energy + i->first;
		
		if (index < end) {
			if (!processRealisation(index, partEnergy, realisation, data))
				return false;
		}
		else {
			realisation.energy = model.norm*partEnergy;
			if (realisation.energy > model.threshold)
				data.realisationRank.insert(realisation.energy, realisation);
		}
	}

	return true;
}

//------------------------------------------------------------------------------
