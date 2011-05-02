/** @file Retina.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/Retina.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool RetinaRenderer::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	if (desc.pRetina == NULL)
		return false;

	// clear memory buffer
	reset();

	if (desc.boundsShow) {
		const BoundingBox &bounds = *desc.pRetina->getRetinaBoundingBox();
		
		reserveLines(12);
		for (U32 i = 0, j = 0; j < 4; i += 3 - j, j++)
			for (U32 k = 0; k < 3; k++)
				addLine(bounds.getEdges()[i], bounds.getEdges()[i ^ (1 << k)], desc.boundsColour);
	}

	if (desc.retinaShow) {
		const Retina::ReceptiveField::Seq &rf = desc.pRetina->getActiveReceptiveFields();
		
		reservePoints((U32)rf.size());
		for (Retina::ReceptiveField::Seq::const_iterator i = rf.begin(); i != rf.end(); i++)
			addPoint(i->position, i->colour);
	}
	
	return true;
}

//------------------------------------------------------------------------------

Retina::Retina(Embodiment &embodiment) :
	Base(embodiment)
{
	pRetinaRenderer.reset(new RetinaRenderer);
}

Retina::~Retina() {
}

bool Retina::create(const Retina::Desc& desc) {
	Base::create(desc); // throws

	receptiveFieldNum = desc.receptiveFieldNum;	
	receptiveFieldGrid = desc.receptiveFieldGrid;
	receptiveFieldGridDiam = receptiveFieldGrid.magnitude();
	receptiveFieldGridDiamInv = REAL_ONE/receptiveFieldGridDiam;
	receptiveFieldDiam = desc.receptiveFieldDiamFac*receptiveFieldGridDiam;
	receptiveFieldDiamInv = REAL_ONE/receptiveFieldDiam;
	//context.getMessageStream()->write(Message::LEVEL_DEBUG, "Retina::create(): volume: %s", desc.volume ? "yes" : "no"));
	clusterDiam = desc.clusterDiam;
	jobTimeOut = desc.jobTimeOut;

	clustering = desc.clustering && receptiveFieldNum > clusterDiam;
	volume = desc.volume;

	// Retina bounding box
	BoundingBox::Desc boundingBoxDesc;
	boundingBoxDesc.dimensions.set(
		REAL_HALF*receptiveFieldGrid.v1*receptiveFieldNum.n1,
		REAL_HALF*receptiveFieldGrid.v2*receptiveFieldNum.n2,
		REAL_HALF*receptiveFieldGrid.v3*receptiveFieldNum.n3
	);
	boundingBoxDesc.pose = desc.retinaPose;
	retinaBoundingBox = boundingBoxDesc.create();
	if (retinaBoundingBox == NULL)
		throw MsgRetina(Message::LEVEL_CRIT, "Retina::create(): Unable to create retina bounding box");
	
	// Retina centre
	retinaPose = retinaBoundingBox->getPose();
	retinaPose.p.add(retinaPose.p, Vec3(
		-REAL_HALF*receptiveFieldGrid.v1*(receptiveFieldNum.n1 - 1),
		-REAL_HALF*receptiveFieldGrid.v2*(receptiveFieldNum.n2 - 1),
		-REAL_HALF*receptiveFieldGrid.v3*(receptiveFieldNum.n3 - 1)
	));

	retinaRendererDesc = desc.retinaRendererDesc;
	retinaRendererDesc.pRetina = this;
	
	return true;
}

void Retina::hierarchicalClustering() {
	const U32 numOfObjects = (U32)parallelsInp->objects.size();
	const Vec3 receptiveFieldRadius(REAL_HALF*receptiveFieldGrid.v1, REAL_HALF*receptiveFieldGrid.v2, REAL_HALF*receptiveFieldGrid.v3);
	const Real receptiveFieldShift = REAL_HALF*receptiveFieldDiam;
	
	for (;;) {
		(void)parallelsSm->wait(jobTimeOut);

		Cluster cluster;

		{
			CriticalSectionWrapper csw(parallelsCs);
			
			if (parallelsClusters.empty()) {
				// job done!
				if (parallelsNumOfJobs == 0) {
					return;
				}
				// some threads are still working
				else {
					continue;
				}
			}
			else {
				// Because the parallelsClusters is a LIFO queue, the procedure is a multithreaded deep first process.
				// FIFO queue would result in a resource hungry breath first type of a process.
				cluster = parallelsClusters.back();
				parallelsClusters.pop_back();
			}

			parallelsNumOfJobs++;
		}

		bool bUniform = false, bInterior;
		U32 uniformObjectIndex = numeric_const<U32>::MAX;
		
		const Image3::Idx3 diam(cluster.end.n1 - cluster.begin.n1, cluster.end.n2 - cluster.begin.n2, cluster.end.n3 - cluster.begin.n3);
		const U32 clusterDiam = (U32)std::min(diam.n1, std::min(diam.n2, diam.n3));
		
		if (clusterDiam >= this->clusterDiam) {
			Vec3 radius(receptiveFieldRadius.v1*diam.n1, receptiveFieldRadius.v2*diam.n2, receptiveFieldRadius.v3*diam.n3);
			Vec3 position(receptiveFieldGrid.v1*cluster.begin.n1 + radius.v1, receptiveFieldGrid.v2*cluster.begin.n2 + radius.v2, receptiveFieldGrid.v3*cluster.begin.n3 + radius.v3);
			parallelsOut->retina.getPose().multiply(position, position);
			
			Real distance = numeric_const<Real>::MAX;
			for (U32 j = 0; j < numOfObjects; j++) {
				// find a distance to the object surface
				const Bounds::Seq& boundsSeq = *parallelsInp->objects[j].pBoundsSeq;
				Real d = Bounds::getSurfaceDistance(boundsSeq.begin(), boundsSeq.end(), position);
				if (d < REAL_ZERO) {
					d = -d - (receptiveFieldDiam - receptiveFieldShift);
					if (d > REAL_ZERO)
						uniformObjectIndex = j;
				}
				else
					d = +d - (receptiveFieldDiam - receptiveFieldShift);
				
				if (distance > d)
					distance = d;
			}
			
			// Is the centre inside any object
			bInterior = uniformObjectIndex != numeric_const<U32>::MAX && volume;
			
			// partition the cluster if it's not uniform
			const Real clearRadius = radius.magnitude();
			if (distance < clearRadius) {
				const Image3::Idx3 centre(cluster.begin.n1 + diam.n1/2 + diam.n1%2, cluster.begin.n2 + diam.n2/2 + diam.n2%2, cluster.begin.n3 + diam.n3/2 + diam.n3%2);
				Cluster partition;
				
				CriticalSectionWrapper csw(parallelsCs);
				
				// sub-cluster #1 (0, 0, 0)
				partition.begin.set(cluster.begin.n1, cluster.begin.n2, cluster.begin.n3);
				partition.end.set(centre.n1, centre.n2, centre.n3);
				parallelsClusters.push_back(partition);
				
				// sub-cluster #2 (1, 0, 0)
				partition.begin.set(centre.n1, cluster.begin.n2, cluster.begin.n3);
				partition.end.set(cluster.end.n1, centre.n2, centre.n3);
				parallelsClusters.push_back(partition);

				// sub-cluster #3 (0, 1, 0)
				partition.begin.set(cluster.begin.n1, centre.n2, cluster.begin.n3);
				partition.end.set(centre.n1, cluster.end.n2, centre.n3);
				parallelsClusters.push_back(partition);

				// sub-cluster #4 (1, 1, 0)
				partition.begin.set(centre.n1, centre.n2, cluster.begin.n3);
				partition.end.set(cluster.end.n1, cluster.end.n2, centre.n3);
				parallelsClusters.push_back(partition);

				// sub-cluster #5 (0, 0, 1)
				partition.begin.set(cluster.begin.n1, cluster.begin.n2, centre.n3);
				partition.end.set(centre.n1, centre.n2, cluster.end.n3);
				parallelsClusters.push_back(partition);
				
				// sub-cluster #6 (1, 0, 1)
				partition.begin.set(centre.n1, cluster.begin.n2, centre.n3);
				partition.end.set(cluster.end.n1, centre.n2, cluster.end.n3);
				parallelsClusters.push_back(partition);
				
				// sub-cluster #7 (0, 1, 1)
				partition.begin.set(cluster.begin.n1, centre.n2, centre.n3);
				partition.end.set(centre.n1, cluster.end.n2, cluster.end.n3);
				parallelsClusters.push_back(partition);
				
				// sub-cluster #8 (1, 1, 1)
				partition.begin.set(centre.n1, centre.n2, centre.n3);
				partition.end.set(cluster.end.n1, cluster.end.n2, cluster.end.n3);
				parallelsClusters.push_back(partition);
				
				parallelsSm->release(8);
				parallelsNumOfJobs--;
				continue;
			}

			// if the cluster does not need painting
			if (!bInterior) {
				CriticalSectionWrapper csw(parallelsCs);
				if (--parallelsNumOfJobs == 0 && parallelsClusters.empty())
					parallelsSm->release(numeric_const<I32>::MAX);
				continue;
			}

			bUniform = true;
		}
		
		{
			CriticalSectionWrapper csw(parallelsCs);
			if (--parallelsNumOfJobs == 0 && parallelsClusters.empty())
				parallelsSm->release(numeric_const<I32>::MAX);
		}

		// create occupancy grid
		Image3::Idx3 index;
		for (index.n1 = cluster.begin.n1; index.n1 < cluster.end.n1; index.n1++)
			for (index.n2 = cluster.begin.n2; index.n2 < cluster.end.n2; index.n2++)
				for (index.n3 = cluster.begin.n3; index.n3 < cluster.end.n3; index.n3++) {
					// Receptive field position
					ReceptiveField rf;
					parallelsOut->retina.getPosition(rf.position, index);

					 // retrieve 1D index
					Image3::Index i = parallelsOut->retina.getIndex(index);

					if (bUniform) {
						// update objects' voxel occupancy arrays,
						// there can be only one object occupying the space
						for (U32 j = 0; j < numOfObjects; j++)
							parallelsOut->objects[j][i] = uniformObjectIndex == j ? Image3::ElementMax : Image3::ElementMin;
						
						// update retina voxel occupancy array
						parallelsOut->retina[i] = parallelsGrayLevels[uniformObjectIndex];
						
						if (!retinaRendererDesc.retinaShow)
							continue;
						
						rf.colour = retinaRendererDesc.colourShow ? parallelsInp->objects[uniformObjectIndex].colour : parallelsGrayColours[uniformObjectIndex];
						//rf.colour = bInterior ? RGBA::BLUE : RGBA::MAGENTA;
						
						CriticalSectionWrapper csw(parallelsRFCs);
						activeReceptiveFields.push_back(rf);

						continue;
					}

					Real alphaSum = REAL_ZERO, retinaColour[3] = {REAL_ZERO};
					
					for (U32 j = 0; j < numOfObjects; j++) {
						// distance to the object surface
						const Bounds::Seq& boundsSeq = *parallelsInp->objects[j].pBoundsSeq;
						Real d = Bounds::getSurfaceDistance(boundsSeq.begin(), boundsSeq.end(), rf.position);
						if (d < REAL_ZERO) {
							if (volume) {
								d += receptiveFieldShift;
								if (d < REAL_ZERO)
									d = REAL_ZERO;
							}
							else
								d = receptiveFieldShift - d;
						}
						else
							d += receptiveFieldShift;

						// normalised inverse distance to the object surface
						d = REAL_ONE - d*receptiveFieldDiamInv;
						
						if (d <= REAL_ZERO) {
							parallelsOut->objects[j][i] = Image3::ElementMin;
							continue;
						}
						
						// object colour
						const RGBA colour = parallelsInp->objects[j].colour;
						// alpha blending coefficients
						const Real alpha = d*colour._rgba.a;
						// retina colour (convex combination)
						retinaColour[0] += alpha*colour._rgba.r;
						retinaColour[1] += alpha*colour._rgba.g;
						retinaColour[2] += alpha*colour._rgba.b;
						// sum up weights
						alphaSum += alpha;
					}

					if (alphaSum < REAL_ONE) {
						// update retina voxel occupancy array
						parallelsOut->retina[i] = Image3::ElementMin;
						continue;
					}

					// normalise RGB colour
					retinaColour[0] *= parallelsAlphaNorm;
					retinaColour[1] *= parallelsAlphaNorm;
					retinaColour[2] *= parallelsAlphaNorm;
					// grayscale colour
					const Real gray = RGBToGray(retinaColour[0], retinaColour[1], retinaColour[2]);
					
					// update retina voxel occupancy array
					parallelsOut->retina[i] = (Image3::Element)Math::round(gray*(Real(Image3::ElementMax)/Real(numeric_const<U8>::MAX)));

					if (!retinaRendererDesc.retinaShow)
						continue;

					// Receptive field colour
					if (retinaRendererDesc.colourShow) {
						// colour
						rf.colour._rgba.r = (U8)Math::round(retinaColour[0]);
						rf.colour._rgba.g = (U8)Math::round(retinaColour[1]);
						rf.colour._rgba.b = (U8)Math::round(retinaColour[2]);
						rf.colour._rgba.a = numeric_const<U8>::MAX;
					}
					else {
						// grayscale
						rf.colour._rgba.r = (U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.r/Real(numeric_const<U8>::MAX));
						rf.colour._rgba.g = (U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.g/Real(numeric_const<U8>::MAX));
						rf.colour._rgba.b = (U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.b/Real(numeric_const<U8>::MAX));
						rf.colour._rgba.a = numeric_const<U8>::MAX;
					}

					CriticalSectionWrapper csw(parallelsRFCs);
					activeReceptiveFields.push_back(rf);
				}
	}
}

void Retina::linearOccupancy() {
	//size_t pi1 = (size_t)::InterlockedIncrement(parallelsIndex), pi0 = pi1 - 1;
	size_t pi1, pi0;
	{
		CriticalSectionWrapper csw(parallelsCs);
		pi1 = (size_t)++parallelsIndex; pi0 = pi1 - 1;
	}
	size_t i1 = (size_t)Math::round(parallelsStep*pi1), i0 = (size_t)Math::round(parallelsStep*pi0);

	const U32 numOfObjects = (U32)parallelsInp->objects.size();
	const Real receptiveFieldShift = REAL_HALF*receptiveFieldDiam;
	
	// create occupancy grid
	for (size_t i = i0; i < i1; i++) {
		// Receptive field position
		ReceptiveField rf;
		parallelsOut->retina.getPosition(rf.position, i);

		Real retinaColour[3] = {REAL_ZERO}, alphaSum = REAL_ZERO;
		
		for (U32 j = 0; j < numOfObjects; j++) {
			// distance to the object surface
			const Bounds::Seq& boundsSeq = *parallelsInp->objects[j].pBoundsSeq;
			Real d = Bounds::getSurfaceDistance(boundsSeq.begin(), boundsSeq.end(), rf.position);
			if (d < REAL_ZERO) {
				if (volume) {
					d += receptiveFieldShift;
					if (d < REAL_ZERO)
						d = REAL_ZERO;
				}
				else
					d = receptiveFieldShift - d;
			}
			else
				d += receptiveFieldShift;

			// normalised inverse distance to the object surface
			d = REAL_ONE - d*receptiveFieldDiamInv;
			
			if (d <= REAL_ZERO) {
				parallelsOut->objects[j][i] = Image3::ElementMin;
				continue;
			}
			
			// update object voxel occupancy array
			parallelsOut->objects[j][i] = (Image3::Element)Math::round(d*Real(Image3::ElementMax));
			
			// object colour
			const RGBA colour = parallelsInp->objects[j].colour;
			// alpha blending coefficients
			const Real alpha = d*colour._rgba.a;
			// retina colour (convex combination)
			retinaColour[0] += alpha*colour._rgba.r;
			retinaColour[1] += alpha*colour._rgba.g;
			retinaColour[2] += alpha*colour._rgba.b;
			alphaSum += alpha;
		}

		if (alphaSum < REAL_ONE) {
			// update retina voxel occupancy array
			parallelsOut->retina[i] = Image3::ElementMin;
			continue;
		}

		// normalise RGB colour
		retinaColour[0] *= parallelsAlphaNorm;
		retinaColour[1] *= parallelsAlphaNorm;
		retinaColour[2] *= parallelsAlphaNorm;
		// grayscale colour
		const Real gray = RGBToGray(retinaColour[0], retinaColour[1], retinaColour[2]);

		// update retina voxel occupancy array
		parallelsOut->retina[i] = (Image3::Element)Math::round(gray*(Real(Image3::ElementMax)/Real(numeric_const<U8>::MAX)));
		
		if (!retinaRendererDesc.retinaShow)
			continue;
		
		// Receptive field colour
		if (retinaRendererDesc.colourShow) {
			// colour
			rf.colour._rgba.r = (U8)Math::round(retinaColour[0]);
			rf.colour._rgba.g = (U8)Math::round(retinaColour[1]);
			rf.colour._rgba.b = (U8)Math::round(retinaColour[2]);
			rf.colour._rgba.a = numeric_const<U8>::MAX;
		}
		else {
			// grayscale
			rf.colour._rgba.r = (U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.r/Real(numeric_const<U8>::MAX));
			rf.colour._rgba.g = (U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.g/Real(numeric_const<U8>::MAX));
			rf.colour._rgba.b = (U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.b/Real(numeric_const<U8>::MAX));
			rf.colour._rgba.a = numeric_const<U8>::MAX;
		}

		// display receptive field only if it is visible
		CriticalSectionWrapper csw(parallelsRFCs);
		activeReceptiveFields.push_back(rf);
	}
}

void Retina::run() {
	if (clustering)
		hierarchicalClustering();
	else
		linearOccupancy();
}

bool Retina::process(RetinaOut &out, const RetinaInp &inp, MSecTmU32 timeOut) {
	const U32 numOfObjects = (U32)inp.objects.size();
	if (numOfObjects <= 0) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Retina::process(): No objects to render");
		return false;
	}

	Image3::Desc desc;
	desc.dimensions = receptiveFieldNum;
	desc.grid = receptiveFieldGrid;
	desc.pose = retinaPose;
	
	if (out.retina.isEmpty() && !out.retina.create(desc)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Retina::process(): Unable to create retina voxel array");
		return false;
	}

	if (clustering) {
		// set all to zeros
		out.retina.zero();
	}

	// do not release previous voxel arrays
	out.objects.resize(std::max(numOfObjects, (U32)out.objects.size()));
	for (U32 i = 0; i < numOfObjects; i++) {
		Image3 &image = out.objects[i];

		if (image.isEmpty() && !image.create(desc)) {
			context.getMessageStream()->write(Message::LEVEL_ERROR, "Retina::process(): Unable to create object voxel array");
			return false;
		}
		
		if (clustering) {
			// set all to zeros
			image.zero();
		}
	}

	// clear active retina receptive fields, but do not release memory
	this->activeReceptiveFields.resize(0);
	
	// Setup parallels
	Parallels *parallels = context.getParallels();
	if (parallels == NULL) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Retina::process(): Parallels has not been initialised");
		return false;
	}

	const U32 numOfElements = out.retina.getNumOfElements();
	this->parallelsNumOfThreads = std::max((U32)1, std::min(numOfElements/2, parallels->getNumOfThreads()));
	if (clustering) {
		this->parallelsNumOfJobs = 0;
		this->parallelsSm.reset(new Semaphore(1, numeric_const<I32>::MAX));
		this->parallelsClusters.resize(0);
		this->parallelsClusters.push_back(Cluster(out.retina.begin(), out.retina.end()));
		this->parallelsGrayLevels.resize(numOfObjects);
		this->parallelsGrayColours.resize(numOfObjects);
	}
	else {
		this->parallelsIndex = 0;
		this->parallelsStep = Real(numOfElements)/Real(parallelsNumOfThreads);
	}

	this->parallelsAlphaNorm = REAL_ZERO;
	for (U32 j = 0; j < numOfObjects; j++) {
		const RigidBodyData &object = inp.objects[j];
		
		if (clustering) {
			const Real gray =
				RGBToGray(object.colour._rgba.r, object.colour._rgba.g, object.colour._rgba.b)*
				object.colour._rgba.a*(Real(Image3::ElementMax)/(Real(numeric_const<U8>::MAX)*Real(numeric_const<U8>::MAX)));
			this->parallelsGrayLevels[j] =
				(Image3::Element)Math::round(gray*(Real(Image3::ElementMax)/Real(numeric_const<U8>::MAX)));
			this->parallelsGrayColours[j].set(
				(U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.r/Real(numeric_const<U8>::MAX)),
				(U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.g/Real(numeric_const<U8>::MAX)),
				(U8)Math::round(gray*retinaRendererDesc.retinaColour._rgba.b/Real(numeric_const<U8>::MAX)),
				numeric_const<U8>::MAX
			);
		}
		this->parallelsAlphaNorm += object.colour._rgba.a;
	}
	this->parallelsAlphaNorm = Real(numOfObjects)/parallelsAlphaNorm;

	this->parallelsInp = &inp;
	this->parallelsOut = &out;

	for (U32 i = 0; i < this->parallelsNumOfThreads; i++)
		parallels->startJob(this);

	if (!parallels->joinJobs(timeOut)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Retina::process(): Parallels time out");
		return false;
	}

	// redraw retina
	CriticalSectionWrapper csw(cs);
	
	if (retinaRendererDesc.boundsShow || retinaRendererDesc.retinaShow)
		pRetinaRenderer->create(retinaRendererDesc);
	
	return true;
}

void Retina::render() {
	// postprocess() and render() are always called from the same thread 
	CriticalSectionWrapper csw(cs);
	
	if (retinaRendererDesc.boundsShow || retinaRendererDesc.retinaShow)
		pRetinaRenderer->render();
}

void Retina::keyboardHandler(unsigned char key, int x, int y) {
	if (key == 5) {// F5
		CriticalSectionWrapper csw(cs);
		
		retinaRendererDesc.boundsShow = !retinaRendererDesc.boundsShow;
	}
	if (key == 6) {// F6
		CriticalSectionWrapper csw(cs);
		
		retinaRendererDesc.retinaShow = !retinaRendererDesc.retinaShow;
	}
	if (key == 7) {// F7
		CriticalSectionWrapper csw(cs);
		
		retinaRendererDesc.colourShow = !retinaRendererDesc.colourShow;
	}
	if (key == 8) {// F8
		CriticalSectionWrapper csw(cs);
		
		volume = !volume;
	}
}

//------------------------------------------------------------------------------
