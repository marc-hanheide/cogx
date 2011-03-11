/*
 * AVS_ContinualPlanner.cpp
 *
 *  Created on: Mar 1, 2011
 *      Author: alper
 *
 *      TODO
 *      1. Create bloxel maps for each object in generateViewCones
 *      2. in processConeGroup update each of these maps with according to cone_depth
 *      3. During ViewConeUpdate put the ObjectSearchResult for all objects there
 *      4. When a new object belief arrives place ObjectPlaceProperty in WM -- Done
 *
 */

#include <CureHWUtils.hpp>
#include <AddressBank/ConfigFileReader.hh>

#include "AVS_ContinualPlanner.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/core/CASTUtils.hpp>
#include "ComaData.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include "GridDataFunctors.hh"
#include "GridDataFunctors.hh"

#include "PBVisualization.hh"

using namespace cast;
using namespace std;
using namespace boost;
using namespace SpatialGridMap;
using namespace cogx;
using namespace Math;
using namespace de::dfki::lt::tr::beliefs::slice;
using namespace de::dfki::lt::tr::beliefs::slice::sitbeliefs;
using namespace de::dfki::lt::tr::beliefs::slice::distribs;
using namespace de::dfki::lt::tr::beliefs::slice::logicalcontent;
using namespace eu::cogx::beliefs::slice;

namespace spatial {


extern "C" {
cast::CASTComponentPtr newComponent() {
	return new AVS_ContinualPlanner();
}
}

AVS_ContinualPlanner::AVS_ContinualPlanner() :
	m_sampler(&m_relationEvaluator) {
	// TODO Auto-generated constructor stub
	m_defaultBloxelCell.pdf = 0;
	m_defaultBloxelCell.occupancy = SpatialGridMap::UNKNOWN;
	m_gotPC = false;
	m_gotNewGenerateViewCone = false;
	m_currentVPGenerationCommand = new SpatialData::RelationalViewPointGenerationCommand;

}

AVS_ContinualPlanner::~AVS_ContinualPlanner() {
	// TODO Auto-generated destructor stub
}

void AVS_ContinualPlanner::runComponent() {
	log("I am running");

	while(isRunning()){
		if(m_gotNewGenerateViewCone){
			generateViewCones(m_currentVPGenerationCommand, m_generateViewConesCommandWMAddress);

		}
		sleepComponent(500);
	}
}


void
AVS_ContinualPlanner::owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID) {
try {
  log("got weighted PC");
  FrontierInterface::ObjectPriorRequestPtr req =
    getMemoryEntry<FrontierInterface::ObjectPriorRequest>(objID.address);
  FrontierInterface::WeightedPointCloudPtr cloud =
    req->outCloud;

  m_cloud = req->outCloud;
  receivePointCloud(cloud, req->totalMass);
}
catch (DoesNotExistOnWMException excp) {
  log("Error!  WeightedPointCloud does not exist on WM!");
  return;
}
}


void
AVS_ContinualPlanner::receivePointCloud(FrontierInterface::WeightedPointCloudPtr cloud, double totalMass)
	{
	  log("KDEing cloud read from file");
	    vector<Vector3> centers;
	    centers.push_back(cloud->center);

	    if(cloud->isBaseObjectKnown) {
	      log("Got distribution around known object pose");
	      m_sampler.kernelDensityEstimation3D(*m_currentBloxelMap,
		  centers,
		  cloud->interval,
		  cloud->xExtent,
		  cloud->yExtent,
		  cloud->zExtent,
		  cloud->values,
		  1.0,
		  totalMass,
		  m_currentCureObstMap
		  );
	      normalizePDF(*m_currentBloxelMap,totalMass);
	    }
	    else {
	    	log("Base object is not known! This means support object is not known to ObjectRelationManager. Something is very wrong!");
	    }

	    if(m_usePeekabot)
	      pbVis->AddPDF(*m_currentBloxelMap);
	    m_currentBloxelMap->clearDirty();
	    m_gotPC = true;
	}

void AVS_ContinualPlanner::start() {
	//Todo subscribe to View Cone Generation

	addChangeFilter(createGlobalTypeFilter<
			SpatialData::RelationalViewPointGenerationCommand> (cdl::ADD),
			new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
					&AVS_ContinualPlanner::newViewPointGenerationCommand));

	addChangeFilter(createGlobalTypeFilter<
			SpatialData::ProcessConeGroup> (cdl::ADD),
			new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
					&AVS_ContinualPlanner::newProcessConeCommand));

	addChangeFilter(createGlobalTypeFilter<
			FrontierInterface::ObjectPriorRequest> (cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
					&AVS_ContinualPlanner::owtWeightedPointCloud));

	addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::ADD),
			new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
					&AVS_ContinualPlanner::newRobotPose));

	// TODO: add code to take in spatial objects
	/*	addChangeFilter(
	 createLocalTypeFilter<SpatialData::SpatialObject> (cdl::ADD),
	 new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
	 &AVS_ContinualPlanner::newSpatialObject));*/

	 addChangeFilter(cast::createGlobalTypeFilter<GroundedBelief>(cast::cdl::ADD),
			            new cast::MemberFunctionChangeReceiver<AVS_ContinualPlanner>(this, &AVS_ContinualPlanner::newGroundedBelief));


	addChangeFilter(
			createLocalTypeFilter<NavData::RobotPose2d> (cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
					&AVS_ContinualPlanner::newRobotPose));
	log("getting ice queryHandlerServer");
	m_queryHandlerServerInterfacePrx = getIceServer<
			ConceptualData::QueryHandlerServerInterface> (m_queryHandlerName);
}

void AVS_ContinualPlanner::newProcessConeCommand(
		const cast::cdl::WorkingMemoryChange &objID) {
log("Got Process Cone Group Command");
	SpatialData::ProcessConeGroupPtr cmd = getMemoryEntry<SpatialData::ProcessConeGroup> (
						objID.address);
	m_processConeGroupCommandWMAddress = objID.address.id;
	m_currentProcessConeGroup = cmd;
	processConeGroup(cmd->coneId);
}

void AVS_ContinualPlanner::newGroundedBelief(
		const cast::cdl::WorkingMemoryChange &objID) {
	// If this is an visualobject belief then add as an ObjectPlaceProperty
	// Based on the current ConeGroup know the location of the object
	dBeliefPtr belief = new dBelief;
	belief = getMemoryEntry<dBelief>(objID.address);

	if (belief->type == "VisualObject"){
		// Let's get it's Id;

		log("Got a new Visual Object!");
		CondIndependentDistribsPtr dist(CondIndependentDistribsPtr::dynamicCast(belief->content));
		BasicProbDistributionPtr  basicdist(BasicProbDistributionPtr::dynamicCast(dist->distribs["label"]));
		FormulaValuesPtr formulaValues(FormulaValuesPtr::dynamicCast(basicdist->values));
		ElementaryFormulaPtr elformula(ElementaryFormulaPtr::dynamicCast(formulaValues->values[0].val));
		log("Visual Object Id: %s", elformula->prop.c_str());

		m_fromBeliefIdtoVisualLabel[objID.address.id] = elformula->prop.c_str();

		SpatialProperties::ObjectPlacePropertyPtr result = new SpatialProperties::ObjectPlaceProperty;
		result->category = m_currentConeGroup.searchedObjectCategory;
		result->relation = m_currentConeGroup.relation;
		result->supportObjectCategory = m_currentConeGroup.supportObjectCategory;
		result->supportObjectId = m_currentConeGroup.supportObjectId;
		result->placeId = m_currentConeGroup.placeId;
	  	log("Publishing ObjectPlaceProperty with: category: %s, relation: %s, supportObjectCategory: %s, supportObjectId: %s",
	  			result->category.c_str(), relationToString(result->relation).c_str(), result->supportObjectCategory.c_str(), result->supportObjectId.c_str());
	  	addToWorkingMemory(newDataID(), result);
	}

}
void AVS_ContinualPlanner::newViewPointGenerationCommand(
		const cast::cdl::WorkingMemoryChange &objID) {
	 try{
	log("got new GenerateViewPointCommand");
	SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand = getMemoryEntry<SpatialData::RelationalViewPointGenerationCommand> (
					objID.address);
	SpatialData::RelationalViewPointGenerationCommandPtr cmd = new SpatialData::RelationalViewPointGenerationCommand(*newVPCommand);

	m_currentVPGenerationCommand = cmd;
	m_gotNewGenerateViewCone = true;
	m_generateViewConesCommandWMAddress= objID.address.id;

	//
	 }
	 catch (const CASTException &e) {
	 	    log("owtRecognizer3DCommand disappeared from WM: %s", e.message.c_str());

	 	  }
}


/* Generate view cones for <object,relation , object/room, room> */
void AVS_ContinualPlanner::generateViewCones(
		SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand, std::string WMAddress) {

	m_gotNewGenerateViewCone = false;

	string plus = "p(+";
			string closebracket = ")";
		string id = plus + m_namegenerator.getUnexploredObjectVarName(newVPCommand->roomId, newVPCommand->searchedObjectCategory, newVPCommand->relation,
				newVPCommand->supportObjectCategory, newVPCommand->supportObject) + closebracket;

		log("Generating View Cones for %s", id.c_str());

	// if we already don't have a room map for this then get the combined map
	if (m_templateRoomBloxelMaps.count(newVPCommand->roomId) == 0) {
		log("Creating a new BloxelMap for room: %d", newVPCommand->roomId);
		FrontierInterface::LocalGridMap combined_lgm;

		vector<comadata::ComaRoomPtr> comarooms;
		getMemoryEntries<comadata::ComaRoom> (comarooms, "coma");

		log("Got %d rooms", comarooms.size());

		if (comarooms.size() == 0){
			log("No such ComaRoom with id %d! Returning", newVPCommand->roomId);

			return;
		}
		unsigned int i = 0;
		for (; i < comarooms.size(); i++) {
			log("Got coma room with room id: %d", comarooms[i]->roomId);
			if (comarooms[i]->roomId == newVPCommand->roomId) {
				break;
			}
		}

		FrontierInterface::LocalMapInterfacePrx agg2(getIceServer<
				FrontierInterface::LocalMapInterface> ("map.manager"));
		combined_lgm = agg2->getCombinedGridMap(comarooms[i]->containedPlaceIds);

		for (unsigned int j =0; j < comarooms[i]->containedPlaceIds.size(); j++){
			log("getting room which contains, placeid: %d", comarooms[i]->containedPlaceIds[j]);
		}

		m_templateRoomBloxelMaps[newVPCommand->roomId]
				= new SpatialGridMap::GridMap<GridMapData>(combined_lgm.size*2 + 1,
						combined_lgm.size*2 + 1, m_cellsize, m_minbloxel, 0, 2.0,
						combined_lgm.xCenter, combined_lgm.yCenter, 0,
						m_defaultBloxelCell);

		//convert 2D map to 3D
		CureObstMap* lgm = new CureObstMap(combined_lgm.size, m_cellsize, '2',
				CureObstMap::MAP1, combined_lgm.xCenter, combined_lgm.yCenter);
		IcetoCureLGM(combined_lgm, lgm);
		m_templateRoomGridMaps[newVPCommand->roomId] = lgm;
		GDMakeObstacle makeobstacle;
		for (int x = -combined_lgm.size; x < combined_lgm.size; x++) {
			for (int y = -combined_lgm.size; y < combined_lgm.size; y++) {
				if ((*lgm)(x, y) == '1') {
					m_templateRoomBloxelMaps[newVPCommand->roomId]->boxSubColumnModifier(
							x + combined_lgm.size, y + combined_lgm.size,
							m_LaserPoseR.getZ(), m_minbloxel * 2, makeobstacle);
				}
			}
		}
	}


	// FIXME !!! This check should be done for all objects that we can find !!!


	bool alreadyGenerated = false;

	if (m_objectBloxelMaps.count(id) > 0) {
		alreadyGenerated = true;
	}
	// if we already have a bloxel map for configuration
	// else create a new one
	if (!alreadyGenerated) {
		//Since we don't have this location currently, first initialize a bloxel map for it from the template room map
		m_objectBloxelMaps[id] = new SpatialGridMap::GridMap<GridMapData>(
				*m_templateRoomBloxelMaps[newVPCommand->roomId]);
	}



	m_currentBloxelMap = m_objectBloxelMaps[id];
	m_currentCureObstMap = m_templateRoomGridMaps[newVPCommand->roomId];

	// now we have our room map let's fill it
	//Query Conceptual to learn the initial pdf values
	log("Querying for %s", id.c_str());
	ConceptualData::ProbabilityDistributions conceptualProbdist = m_queryHandlerServerInterfacePrx->query(id);
	SpatialProbabilities::ProbabilityDistribution probdist = conceptualProbdist[0];


	if(probdist.massFunction.size() == 0){
		log("Got an empty distribution!");
	}
	double pdfmass = probdist.massFunction[0].probability;
	log("Got probability for %s Conceptual %f",id.c_str(),pdfmass);

	//Todo: Somehow get the probability from probDist
	//double pdfmass = 0.5;

	//Todo: Generate viewpoints on the selected room's bloxel map and for a given pdf id
	if (newVPCommand->relation != SpatialData::INROOM && !alreadyGenerated) {
		log("Searching with a support object");
		// indirect search this must be known object
		// Ask for the cloud

		// Construct Request object
		std::vector<std::string> labels;
		vector<Vector3> centers;
		std::vector<FrontierInterface::ObjectRelation> relation;
		if (m_fromBeliefIdtoVisualLabel.count(newVPCommand->supportObject) == 0){
			log("No such visual object! I am not going to do anything!");
			log("The ones we know are:");
			map<string,string>::const_iterator end = m_fromBeliefIdtoVisualLabel.end();
				    for (map<string,string>::const_iterator it = m_fromBeliefIdtoVisualLabel.begin(); it != end; ++it)
				    {
				        log("beliefid: %s", it->first.c_str());
				        log("visualid: %s", it->second.c_str());
				    }
			return;
		}

		labels.push_back(newVPCommand->searchedObjectCategory);
		labels.push_back(m_fromBeliefIdtoVisualLabel[newVPCommand->supportObject]);


		for (unsigned int j=0; j < labels.size(); j++){
			log("ObjectPriorRequest for: %s", labels[j].c_str());
		}
		relation.push_back(
				(newVPCommand->relation == SpatialData::INOBJECT ? FrontierInterface::IN
						: FrontierInterface::ON));

		FrontierInterface::WeightedPointCloudPtr queryCloud =
				new FrontierInterface::WeightedPointCloud;
		FrontierInterface::ObjectPriorRequestPtr objreq =
				new FrontierInterface::ObjectPriorRequest;
		objreq->relationTypes = relation; // ON or IN or whatnot
		objreq->objects = labels; // Names of objects, starting with the query object
		objreq->cellSize = m_cellsize; // CelGOTONODEll size of map (affects spacing of samples)
		objreq->outCloud = queryCloud; // Data struct to receive output
		objreq->totalMass = 1.0; // we will normalize anyways
		//wait until we get the cloud back
		{
			log("Asking for ObjectPriorRequest");
			m_gotPC = false;
			addToWorkingMemory(newDataID(), objreq);
			log("ObjectPriorRequest added to WM");
			while (!m_gotPC)
				usleep(2500);
			log("got PC for direct search");
		}

		if (m_cloud->isBaseObjectKnown) {
			log("Got distribution around known object pose");
			m_sampler.kernelDensityEstimation3D(*m_objectBloxelMaps[id],
					centers, m_cloud->interval, m_cloud->xExtent,
					m_cloud->yExtent, m_cloud->zExtent, m_cloud->values, 1.0,
					pdfmass, m_templateRoomGridMaps[newVPCommand->roomId]);
			normalizePDF(*m_objectBloxelMaps[id], pdfmass);
		}

	} else if (newVPCommand->relation == SpatialData::INROOM && !alreadyGenerated) {
		log("Searching in the room, assuming uniform probability");
		// uniform over the room
		GDProbSet resetter(0.0);
		m_objectBloxelMaps[id]->universalQuery(resetter, true);
		double fixedpdfvalue = pdfmass / (m_objectBloxelMaps[id]->getZBounds().second - m_objectBloxelMaps[id]->getZBounds().first);

		GDProbInit initfunctor(fixedpdfvalue);
		log("Setting each bloxel near an obstacle to a fixed value of %f, in total: %f", fixedpdfvalue, pdfmass);

		CureObstMap* lgm = m_templateRoomGridMaps[newVPCommand->roomId];
		for (int x = -lgm->getSize(); x <= lgm->getSize(); x++) {
			for (int y = -lgm->getSize(); y <= lgm->getSize(); y++) {
				int bloxelX = x + lgm->getSize();
				int bloxelY = y + lgm->getSize();
				if ((*lgm)(x, y) == '3' || (*lgm)(x, y) == '1') {
					// For each "high" obstacle cell, assign a uniform probability density to its immediate neighbors
					// (Only neighbors which are not unknown in the Cure map. Unknown 3D
					// space is still assigned, though)

					for (int i = -1; i <= 1; i++) {
						for (int j = -1; j <= 1; j++) {
							if ((*lgm)(x + i, y + j) == '0'
									&& (bloxelX + i
											<= m_objectBloxelMaps[id]->getMapSize().first
											&& bloxelX + i > 0)
									&& (bloxelY + i
											<= m_objectBloxelMaps[id]->getMapSize().second
											&& bloxelY + i > 0)) {
//								/log("modifying bloxelmap pdf");
								m_objectBloxelMaps[id]->boxSubColumnModifier(
										bloxelX + i, bloxelY + j, m_mapceiling
												/ 2, m_mapceiling, initfunctor);
							}
						}
					}
				}
			}
		}

		double massAfterInit = initfunctor.getTotal();
			//    double normalizeTo = (initfunctor.getTotal()*m_pout)/(1 - m_pout);
		normalizePDF(*m_objectBloxelMaps[id], pdfmass, massAfterInit);
		log("Done setting.");
	}

	//Now that we've got our map generate cones for this
	//Todo: and generateViewCones based on this
    if(m_usePeekabot){
    	log("Displaying Map in PB.");
    	pbVis->DisplayMap(*m_objectBloxelMaps[id]);
      pbVis->AddPDF(*m_objectBloxelMaps[id]);

    }
	log("getting cones..");
	ViewPointGenerator coneGenerator(this,
			m_templateRoomGridMaps[newVPCommand->roomId],
			m_objectBloxelMaps[id], m_samplesize, m_sampleawayfromobs,
			m_conedepth, m_horizangle, m_vertangle, m_minDistance, pdfmass,
			0.7, 0, 0);
	vector<ViewPointGenerator::SensingAction> viewcones =
			coneGenerator.getBest3DViewCones();

	log("got %d cones..", viewcones.size());

	//Getting the place belief pointer


	// 1. Create conegroups out of viewcones
	// 2. Fill in relevant fields
	// 3. Create beliefs about them

	for (unsigned int i=0; i < viewcones.size(); i++){
		if(m_usePeekabot){
			PostViewCone(viewcones[i]);
		}


		/* GETTING PLACE BELIEFS */

		int closestnode = GetClosestNodeId(viewcones[i].pos[0], viewcones[i].pos[1], viewcones[i].pos[2]);
		int conePlaceId = GetPlaceIdFromNodeId(closestnode);

		cast::cdl::WorkingMemoryAddress placeWMaddress;
		// Get the place id for this cone
		vector< boost::shared_ptr< cast::CASTData<GroundedBelief> > > placeBeliefs;
			getWorkingMemoryEntries<GroundedBelief> ("spatial.sa", 0, placeBeliefs);

			if (placeBeliefs.size() ==0){
				log("Could not get any  GroundedBeliefs from spatial.sa returning without doing anything...");
				return;
			}
			else{
				log("Got %d GroundedBeliefs from spatial.sa", placeBeliefs.size());
			}

			for(unsigned int j=0; j < placeBeliefs.size(); j++){
				if (placeBeliefs[j]->getData()->type != "PlgenerateViewace"){
					log("Not a place belief, but a %s belief", placeBeliefs[j]->getData()->type.c_str());
					continue;
				}
			CondIndependentDistribsPtr dist(CondIndependentDistribsPtr::dynamicCast(placeBeliefs[j]->getData()->content));
			BasicProbDistributionPtr  basicdist(BasicProbDistributionPtr::dynamicCast(dist->distribs["PlaceId"]));
			FormulaValuesPtr formulaValues(FormulaValuesPtr::dynamicCast(basicdist->values));

			IntegerFormulaPtr intformula(IntegerFormulaPtr::dynamicCast(formulaValues->values[0].val));
			int placeid = intformula->val;
			log("Place Id for this belief: %d", placeid);
			if (conePlaceId == placeid){
				log("Got  place belief from roomid: %d", conePlaceId);
				placeWMaddress.id = placeBeliefs[j]->getID();
				placeWMaddress.subarchitecture = "spatial";
				break;
			}
		}




		// FIXME ASSUMPTION: One cone per conegroup
		ConeGroup c;
		c.searchedObjectCategory = newVPCommand->searchedObjectCategory;
		c.bloxelMapId = id;
		c.viewcones.push_back(viewcones[i]);
		c.roomId = newVPCommand->roomId;
		c.placeId = conePlaceId;
		c.relation = newVPCommand->relation;

		if (newVPCommand->relation == SpatialData::INROOM){
			c.supportObjectId = "";
			c.supportObjectCategory = "";
			c.searchedObjectCategory = newVPCommand->searchedObjectCategory;
		}
		else {
			c.relation = (newVPCommand->relation == SpatialData::INOBJECT ? SpatialData::INOBJECT
					: SpatialData::ON);
					c.supportObjectId = newVPCommand->supportObject;
					//FIXME: Get category from object ID
					c.supportObjectCategory = "";
					c.searchedObjectCategory = newVPCommand->searchedObjectCategory;
		}


		/* GETTING COMAROOM BELIEFS */
		log("Looking for Coma room beliefs...");
		 cast::cdl::WorkingMemoryAddress WMaddress;
		if (newVPCommand->relation == SpatialData::INROOM){
			//get the roomid belief WMaddress
			vector< boost::shared_ptr< cast::CASTData<GroundedBelief> > > comaRoomBeliefs;
			getWorkingMemoryEntries<GroundedBelief> ("coma", 0, comaRoomBeliefs);

			if (comaRoomBeliefs.size() ==0){
				log("Could not get any grounded beliefs returning without doing anything...");
				return;
			}
			else{
				log("Go %d Grounded beliefs", comaRoomBeliefs.size());
			}

			for(unsigned int j =0; j < comaRoomBeliefs.size(); j++){
				if (comaRoomBeliefs[j]->getData()->type != "ComaRoom"){
									log("Not a place belief, but a %s belief", comaRoomBeliefs[j]->getData()->type.c_str());
									continue;
					}
			CondIndependentDistribsPtr dist(CondIndependentDistribsPtr::dynamicCast(comaRoomBeliefs[j]->getData()->content));
			BasicProbDistributionPtr  basicdist(BasicProbDistributionPtr::dynamicCast(dist->distribs["RoomId"]));
			FormulaValuesPtr formulaValues(FormulaValuesPtr::dynamicCast(basicdist->values));

			IntegerFormulaPtr intformula(IntegerFormulaPtr::dynamicCast(formulaValues->values[0].val));
			int roomid = intformula->val;
			log("ComaRoom Id for this belief: %d", roomid);
			if (newVPCommand->roomId == roomid){
				log("Got room belief from roomid: %d", newVPCommand->roomId);
				WMaddress.id = comaRoomBeliefs[j]->getID();
				WMaddress.subarchitecture = "coma";
				break;
			}
		}
		}
		else if (newVPCommand->relation == SpatialData::INOBJECT || newVPCommand->relation == SpatialData::ON) {
			//else get the visualobject belief pointer instead
			log("Getting VisualObject beliefs");
			vector< boost::shared_ptr< cast::CASTData<GroundedBelief> > > visualObjectBeliefs;
						getWorkingMemoryEntries<GroundedBelief> ("binder", 0, visualObjectBeliefs);

						if (visualObjectBeliefs.size() ==0){
							log("Could not get any visual objects returning without doing anything...");
							return;
						}
						else{
							log("Got %d Grounded beliefs", visualObjectBeliefs.size());
						}

						for(unsigned int j=0; j < visualObjectBeliefs.size(); j++){
							if (visualObjectBeliefs[j]->getData()->type != "VisualObject"){
												log("Not a visual object belief, but a %s belief", visualObjectBeliefs[j]->getData()->type.c_str());
												continue;
								}
						CondIndependentDistribsPtr dist(CondIndependentDistribsPtr::dynamicCast(visualObjectBeliefs[j]->getData()->content));
						BasicProbDistributionPtr  basicdist(BasicProbDistributionPtr::dynamicCast(dist->distribs["label"]));
						FormulaValuesPtr formulaValues(FormulaValuesPtr::dynamicCast(basicdist->values));

						ElementaryFormulaPtr elformula(ElementaryFormulaPtr::dynamicCast(formulaValues->values[0].val));
						string objectId = elformula->prop;
						log("ObjectId for this belief: %s", objectId.c_str());
						if (newVPCommand->supportObject == objectId){
							log("Got the right object belief: %s", newVPCommand->supportObject.c_str());
							WMaddress.id = visualObjectBeliefs[j]->getID();
							WMaddress.subarchitecture = "binder";
							break;
						}
					}
		}


		log("Creating ConeGroup belief");
		m_coneGroupId++;
		m_beliefConeGroups[m_coneGroupId] = c;
		eu::cogx::beliefs::slice::GroundedBeliefPtr b = new eu::cogx::beliefs::slice::GroundedBelief;
		epstatus::PrivateEpistemicStatusPtr beliefEpStatus= new epstatus::PrivateEpistemicStatus;
		beliefEpStatus->agent = "self";

		b->estatus = beliefEpStatus;
		b->type = "conegroup";
		b->id = newDataID();
		CondIndependentDistribsPtr CondIndProbDist = new  CondIndependentDistribs;

		BasicProbDistributionPtr coneGroupIDProbDist = new BasicProbDistribution;
		BasicProbDistributionPtr searchedObjectLabelProbDist = new BasicProbDistribution;
		BasicProbDistributionPtr relationLabelProbDist = new BasicProbDistribution;
		BasicProbDistributionPtr supportObjectLabelProbDist = new BasicProbDistribution;
		BasicProbDistributionPtr coneProbabilityProbDist = new BasicProbDistribution;
		BasicProbDistributionPtr placeFromWhichObjectCanBeSeenProbDist = new BasicProbDistribution;
		BasicProbDistributionPtr isVisitedProbDist = new BasicProbDistribution;

		FormulaProbPairs pairs;
		FormulaProbPair searchedObjectFormulaPair, coneGroupIDLabelFormulaPair,
		relationLabelFormulaPair,supportObjectLabelFormulaPair, coneProbabilityFormulaPair,
		placeFromWhichObjectCanBeSeenFormulaPair, isVisitedFormulaPair ;

		IntegerFormulaPtr coneGroupIDLabelFormula = new IntegerFormula;
		ElementaryFormulaPtr searchedObjectLabelFormula = new ElementaryFormula;
		ElementaryFormulaPtr relationLabelFormula = new ElementaryFormula;
		PointerFormulaPtr supportObjectLabelFormula = new PointerFormula;
		PointerFormulaPtr placeFromWhichObjectCanBeSeenFormula = new PointerFormula;
		FloatFormulaPtr coneProbabilityFormula = new FloatFormula;
		BooleanFormulaPtr isVisitedFormula = new BooleanFormula;

		coneGroupIDLabelFormula->val = m_coneGroupId;
		searchedObjectLabelFormula->prop =c.searchedObjectCategory;
		relationLabelFormula->prop = (newVPCommand->relation == SpatialData::ON ? "on" : "in");
		supportObjectLabelFormula->pointer =  WMaddress; //c.supportObjectId; // this should be a pointer ideally
		placeFromWhichObjectCanBeSeenFormula->pointer = placeWMaddress;
		coneProbabilityFormula->val = c.getTotalProb();
		isVisitedFormula->val = false;

		searchedObjectFormulaPair.val = searchedObjectLabelFormula;
		searchedObjectFormulaPair.prob = 1;

		coneGroupIDLabelFormulaPair.val = coneGroupIDLabelFormula;
		coneGroupIDLabelFormulaPair.prob = 1;

		relationLabelFormulaPair.val = relationLabelFormula;
		relationLabelFormulaPair.prob = 1;

		supportObjectLabelFormulaPair.val = supportObjectLabelFormula;
		supportObjectLabelFormulaPair.prob = 1;

		coneProbabilityFormulaPair.val = coneProbabilityFormula;
		coneProbabilityFormulaPair.prob = 1;

		placeFromWhichObjectCanBeSeenFormulaPair.val = placeFromWhichObjectCanBeSeenFormula;
		placeFromWhichObjectCanBeSeenFormulaPair.prob = 1;

		isVisitedFormulaPair.val = isVisitedFormula;
		isVisitedFormulaPair.prob=1;

		pairs.push_back(coneGroupIDLabelFormulaPair);
		FormulaValuesPtr formulaValues1 = new FormulaValues;
		formulaValues1->values = pairs;
		coneGroupIDProbDist->values = formulaValues1;
		pairs.clear();

		pairs.push_back(searchedObjectFormulaPair);
		FormulaValuesPtr formulaValues2 = new FormulaValues;
		formulaValues2->values = pairs;
		searchedObjectLabelProbDist->values = formulaValues2;
		pairs.clear();

		pairs.push_back(coneProbabilityFormulaPair);
		FormulaValuesPtr formulaValues3 = new FormulaValues;
		formulaValues3->values = pairs;
		coneProbabilityProbDist->values = formulaValues3;
		pairs.clear();


		pairs.push_back(relationLabelFormulaPair);
		FormulaValuesPtr formulaValues4 = new FormulaValues;
		formulaValues4->values = pairs;
		relationLabelProbDist->values = formulaValues4;
		pairs.clear();

		pairs.push_back(supportObjectLabelFormulaPair);
		FormulaValuesPtr formulaValues5 = new FormulaValues;
		formulaValues5->values = pairs;
		supportObjectLabelProbDist->values = formulaValues5;
		pairs.clear();

		pairs.push_back(placeFromWhichObjectCanBeSeenFormulaPair);
		FormulaValuesPtr formulaValues6 = new FormulaValues;
		formulaValues6->values = pairs;
		placeFromWhichObjectCanBeSeenProbDist->values = formulaValues6;
		pairs.clear();

		pairs.push_back(isVisitedFormulaPair);
		FormulaValuesPtr formulaValues7 = new FormulaValues;
		formulaValues7->values = pairs;
		isVisitedProbDist->values = formulaValues7;
		pairs.clear();

		coneGroupIDProbDist->key = "id";
		CondIndProbDist->distribs["id"] = coneGroupIDProbDist;
		searchedObjectLabelProbDist->key = "cg-label";
		CondIndProbDist->distribs["cg-label"] = searchedObjectLabelProbDist;
		relationLabelProbDist->key = "cg-relation";
		CondIndProbDist->distribs["cg-relation"] = relationLabelProbDist;
		supportObjectLabelProbDist->key = "cg-related-to";
		CondIndProbDist->distribs["cg-related-to"] = supportObjectLabelProbDist;
		coneProbabilityProbDist->key = "p-visible";
		CondIndProbDist->distribs["p-visible"] = coneProbabilityProbDist;
		placeFromWhichObjectCanBeSeenProbDist->key = "cg-place";
		CondIndProbDist->distribs["cg-place"] = placeFromWhichObjectCanBeSeenProbDist;

		isVisitedProbDist->key = "is-visited";
		CondIndProbDist->distribs["is-visited"] = isVisitedProbDist;

		b->content = CondIndProbDist;
		log("writing belief to WM..");
		addToWorkingMemory(b->id, "binder", b);
		log("wrote belief to WM..");
	}

if (WMAddress != ""){
    newVPCommand->status = SpatialData::SUCCESS;
    log("Overwriting command to change status to: SUCCESS");
    overwriteWorkingMemory<SpatialData::RelationalViewPointGenerationCommand>(WMAddress , newVPCommand);
}

}

void AVS_ContinualPlanner::IcetoCureLGM(FrontierInterface::LocalGridMap icemap,
		CureObstMap* lgm) {
	log(
			"icemap.size: %d, icemap.data.size %d, icemap.cellSize: %f, centerx,centery: %f,%f",
			icemap.size, icemap.data.size(), icemap.cellSize, icemap.xCenter,
			icemap.yCenter);
	int lp = 0;
	for (int x = -icemap.size; x <= icemap.size; x++) {
		for (int y = -icemap.size; y <= icemap.size; y++) {
			(*lgm)(x, y) = (icemap.data[lp]);
			lp++;
		}
	}
	log("converted icemap to Cure::LocalGridMap");
}

/* Process ConeGroup with id */
void AVS_ContinualPlanner::processConeGroup(int id) {
	// Todo: Loop over cones in conegroup
	// FIXME ASSUMPTION ConeGroup contains one cone
	// FIXME ASSUMPTION Always observe one object at a time

	// Todo: Once we get a response from vision, calculate the remaining prob. value
	// Get ConeGroup
	log("Processing Cone Group");
	if (m_beliefConeGroups.count(id) == 0 ){
		log("No Cone Groups with id: %d, this is an indication of id mismatch!", id);
		log("We have %d cone groups", m_beliefConeGroups.size());

		MapConeType::const_iterator end = m_beliefConeGroups.end();
	    for (MapConeType::const_iterator it = m_beliefConeGroups.begin(); it != end; ++it)
	    {
	        log("key: %d", it->first);
	    }
		return;
	}

	m_currentConeGroup = m_beliefConeGroups[id];
	m_currentViewCone = m_currentConeGroup.viewcones[0];

			// FIXME post a nav command
	log("Posting a nav command");

	Cure::Pose3D pos;
	pos.setX(m_currentViewCone.pos[0]);
	pos.setY(m_currentViewCone.pos[1]);
	pos.setTheta(m_currentViewCone.pan);
	PostNavCommand(pos, SpatialData::GOTOPOSITION);
}


void AVS_ContinualPlanner::ViewConeUpdate(ViewPointGenerator::SensingAction viewcone, BloxelMap* map){

double sensingProb = 0.8;
GDProbSum sumcells;
GDIsObstacle isobstacle;
/* DEBUG */
GDProbSum conesum;
map->coneQuery(viewcone.pos[0],viewcone.pos[1],
    viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, conesum, conesum, m_minDistance);
	log("ViewConeUpdate: Cone sums to %f",conesum.getResult());

/* DEBUG */


map->universalQuery(sumcells);
double initialMapPDFSum = sumcells.getResult();
log("ViewConeUpdate: Whole map PDF sums to: %f", initialMapPDFSum);
// then deal with those bloxels that belongs to this cone

GDProbScale scalefunctor(1-sensingProb);
map->coneModifier(viewcone.pos[0],viewcone.pos[1],
    viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, scalefunctor,scalefunctor, m_minDistance);
sumcells.reset();
map->universalQuery(sumcells);

double finalMapPDFSum = sumcells.getResult();
double differenceMapPDFSum = initialMapPDFSum - finalMapPDFSum;

if (differenceMapPDFSum < 0){
	log("We managed to observe negative probability, something is very wrong!");
}

log("ViewConeUpdate: After cone update map sums to: %f", sumcells.getResult());


if(m_usePeekabot){
	  pbVis->AddPDF(*map);
	}

SpatialData::ObjectSearchResultPtr result = new SpatialData::ObjectSearchResult;
result->searchedObjectCategory = m_currentConeGroup.searchedObjectCategory;
	result->relation = m_currentConeGroup.relation;
	result->supportObjectCategory = m_currentConeGroup.supportObjectCategory;
	result->supportObjectId = m_currentConeGroup.supportObjectId;
	result->beta =differenceMapPDFSum/initialMapPDFSum;

	log("The observed ratio of location: %f", result->beta);

	log("Publishing ObjectSearchResult with: category: %s, relation: %s, supportObjectCategory: %s, supportObjectId: %s",
  			result->searchedObjectCategory.c_str(), relationToString(result->relation).c_str(), result->supportObjectCategory.c_str(), result->supportObjectId.c_str());

  	addToWorkingMemory(newDataID(), result);
}



void AVS_ContinualPlanner::Recognize(){
	// we always ask for all the objects to be recognized
	for (unsigned int i=0; i< m_siftObjects.size(); i++){
		//todo: ask for visual object recognition
	}
	for (unsigned int i=0; i<m_ARtaggedObjects.size(); i++){
		//todo: ask for tagged objects
	}
}


std::string AVS_ContinualPlanner::convertLocation2Id(
		SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand) {
	string id = "";
	string relationstring =
			(newVPCommand->relation == SpatialData::INOBJECT) ? "INOBJECT"
					: "INROOM";
	if (newVPCommand->supportObject != "")
		id = newVPCommand->searchedObjectCategory + relationstring
				+ newVPCommand->supportObject + boost::lexical_cast<string>(
				newVPCommand->roomId);
	else
		id = newVPCommand->searchedObjectCategory + relationstring
				+ newVPCommand->supportObject + boost::lexical_cast<string>(
				newVPCommand->roomId);
	return id;
}



void AVS_ContinualPlanner::configure(
		const std::map<std::string, std::string>& _config) {
	map<string, string>::const_iterator it = _config.find("-c");

	if (it == _config.end()) {
		println("configure(...) Need config file (use -c option)\n");
		std::abort();
	}
	std::string configfile = it->second;

	Cure::ConfigFileReader *cfg;
	if (it != _config.end()) {
		cfg = new Cure::ConfigFileReader;
		log("About to try to open the config file");
		if (cfg->init(it->second) != 0) {
			delete cfg;
			cfg = 0;
			log("Could not init Cure::ConfigFileReader with -c argument");
		} else {
			log("Managed to open the Cure config file");
		}
	}

	if (cfg->getSensorPose(1, m_LaserPoseR)) {
		println("configure(...) Failed to get sensor pose for laser");
		std::abort();
	}

    cfg->getRoboLookHost(m_PbHost);
    std::string usedCfgFile, tmp;
    if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
      m_PbHost = tmp;
    }

	m_gridsize = 200;
	m_cellsize = 0.05;
	it = _config.find("--gridsize");
	if (it != _config.end()) {

		m_gridsize = (atoi(it->second.c_str()));
		log("Gridsize set to: %d", m_gridsize);
	}
	it = _config.find("--cellsize");
	if (it != _config.end()) {
		m_cellsize = (atof(it->second.c_str()));
		log("Cellsize set to: %f", m_cellsize);
	}

	m_samplesize = 100;

	 it = _config.find("--samplesize");
	    if (it != _config.end()) {
	      m_samplesize = (atof(it->second.c_str()));
	      log("Samplesize set to: %d", m_samplesize);
	    }

	m_minbloxel = 0.05;
	it = _config.find("--minbloxel");
	if (it != _config.end()) {
		m_minbloxel = (atof(it->second.c_str()));
		log("Min bloxel height set to: %f", m_minbloxel);
	}

	m_mapceiling = 2.0;
	it = _config.find("--mapceiling");
	if (it != _config.end()) {
		m_mapceiling = (atof(it->second.c_str()));
		log("Map ceiling set to: %d", m_mapceiling);
	}

	it = _config.find("--kernel-width-factor");
	if (it != _config.end()) {
		m_sampler.setKernelWidthFactor(atoi(it->second.c_str()));
	}

	m_horizangle = M_PI / 4;
	it = _config.find("--cam-horizangle");
	if (it != _config.end()) {
		m_horizangle = (atof(it->second.c_str())) * M_PI / 180.0;
		log("Camera FoV horizontal angle set to: %f", m_horizangle);
	}

	m_vertangle = M_PI / 4;
	it = _config.find("--cam-vertangle");
	if (it != _config.end()) {
		m_vertangle = (atof(it->second.c_str())) * M_PI / 180.0;
		log("Camera FoV vertical angle set to: %f", m_vertangle);
	}

	m_conedepth = 2.0;
	it = _config.find("--cam-conedepth");
	if (it != _config.end()) {
		m_conedepth = (atof(it->second.c_str()));
		log("Camera view cone depth set to: %f", m_conedepth);
	}

	m_sampleawayfromobs= 0.2;
	    it = _config.find("--sampleawayfromobs");
	    if (it != _config.end()) {
	      m_sampleawayfromobs= (atof(it->second.c_str()));
	      log("Distance to nearest obs for samples set to: %f", m_sampleawayfromobs);
	    }

	m_minDistance = m_conedepth / 4.0;

	  m_usePeekabot = false;
	    if (_config.find("--usepeekabot") != _config.end())
	      m_usePeekabot= true;

	    if(m_usePeekabot){
	      pbVis = new VisualPB_Bloxel(m_PbHost,5050,m_gridsize,m_gridsize,m_cellsize,1,true);//host,port,xsize,ysize,cellsize,scale, redraw whole map every time
	      pbVis->connectPeekabot();
	    }

	m_usePTZ = false;
	if (_config.find("--ctrl-ptu") != _config.end()) {
		m_usePTZ = true;
		log("will use ptu");
	}

	m_ignoreTilt = false;
		if (_config.find("--ignore-tilt") != _config.end())
			m_ignoreTilt = true;

	// QueryHandler name
	if ((it = _config.find("--queryhandler")) != _config.end()) {
		m_queryHandlerName = it->second;
	}

	if (m_usePTZ) {
			log("connecting to PTU");
			Ice::CommunicatorPtr ic = getCommunicator();

			Ice::Identity id;
			id.name = "PTZServer";
			id.category = "PTZServer";

			std::ostringstream str;
			str << ic->identityToString(id) << ":default" << " -h localhost"
					<< " -p " << cast::cdl::CPPSERVERPORT;

			Ice::ObjectPrx base = ic->stringToProxy(str.str());
			m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);
		}

	if ((it = _config.find("--siftobjects")) != _config.end()) {
		istringstream istr(it->second);
		string label;
		while (istr >> label) {
			m_siftObjects.push_back(label);
		}
	}
	log("Loaded sift objects.");
	for (unsigned int i = 0; i < m_siftObjects.size(); i++)
		log("%s , ", m_siftObjects[i].c_str());


	if ((it = _config.find("--ARtagobjects")) != _config.end()) {
			istringstream istr(it->second);
			string label;
			while (istr >> label) {
				m_ARtaggedObjects.push_back(label);
			}
		}
		log("Loaded tagged objects.");
		for (unsigned int i = 0; i < m_ARtaggedObjects.size(); i++)
			log("%s , ", m_ARtaggedObjects[i].c_str());

		m_allObjects.reserve((m_siftObjects.size() + m_ARtaggedObjects.size()));
		m_allObjects.insert(m_allObjects.end(), m_siftObjects.begin(), m_siftObjects.end());
		m_allObjects.insert(m_allObjects.end(), m_ARtaggedObjects.begin(), m_ARtaggedObjects.end());

		m_currentProcessConeGroup = new SpatialData::ProcessConeGroup;
		m_coneGroupId = 0;

		  SpatialData::AVSInterfacePtr servant = new AVSServer(this);
		  registerIceServer<SpatialData::AVSInterface, SpatialData::AVSInterface>(servant);


}

void AVS_ContinualPlanner::AVSServer::simulateViewCones(const SpatialData::RelationalViewPointGenerationCommandPtr &cmd ,
	    const Ice::Current &){
	SpatialData::RelationalViewPointGenerationCommandPtr cmd2 = new SpatialData::RelationalViewPointGenerationCommand;
	cmd2->searchedObjectCategory = cmd->searchedObjectCategory;
	cmd2->relation = cmd->relation;
	cmd2->supportObject = cmd->supportObject;
	cmd2->roomId = cmd->roomId;
	cmd2->supportObjectCategory = cmd->supportObjectCategory;

	m_pOwner->generateViewCones(cmd2, "");
}


void AVS_ContinualPlanner::newRobotPose(const cdl::WorkingMemoryChange &objID) {
	try {
		lastRobotPose = getMemoryEntry<NavData::RobotPose2d> (objID.address);

	} catch (DoesNotExistOnWMException e) {
		log("Error! robotPose missing on WM!");
		return;
	}

}

AVS_ContinualPlanner::NavCommandReceiver::NavCommandReceiver(
		AVS_ContinualPlanner & _component, SpatialData::NavCommandPtr _cmd) :
	m_component(_component), m_cmd(_cmd) {
	m_component.log("received NavCommandReceiver notification");
	string id(m_component.newDataID());
	m_component.log("ID post: %s", id.c_str());

	m_component.addChangeFilter(createIDFilter(id, cdl::OVERWRITE), this);
	m_component.addToWorkingMemory<SpatialData::NavCommand> (id, m_cmd);

}

void AVS_ContinualPlanner::NavCommandReceiver::workingMemoryChanged(
		const cast::cdl::WorkingMemoryChange &objID) {
	m_component.log("received inner notification");
	try {
		m_component.owtNavCommand(objID);
	} catch (const CASTException &e) {
		//      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
	}

}

void AVS_ContinualPlanner::owtNavCommand(
		const cast::cdl::WorkingMemoryChange &objID) {
	SpatialData::NavCommandPtr cmd(getMemoryEntry<SpatialData::NavCommand> (objID.address));
	try {
	if (cmd->comp == SpatialData::COMMANDSUCCEEDED) {
		// it means we've reached viewcone position
	//	MovePanTilt(0.0, m_currentViewCone.tilt, 0.08);
	//	Recognize();
		log("Nav Command succeeded");
		ViewConeUpdate(m_currentViewCone, m_objectBloxelMaps[m_currentConeGroup.bloxelMapId]);
		m_currentProcessConeGroup->status = SpatialData::SUCCESS;
		log("Overwriting command to change status to: SUCCESS");
		overwriteWorkingMemory<SpatialData::ProcessConeGroup>(m_processConeGroupCommandWMAddress , m_currentProcessConeGroup);
	}
	if (cmd->comp == SpatialData::COMMANDFAILED) {
				// it means we've failed to reach the viewcone position
			}
	} catch (const CASTException &e) {
			//      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
		}
	}

void AVS_ContinualPlanner::PostNavCommand(Cure::Pose3D position,
		SpatialData::CommandType cmdtype) {
	SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
	cmd->prio = SpatialData::URGENT;
	cmd->cmd = cmdtype;
	cmd->pose.resize(3);
	cmd->pose[0] = position.getX();
	cmd->pose[1] = position.getY();
	cmd->pose[2] = position.getTheta();
	cmd->tolerance.resize(1);
	cmd->tolerance[0] = 0.1;
	cmd->status = SpatialData::NONE;
	cmd->comp = SpatialData::COMMANDPENDING;
	new NavCommandReceiver(*this, cmd);
	log("posted nav command");
}

void AVS_ContinualPlanner::addRecognizer3DCommand(
		VisionData::Recognizer3DCommandType cmd, std::string label,
		std::string visualObjectID) {
	log("posting recognizer command: %s.", label.c_str());
	VisionData::Recognizer3DCommandPtr rec_cmd =
			new VisionData::Recognizer3DCommand;
	rec_cmd->cmd = cmd;
	rec_cmd->label = label;
	rec_cmd->visualObjectID = visualObjectID;
	log("constructed visual command, adding to WM");
	addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
	log("added to WM");
}


void AVS_ContinualPlanner::MovePanTilt(double pan, double tilt, double tolerance) {

	if (m_usePTZ) {
		log(" Moving pantilt to: %f %f with %f tolerance", pan, tilt, tolerance);
		ptz::PTZPose p;
		ptz::PTZReading ptuPose;
		p.pan = pan;
		p.tilt = tilt;

		p.zoom = 0;
		m_ptzInterface->setPose(p);
		bool run = true;
		ptuPose = m_ptzInterface->getPose();
		double actualpan = ptuPose.pose.pan;
		double actualtilt = ptuPose.pose.tilt;

		while (run) {
			m_ptzInterface->setPose(p);
			ptuPose = m_ptzInterface->getPose();
			actualpan = ptuPose.pose.pan;
			actualtilt = ptuPose.pose.tilt;

			log("desired pan tilt is: %f %f", pan, tilt);
			log("actual pan tilt is: %f %f", actualpan, actualtilt);
			log("tolerance is: %f", tolerance);

			//check that pan falls in tolerance range
			if (actualpan < (pan + tolerance) && actualpan > (pan - tolerance)) {
				run = false;
			}

			//only check tilt if pan is ok

			if (!run) {
				if (m_ignoreTilt) {
					run = false;
				} else {
					if (actualtilt < (tilt + tolerance) && actualtilt > (tilt
							- tolerance)) {
						run = false;
					} else {
						//if the previous check fails, loop again
						run = true;
					}
				}
			}

			usleep(100000);
		}
		log("Moved.");
		sleep(1);
	}
}

void AVS_ContinualPlanner::addARTagCommand(){
	//todo: add new AR tag command
}
string AVS_ContinualPlanner::relationToString(SpatialData::SpatialRelation rel){
	if(rel == SpatialData::INROOM)
		return "inroom";
	else if (rel == SpatialData::INOBJECT)
		return "inobject";
	else if (rel == SpatialData::ON)
		return "on";
	else
		return "This is not a relation, something bad happened";
}


int AVS_ContinualPlanner::GetPlaceIdFromNodeId(int nodeId){
  FrontierInterface::PlaceInterfacePrx agg(getIceServer<FrontierInterface::PlaceInterface>("place.manager"));
  SpatialData::PlacePtr p = new SpatialData::Place;
   p = agg->getPlaceFromNodeID(nodeId);
   if (p != NULL)
     return p->id;
   else
     return -1;
}
int AVS_ContinualPlanner::GetClosestNodeId(double x, double y, double a){

  NavData::NavGraphInterfacePrx agg(getIceServer<NavData::NavGraphInterface>("navgraph.process"));
  int d = agg->getClosestNodeId(x,y,a,3);
  return d;
}

void AVS_ContinualPlanner::PostViewCone(const ViewPointGenerator::SensingAction &nbv)
{
/* Add plan to PB BEGIN */
NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
NavData::ViewPoint viewpoint;
viewpoint.pos.x = nbv.pos[0];
viewpoint.pos.y = nbv.pos[1];
viewpoint.pos.z = nbv.pos[2];
viewpoint.length = m_conedepth;
viewpoint.pan = nbv.pan;
viewpoint.tilt = nbv.tilt;
obs->planlist.push_back(viewpoint);
addToWorkingMemory(newDataID(), obs);
/* Add plan to PB END */

//cout << "selected cone " << viewpoint.pos.x << " " << viewpoint.pos.y << " " <<viewpoint.pos.z << " " << viewpoint.pan << " " << viewpoint.tilt << endl;
}



} //namespace
