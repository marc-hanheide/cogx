/** @file PushingApplication.cpp
 * 
 *
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2009      Sergio Roa
 *
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI
 * @author	Manuel Noll (DFKI)

   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.
 
*/

#include <scenario/PushingApplication.h>

namespace smlearning {


bool XMLData(Scenario::Desc &val, XMLContext* xmlcontext, Context *context) {
	if (xmlcontext == NULL) {
		ASSERT(false)
		return false;
	}
    
	std::string driver;
	XMLData("driver", driver, xmlcontext->getContextFirst("arm")); // Get arm driver name
	val.descArmActor.armDesc.pArmDesc = Arm::Desc::load(*context, driver); // Load driver
	
	// finger setup
	val.descArmActor.fingerDesc.clear();
	golem::Real baseLength = 0.1848;
	XMLData(baseLength, xmlcontext->getContextFirst("effector base_length"));
	golem::Real fingerLength = 0.135;
	XMLData(fingerLength, xmlcontext->getContextFirst("effector finger_length"));
	golem::Real fingerDiam = 0.02;
	XMLData(fingerDiam, xmlcontext->getContextFirst("effector finger_diameter"));
	golem::Real tipRadius = 0.015;
	XMLData(tipRadius, xmlcontext->getContextFirst("effector tip_radius"));
	
	golem::BoundingBox::Desc* pFingerRodShapeDesc = new golem::BoundingBox::Desc;
	pFingerRodShapeDesc->dimensions.set(fingerDiam/2.0, fingerLength/2.0, fingerDiam/2.0);
	pFingerRodShapeDesc->pose.p.v2 += baseLength + fingerLength/2.0;
	pFingerRodShapeDesc->group = val.descArmActor.effectorGroup;
	val.descArmActor.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerRodShapeDesc));
	golem::BoundingSphere::Desc* pFingerTipShapeDesc = new golem::BoundingSphere::Desc;
	pFingerTipShapeDesc->radius = tipRadius;
	pFingerTipShapeDesc->pose.p.v2 += golem::Real(baseLength + fingerLength);
	pFingerTipShapeDesc->group = val.descArmActor.effectorGroup;
	val.descArmActor.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerTipShapeDesc));
	
	// end-effector reference pose
	val.descArmActor.referencePose.setId();
	val.descArmActor.referencePose.p.v2 += golem::Real(baseLength + fingerLength);



	
	//polyflap interaction settings

	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	//maximum X value for polyflap position
	XMLData(val.featLimits.maxX, xmlcontext->getContextFirst("ObjectInteraction maxX"));
	//maximum Y value for polyflap position
	XMLData(val.featLimits.maxY, xmlcontext->getContextFirst("ObjectInteraction maxY"));
	//maximum Z value for polyflap position
	XMLData(val.featLimits.maxZ, xmlcontext->getContextFirst("ObjectInteraction maxZ"));
	//minimum X value for polyflap position
	XMLData(val.featLimits.minX, xmlcontext->getContextFirst("ObjectInteraction minX"));
	//minimum Y value for polyflap position
	XMLData(val.featLimits.minY, xmlcontext->getContextFirst("ObjectInteraction minY"));
	//minimum Z value for polyflap position
	XMLData(val.featLimits.minZ, xmlcontext->getContextFirst("ObjectInteraction minZ"));
	//minimum duration value for pushing action
	XMLData(val.featLimits.minDuration, xmlcontext->getContextFirst("ObjectInteraction minPushDuration"));
	//maximum duration value for pushing action
	XMLData(val.featLimits.maxDuration, xmlcontext->getContextFirst("ObjectInteraction maxPushDuration"));
	//minimum value for a label
	XMLData(val.featLimits.minValLabel, xmlcontext->getContextFirst("ObjectInteraction minValLabel"));
	//maximum value for a label
	XMLData(val.featLimits.maxValLabel, xmlcontext->getContextFirst("ObjectInteraction maxValLabel"));

	//minimal duration of a movement (by normal speed)
	XMLData(val.descArmActor.minDuration, xmlcontext->getContextFirst("ObjectInteraction minDuration"));


	Real x;
	Real y;
	Real z;
	
	//Polyflap Position and orientation
	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction startPosition x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction startPosition y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction startPosition z"));
	val.descActorObject.startPosition.set(x, y, z);

	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction startRotation x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction startRotation y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction startRotation z"));
	val.descActorObject.startRotation.set(y, x, z);

	//Polyflap dimensions		
	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions z"));
	val.descActorObject.dimensions.set(x, y, z);
	//Polyflap width
	XMLData(val.descActorObject.width, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions width"));
	

	//vertical distance from the ground
	//const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	XMLData(val.descActorObject.over, xmlcontext->getContextFirst("ObjectInteraction over"));
	//distance from the front/back of the polyflap
	XMLData(val.descActorObject.dist, xmlcontext->getContextFirst("ObjectInteraction dist"));

	Real r;

	//distance from the side of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction side"));
	val.descActorObject.side = val.descActorObject.dimensions.v1*r;
	//center of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction center"));
	val.descActorObject.center = val.descActorObject.dimensions.v2*r;
	//distance from the top of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction top"));
	val.descActorObject.top = val.descActorObject.dimensions.v2 - r;
	//lenght of the movement		
	XMLData(val.distance, xmlcontext->getContextFirst("ObjectInteraction distance"));
	

	XMLData(val.startingPositionsConfig, xmlcontext->getContextFirst("loop startingPositions"));
	
	
	return true;
}

bool XMLData(PredictingScenario::Desc &val, XMLContext* xmlcontext, Context *context) {
	XMLData ((Scenario::Desc&)val, xmlcontext, context);
}

// bool XMLData(ActiveGNGScenario::Desc &val, XMLContext* xmlcontext, Context *context) {
// 	XMLData ((Scenario::Desc&)val, xmlcontext, context);
// }

bool XMLData(ActiveLearnScenario::Desc &val, XMLContext* xmlcontext, Context *context) {
	XMLData ((Scenario::Desc&)val, xmlcontext, context);
}

bool XMLData(PredictingActiveLearnScenario::Desc &val, XMLContext* xmlcontext, Context *context) {
	XMLData ((Scenario::Desc&)val, xmlcontext, context);
}


}; // namespace smlearning 

