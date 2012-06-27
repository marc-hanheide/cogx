/** @file ConcreteActor.h
 * 
 * 
 * @author	Manuel Noll (DFKI)
 *
 * @version 1.0
 *
 * 2011      Manuel Noll
 
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

#ifndef CONCRETEACTOR_H
#define CONCRETEACTOR_H


#include <Golem/Math/Math.h>
#include <Golem/Phys/Object.h>
#include <Golem/Phys/Scene.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Phys/Msg.h>

// using namespace golem;
// using namespace std;

namespace smlearning {

/** \brief The abstract class for an concrete actor in the golem world.
*
*	The ConcreteActor class sets shape, rotation and position of an object within a given scene.
*	This means that the class represents the concrete description of an actor.
*	In order to model a concrete actor one has to derive a class from ConcreteActor and define
*	the getShape function in that derived one.
*
*	The ConcreteActor class is used as an object for the ActorObject class.
*
*/

class ConcreteActor
{
public:
	/** defines the concrete shape of the actor */
	virtual golem::Actor::Desc* getShape()=0;
	virtual void setShape(golem::Creator &,const golem::Vec3&,const golem::Real &)=0;
	golem::Bounds::SeqPtr getLocalBoundsSeq (golem::U32 group = golem::Bounds::GROUP_ALL) const {
		// CriticalSectionWrapper csw(csBoundsSeq);
		golem::Bounds::SeqPtr pBoundsSeq = golem::Bounds::clone(boundsConstSeq.begin(), boundsConstSeq.end(), group);
		
		return pBoundsSeq;
	}
protected:
	typedef std::map<const NxShapeDesc*, golem::Bounds::Desc::Ptr> BoundsDescMap;
	typedef golem::shared_ptr<NxShapeDesc> NxShapeDescPtr;
	typedef std::map<const golem::Bounds::Desc*, NxShapeDescPtr> NxShapeDescMap;
	struct BoundsData {
		typedef std::map<const golem::Bounds*, BoundsData> Seq;

		golem::Bounds::Ptr pBounds;
		golem::Bounds::Desc::Ptr pBoundsDesc;
		const NxShape* pConstNxShape;
		NxShape* pNxShape;
	};
	virtual void setupBounds ()
	{
		// boundsDataSeq.clear();
		boundsConstSeq.clear();

		for (NxArray<NxShapeDesc*, NxAllocatorDefault>::const_iterator i = shapeDesc->nxActorDesc.shapes.begin(); i != shapeDesc->nxActorDesc.shapes.end(); i++) {
			BoundsData boundsData;

			boundsData.pBoundsDesc = createBoundsDesc(**i); // Access to PhysX (for ConvexMesh)

			if (boundsData.pBoundsDesc == NULL)
				throw golem::MsgActorBoundsDescCreate(golem::Message::LEVEL_CRIT, "Actor::create(): Unable to create bounds description");

			boundsData.pBounds = boundsData.pBoundsDesc->create();
			if (boundsData.pBounds == NULL)
				throw golem::MsgActorBoundsCreate(golem::Message::LEVEL_CRIT, "Actor::create(): Unable to create bounds");

			// boundsDataSeq.insert(std::pair<const Bounds*, BoundsData>(boundsData.pBounds.get(), boundsData));
			boundsConstSeq.push_back(boundsData.pBounds.get());
		}

	}

	golem::Bounds::Desc::Ptr createBoundsDesc(const NxPlaneShapeDesc *pDesc) const {
		ASSERT(pDesc != NULL)
			golem::Bounds::Desc::Ptr pBoundsDesc;
		golem::BoundingPlane::Desc *pBoundingPlaneDesc = new golem::BoundingPlane::Desc;
		pBoundsDesc.reset(pBoundingPlaneDesc);

		//pDesc->localPose.M.getRowMajor(&pBoundingPlaneDesc->pose.R.m11);
		//pDesc->localPose.t.get(&pBoundingPlaneDesc->pose.p.v1);
		// Ignore NxSphereShape local pose (it is not used by Novodex)
		pBoundingPlaneDesc->normal.set(&pDesc->normal.x);
		pBoundingPlaneDesc->distance = (golem::Real)pDesc->d;

		return pBoundsDesc;
	}

	golem::Bounds::Desc::Ptr createBoundsDesc(const NxSphereShapeDesc *pDesc) const {
		ASSERT(pDesc != NULL)
			golem::Bounds::Desc::Ptr pBoundsDesc;
		golem::BoundingSphere::Desc *pBoundingSphereDesc = new golem::BoundingSphere::Desc;
		pBoundsDesc.reset(pBoundingSphereDesc);

		pDesc->localPose.M.getRowMajor(&pBoundingSphereDesc->pose.R.m11);
		pDesc->localPose.t.get(&pBoundingSphereDesc->pose.p.v1);	
		pBoundingSphereDesc->radius = (golem::Real)pDesc->radius;
		
		return pBoundsDesc;
	}

	golem::Bounds::Desc::Ptr createBoundsDesc(const NxBoxShapeDesc *pDesc) const {
		ASSERT(pDesc != NULL)
			golem::Bounds::Desc::Ptr pBoundsDesc;
		golem::BoundingBox::Desc *pBoundingBoxDesc = new golem::BoundingBox::Desc;
		pBoundsDesc.reset(pBoundingBoxDesc);

		pDesc->localPose.M.getRowMajor(&pBoundingBoxDesc->pose.R.m11);
		pDesc->localPose.t.get(&pBoundingBoxDesc->pose.p.v1);
		pDesc->dimensions.get(&pBoundingBoxDesc->dimensions.v1);
		
		return pBoundsDesc;
	}
	
	/** Returns bounds description associated with the Novodex shape description. */
	golem::Bounds::Desc::Ptr createBoundsDesc(const NxShapeDesc &nxShapeDesc) {
		// check if bound description has been already created
		BoundsDescMap::const_iterator pos = boundsDescMap.find(&nxShapeDesc);
		if (pos != boundsDescMap.end())
			return pos->second;
	
		golem::Bounds::Desc::Ptr pBoundsDesc;

		// check if description is valid
		if (!nxShapeDesc.isValid())
			throw golem::MsgSceneNxShapeDescInvalidDesc(golem::Message::LEVEL_CRIT, "Scene::createBoundsDesc(): invalid Novodex shape description");

		NxShapeDescPtr nxShapeDescPtr;

		if (nxShapeDesc.getType() == NX_SHAPE_PLANE) {
			const NxPlaneShapeDesc *pDescSrc = dynamic_cast<const NxPlaneShapeDesc*>(&nxShapeDesc);
			if (pDescSrc != NULL) {
				NxPlaneShapeDesc *pDescDst = new NxPlaneShapeDesc();
				nxShapeDescPtr.reset(pDescDst);
				::memcpy(pDescDst, pDescSrc, sizeof(NxPlaneShapeDesc));
				pBoundsDesc = createBoundsDesc(pDescDst);
			}
		}
		else if (nxShapeDesc.getType() == NX_SHAPE_SPHERE) {
			const NxSphereShapeDesc *pDescSrc = dynamic_cast<const NxSphereShapeDesc*>(&nxShapeDesc);
			if (pDescSrc != NULL) {
				NxSphereShapeDesc *pDescDst = new NxSphereShapeDesc();
				nxShapeDescPtr.reset(pDescDst);
				::memcpy(pDescDst, pDescSrc, sizeof(NxSphereShapeDesc));
				pBoundsDesc = createBoundsDesc(pDescDst);
			}
		}
		else if (nxShapeDesc.getType() == NX_SHAPE_BOX) {
			const NxBoxShapeDesc *pDescSrc = dynamic_cast<const NxBoxShapeDesc*>(&nxShapeDesc);
			if (pDescSrc != NULL) {
				NxBoxShapeDesc *pDescDst = new NxBoxShapeDesc();
				nxShapeDescPtr.reset(pDescDst);
				::memcpy(pDescDst, pDescSrc, sizeof(NxBoxShapeDesc));
				pBoundsDesc = createBoundsDesc(pDescDst);
			}
		}
		// else if (nxShapeDesc.getType() == NX_SHAPE_CONVEX) {
		// 	const NxConvexShapeDesc *pDescSrc = dynamic_cast<const NxConvexShapeDesc*>(&nxShapeDesc);
		// 	if (pDescSrc != NULL) {
		// 		NxConvexShapeDesc *pDescDst = new NxConvexShapeDesc();
		// 		nxShapeDescPtr.reset(pDescDst);
		// 		::memcpy(pDescDst, pDescSrc, sizeof(NxConvexShapeDesc));
		// 		pBoundsDesc = createBoundsDesc(pDescDst); // Access to PhysX
		// 	}
		// }
		else {
			ASSERT(false)
				}
		
		if (pBoundsDesc == NULL)
			throw golem::MsgSceneNxShapeDescCreate(golem::Message::LEVEL_CRIT, "Scene::createBoundsDesc(): failed to create bounds description");

		nxShapeDescMap[pBoundsDesc.get()] = nxShapeDescPtr;
		boundsDescMap[nxShapeDescPtr.get()] = pBoundsDesc;
		
		return pBoundsDesc;
	}
	golem::Actor::Desc* shapeDesc;
	golem::Bounds::ConstSeq boundsConstSeq;
	BoundsDescMap boundsDescMap;
	NxShapeDescMap nxShapeDescMap;

};

}; // namespace smlearning 

#endif
