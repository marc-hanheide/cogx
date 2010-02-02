

#include <VisionData.hpp>
#include "Pose.hh"


bool convertRecognizerPose2VisualObjectPose(P::Pose& rPose, cogx::Math::Pose3& mPose){

	mPose.rot.m00 = rPose.R(1,1); mPose.rot.m01 = rPose.R(1,2); mPose.rot.m02 = rPose.R(1,3); 
	mPose.rot.m10 = rPose.R(2,1); mPose.rot.m11 = rPose.R(2,2); mPose.rot.m12 = rPose.R(2,3); 
	mPose.rot.m20 = rPose.R(3,1); mPose.rot.m21 = rPose.R(3,2); mPose.rot.m22 = rPose.R(3,3);
	
	mPose.pos.x = rPose.t(1,1);
	mPose.pos.y = rPose.t(2,1);
	mPose.pos.z = rPose.t(3,1);
	
}