

#include <VisionData.hpp>
#include "PoseCv.hh"


bool convertPoseCv2MathPose(P::PoseCv& cvPose, cogx::Math::Pose3& mPose){

	mPose.rot.m00 = cvmGet(cvPose.R,0,0); mPose.rot.m01 = cvmGet(cvPose.R,0,1); mPose.rot.m02 = cvmGet(cvPose.R,0,2);
	mPose.rot.m10 = cvmGet(cvPose.R,1,0); mPose.rot.m11 = cvmGet(cvPose.R,1,1); mPose.rot.m12 = cvmGet(cvPose.R,1,2);
	mPose.rot.m20 = cvmGet(cvPose.R,2,0); mPose.rot.m21 = cvmGet(cvPose.R,2,1); mPose.rot.m22 = cvmGet(cvPose.R,2,2);

	
	mPose.pos.x = cvmGet(cvPose.t,0,0);
	mPose.pos.y = cvmGet(cvPose.t,1,0);
	mPose.pos.z = cvmGet(cvPose.t,2,0);
	
}