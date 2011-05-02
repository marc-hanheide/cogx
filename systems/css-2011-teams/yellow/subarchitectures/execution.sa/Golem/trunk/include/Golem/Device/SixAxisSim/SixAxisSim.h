/** @file SixAxisSim.h
 * 
 * 6-axis arm simulator
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_SIXAXISSIM_SIXAXISSIM_H_
#define _GOLEM_DEVICE_SIXAXISSIM_SIXAXISSIM_H_

//------------------------------------------------------------------------------

#include <Golem/Device/BufCtrlSim/BufCtrlSim.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadArmDesc(void* pContext, const std::string& path, void* pArmDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//
//            t1    t2                t3         t4    t5       
//            ^    ^                 ^           ^ Z  ^         
//            |   /                 /            |   /          
//            |  /                 /             |  /           
//            | /                 /              | /            
//            |/       l1        /       l2      |/       Y     
//            O*****************O****************O---------> t6 
//           /*                /                /|              
//          / *               /                / | T            
//         /  *              /                /  |              
//        /   * l0          /                v X |              
//            *                                                 
//            *^ Z                                              
//            *|                                                
//            *|                                                
//            *| S     Y                                        
//             O-------->                                       
//            /                                                 
//           /                                                  
//          /                                                   
//         v X                                                   
//

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR SixAxisSimArm: public BufCtrlSimArm {
public:	
	/** Number of the manipulator joints */
	static const U32 NUM_JOINTS = 6;

	/** SixAxisSimArm description */
	class GOLEM_LIBRARY_DECLDIR Desc : public BufCtrlSimArm::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(SixAxisSimArm, Arm::Ptr, Context&)

		/** Links lengths */
		Real L0; // [m]
		Real L1; // [m]
		Real L2; // [m]
		Real L3; // [m]

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			BufCtrlSimArm::Desc::setToDefault();

			L0 = Real(0.2); // [m]
			L1 = Real(0.2); // [m]
			L2 = Real(0.2); // [m]
			L3 = Real(0.05); // [m]

			joints.clear();
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				joints.push_back(Joint::Desc::Ptr(new BufCtrlSimJoint::Desc));
			
			name = "6-axis arm simulator";
			
			timeQuant = (SecTmReal)0.002;
			deltaSync = (SecTmReal)0.002;
			deltaRecv = (SecTmReal)0.01;
			deltaSend = (SecTmReal)0.07;

			referencePose.p.v2 += L3;
		}
		
		virtual bool isValid() const {
			if (!BufCtrlSimArm::Desc::isValid())
				return false;

			if (L0 <= REAL_ZERO || L1 <= REAL_ZERO || L2 <= REAL_ZERO || L3 <= REAL_ZERO)
				return false;
			if (joints.size() != NUM_JOINTS)
				return false;

			return true;
		}
	};

protected:
	/** Links lengths */
	Real L0; // [m]
	Real L1; // [m]
	Real L2; // [m]
	Real L3; // [m]

	// Initialisation
	bool create(const Desc& desc);

	SixAxisSimArm(golem::Context& context);

	/** */
	virtual ~SixAxisSimArm();
	
public:
	// Arm
	virtual void forwardTransform(Mat34& trn, const ConfigspaceCoord& cc) const;
	virtual void velocitySpatial(Twist& v, const ConfigspaceCoord& cc, const ConfigspaceCoord& dcc) const;
	virtual void jacobianSpatial(Jacobian& jac, const ConfigspaceCoord& cc) const;
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_SIXAXISSIM_SIXAXISSIM_H_*/
