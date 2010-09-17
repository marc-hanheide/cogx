package motivation.util.viewer.plugins;

import java.util.Vector;

import cast.core.CASTUtils;

import binder.utils.BeliefModelUtils;

import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.*;


/**
 @author	Geert-Jan Kruijff
 @version	091016
 */ 

public class  BeliefInfo implements Plugin {


	@Override
	public Vector<Object> toVector(Ice.ObjectImpl iceObject) {
		Vector<Object> extraInfo=new Vector<Object>();
		Belief belief = (Belief) iceObject;
		if (belief != null) { 
			String agentStatus = "{robot}";
			if (belief.ags instanceof AttributedAgentStatus) {
				agentStatus = "{robot[human]}";
			}
			else if (belief.ags instanceof MutualAgentStatus) {
				agentStatus = "{robot,human}";
			} // end if..else for agent status
			extraInfo.add("Status:"+agentStatus);
			if (belief.sigma != null) { 
				extraInfo.add("ST frame:"+belief.sigma.id);
			} else { 
				extraInfo.add("ST frame: [continue active]");
			} 
			if (belief.phi instanceof UncertainSuperFormula) { 
				extraInfo.add("Phi: "+BeliefModelUtils.getFormulaPrettyPrint(((UncertainSuperFormula)belief.phi)));			
				extraInfo.add("Prob: "+((UncertainSuperFormula)belief.phi).prob);
			} 
		}
		return extraInfo;
	}

}
