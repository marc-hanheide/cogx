
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

package beliefmodels.builders;


import java.util.Set;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.epstatus.EpistemicStatus;
import beliefmodels.autogen.framing.SpatioTemporalFrame;
import beliefmodels.autogen.history.CASTBeliefHistory;

public class PerceptUnionBuilder extends AbstractBeliefBuilder {


	
	/**
	 * Construct a new percept union belief
	 * 
	 * @param curPlace the current place
	 * @param curTime the curernt time
	 * @param content the belief content
	 * @param hist the percept history
	 * @return the resulting belief
	 * @throws BinderException 
	 */
	public static PerceptUnionBelief createNewPerceptUnionBelief (String id, String type, String curPlace, CASTTime curTime, ProbDistribution content, CASTBeliefHistory hist) 
		throws BeliefException {
		
		if (curPlace == null || curTime == null || content == null || hist == null) {
			throw new BeliefException("error, one of the belief component is null");
		}
		
		// constructing the spatio-temporal frame
		SpatioTemporalFrame frame = 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame(curPlace, curTime, curTime);
		
		// constructing the epistemic status
		EpistemicStatus status = 
			EpistemicStatusBuilder.createNewPrivateEpistemicStatus(EpistemicStatusBuilder.ROBOT_AGENT);
	
		// and creating the belief
		return new PerceptUnionBelief(frame,status,id, type, content,hist);
	}
	

	/**
	 * Construct a new percept union belief similar to a given percept
	 * 
	 * @param percept the percept
	 * @param id the identifier for the new belief
	 * @throws BinderException 
	 */
	public static PerceptUnionBelief createNewSingleUnionBelief (PerceptBelief percept, WorkingMemoryAddress address, String id)  
		throws BeliefException {

		if (percept == null) {
			throw new BeliefException("ERROR, belief is null");
		}
		
		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, percept.content, createHistory(address));
	}
	
	
	
	/**
	 * Construct a new percept union belief similar to a given percept, excepted the existence probability
	 * 
	 * @param percept the percept
	 * @param existProb the existence probability
	 * @param id the identifier for the new belief
	 * @throws BinderException 
	 */
	public static PerceptUnionBelief createNewSingleUnionBelief (PerceptBelief percept, WorkingMemoryAddress address, float existProb, String id)  
		throws BeliefException {

		if (percept == null) {
			throw new BeliefException("ERROR, belief is null");
		}
		else if (percept.content == null) {
			throw new BeliefException("ERROR, belief content is null");
		}
		else if (!(percept.content instanceof DistributionWithExistDep)) {
			throw new BeliefException("ERROR, belief content does not include an existence dependency");
		}
		DistributionWithExistDep newDistrib = 
			BeliefContentBuilder.createNewDistributionWithExistDep(existProb, ((DistributionWithExistDep)percept.content).Pc);
		
		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, newDistrib, createHistory(address));
	}


	/**
	 * Construct a new percept union belief from the merge of a percept and a union
	 * 
	 * @param percept the percept
	 * @param existingUnion the union
	 * @param id the identifier for the new belief
	 * @throws BinderException 
	 */
	public static PerceptUnionBelief createNewDoubleUnionBelief (PerceptBelief percept, WorkingMemoryAddress address,
			PerceptUnionBelief existingUnion, float existProb, String id)
	throws BeliefException {

		if (percept ==null || existingUnion == null) {
			throw new BeliefException("percept is null");
		}
		 
		else if (percept.content == null || existingUnion.content == null) {
			throw new BeliefException ("percept content is null");
		}
		
		
		// we first check if the probabilistic content of the percept and union are
		// defined as a distrib with existence dependency
		if (!(percept.content instanceof DistributionWithExistDep) ||
				!(existingUnion.content instanceof DistributionWithExistDep)) {
			throw new BeliefException("ERROR: percept or union content is not structured " +
			"as a distribution with existence dependency");
		}

		// and that the rest is conditionally independent
		else if (!(((DistributionWithExistDep)percept.content).Pc instanceof CondIndependentDistribs) ||
				!(((DistributionWithExistDep)existingUnion.content).Pc instanceof CondIndependentDistribs)) {
			throw new BeliefException("ERROR: percept or union content does not include " + 
			" a conditionally independent distribution");
		}

		
		// we create a new conditionally independent distrib
		CondIndependentDistribs newCIDistrib = BeliefContentBuilder.createNewCondIndependentDistribs();

		// extracting the conditionally independent distrib in the percept
		CondIndependentDistribs perceptCIDistrib = ((CondIndependentDistribs)((DistributionWithExistDep)percept.content).Pc);

		// and inserting each subdistrib into the new belief content
		for (String featlabel : perceptCIDistrib.distribs.keySet()) {
			BeliefContentBuilder.putNewCondIndependentDistrib(newCIDistrib, 
					(BasicProbDistribution)perceptCIDistrib.distribs.get(featlabel));
		}

		// extracting the conditionally independent distrib in the unnion
		CondIndependentDistribs unionCIDistrib = ((CondIndependentDistribs)((DistributionWithExistDep)existingUnion.content).Pc);

		// and inserting each subdistrib into the new belief content
		for (String featlabel : unionCIDistrib.distribs.keySet()) {
			BeliefContentBuilder.putNewCondIndependentDistrib(newCIDistrib, 
					(BasicProbDistribution)unionCIDistrib.distribs.get(featlabel));
		}

		// creating the new, full belief content
		DistributionWithExistDep newDistrib = BeliefContentBuilder.createNewDistributionWithExistDep(existProb, newCIDistrib);

		// creating and returning the union belief
		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, newDistrib, createHistory(address));
	}

}
