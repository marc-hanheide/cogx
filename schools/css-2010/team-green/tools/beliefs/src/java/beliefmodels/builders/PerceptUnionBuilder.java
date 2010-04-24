
// =================================================================                                                        
// Copyright (C) 2010
// Pierre Lison (pierre.lison@dfki.de)
// Geert-Jan M. Kruijff (gj@dfki.de)
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


import java.util.List;
import java.util.Set;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.epstatus.EpistemicStatus;
import beliefmodels.autogen.framing.SpatioTemporalFrame;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.utils.FeatureContentUtils;

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
		
		if (id == null || type == null || id.equals("") || type.equals("") || 
				curPlace == null || curPlace.equals("") || curTime == null || content == null || hist == null) 
		{
			throw new BeliefException("Error in constructing PerceptUnionBelief: parameters cannot be null or empty");
		} // end if  
		
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
	 * Construct a new percept union belief, from a given percept belief and its working memory address (CAST). 
	 * 
	 * @param percept The percept from which the union is to be created
	 * @param address The address to be used for creating the history 
	 * @param id The identifier for the new belief (union)
	 * @throws BinderException Thrown if any of the parameters are empty or null
	 */
	public static PerceptUnionBelief createNewSingleUnionBelief (PerceptBelief percept, WorkingMemoryAddress address, String id)  
		throws BeliefException {

		if (percept == null) {
			throw new BeliefException("Error in constructing PerceptUnionBelief: source belief is null");
		}
		
		if (address == null || address.id == null || address.subarchitecture == null || address.id.equals("") || address.subarchitecture.equals("")) {
			throw new BeliefException("Error in constructing PerceptUnionBelief: address for source belief is null or has empty information");
		}
		
		if (id == null || id.equals("")) { 
			throw new BeliefException("Error in constructing PerceptUnionBelief: id for union belief cannot be null or empty");
		}
		
		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, FeatureContentUtils.duplicateContent(percept.content), createHistory(address));
	}
	
	
	
	/**
	 * Construct a new percept union belief similar to a given percept, 
	 * except for the existence distributed which is constructed with the provided existProb parameter. 
	 * 
	 * @param percept 
	 * 			the percept, which must have a discrete distribution with existence dependency
	 * @pram  address
	 * 			the working memory address of the percpet
	 * @param existProb 
	 * 			the existence probability
	 * @param id 
	 * 			the identifier for the new belief
	 * @throws BinderException 
	 */
	public static PerceptUnionBelief createNewSingleUnionBelief (PerceptBelief percept, WorkingMemoryAddress address, float existProb, String id)  
		throws BeliefException {

		if (address == null || address.id == null || address.subarchitecture == null || address.id.equals("") || address.subarchitecture.equals("")) {
			throw new BeliefException("Error in constructing PerceptUnionBelief: address for source belief is null or has empty information");
		}
		
		if (id == null || id.equals("")) { 
			throw new BeliefException("Error in constructing PerceptUnionBelief: id for union belief cannot be null or empty");
		}

		if (percept == null) {
			throw new BeliefException("Error in constructing PerceptUnionBelief: source belief is null");
		}
		
		if (percept.content == null) {
			throw new BeliefException("Error in constructing PerceptUnionBelief: content in source belief is null");
		}
		else if (!(percept.content instanceof DistributionWithExistDep)) {
			throw new BeliefException("Error in constructing PerceptUnionBelief: content in source belief does not include an existence distribution");
		}
		
		DistributionWithExistDep newDistrib = 
			BeliefContentBuilder.createNewDistributionWithExistDep(existProb, ((DistributionWithExistDep)percept.content).Pc);
		
		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, newDistrib, createHistory(address));
	}

 
	/**
	 * Construct a new percept union belief from the merge of a percept and a union. These beliefs must have content, 
	 * of type DistributionWithExistDep. Each existence distribution must have a set of conditionally independent distributions.
	 * The distributions of the percept and the union must model different features (i.e. no label appears in both distributions). 
	 * 
	 * @param percept 			The percept belief
	 * @param existingUnion 	The union belief
	 * @param id 				The identifier for the new belief
	 * @param addresses			The address of the percept belief and the existing union
	 * @throws BinderException 	Thrown if content is null, or not of the correct type (DistributionWithExistDep and conditionally 
	 * 							independent distributions. 
	 */
	
	public static PerceptUnionBelief createNewDoubleUnionBelief (PerceptBelief percept, List<WorkingMemoryAddress> addresses,
			Belief existingUnion, float existProb, String id)
	throws BeliefException {

		// Check whether we have content to operate on
		if (percept ==null || existingUnion == null) {
			throw new BeliefException("Error in constructing the merger of a union and a percept: "+
					"Percept belief is null");
		} else if (percept.content == null) {
			throw new BeliefException ("Error in constructing the merger of a union and a percept: "+
					"Percept belief content is null");
		} else if (existingUnion.content == null) {
			throw new BeliefException ("Error in constructing the merger of a union and a percept: "+
					"Union belief content is null");
		} // end if..else
		// Check whether we have the right content to operate on: 
		// Content of the percept and union must be defined as a distribution with existence dependency
		if (!(percept.content instanceof DistributionWithExistDep) ||
				!(existingUnion.content instanceof DistributionWithExistDep)) {
			throw new BeliefException("Error in constructing the merger of a union and a percept: "+
					"Percept or union content is not structured as a distribution with existence dependency");
		} 
		// The distribution itself must be a set of conditionally independent distributions
		else if (!(((DistributionWithExistDep)percept.content).Pc instanceof CondIndependentDistribs) ||
				!(((DistributionWithExistDep)existingUnion.content).Pc instanceof CondIndependentDistribs)) {
			throw new BeliefException("Error in constructing the merger of a union and a percept: "+
					"Percept or union content does not include a conditionally independent distribution");
		} // end if..else

		
		// we create a new conditionally independent distrib
		CondIndependentDistribs newCIDistrib = BeliefContentBuilder.createNewCondIndependentDistribs();

		// extracting the conditionally independent distrib in the percept
		CondIndependentDistribs perceptCIDistrib = ((CondIndependentDistribs)((DistributionWithExistDep)percept.content).Pc);

		// and inserting each subdistrib into the new belief content
		for (String featlabel : perceptCIDistrib.distribs.keySet()) {
			BeliefContentBuilder.putNewCondIndependentDistrib(newCIDistrib, 
					(BasicProbDistribution)perceptCIDistrib.distribs.get(featlabel));
		} 

		// extracting the conditionally independent distrib in the union belief
		CondIndependentDistribs unionCIDistrib = ((CondIndependentDistribs)((DistributionWithExistDep)existingUnion.content).Pc);

		// and inserting each subdistrib into the new belief content
		for (String featlabel : unionCIDistrib.distribs.keySet()) {
			BeliefContentBuilder.putNewCondIndependentDistrib(newCIDistrib, 
					(BasicProbDistribution)unionCIDistrib.distribs.get(featlabel));
		}

		// creating the new, full belief content
		DistributionWithExistDep newDistrib = BeliefContentBuilder.createNewDistributionWithExistDep(existProb, newCIDistrib);

		// creating and returning the union belief
		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, newDistrib, createHistory(addresses));
	}

}
