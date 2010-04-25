
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
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.beliefs.TemporalUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionValues;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.epstatus.EpistemicStatus;
import beliefmodels.autogen.framing.SpatioTemporalFrame;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.utils.FeatureContentUtils;

public class TemporalUnionBuilder extends AbstractBeliefBuilder {


	
	/**
	 * Construct a new temporal union belief
	 * 
	 * @param curPlace the current place
	 * @param curTime the curernt time
	 * @param content the belief content
	 * @param hist the mmbelief history
	 * @return the resulting belief
	 * @throws BinderException 
	 */
	public static TemporalUnionBelief createNewTemporalUnionBelief (String id, String type, String curPlace, CASTTime curTime, 
			ProbDistribution content, CASTBeliefHistory hist) 
		throws BeliefException {
		
		if (id == null || type == null || id.equals("") || type.equals("") || 
				curPlace == null || curPlace.equals("") || curTime == null || content == null || hist == null) 
		{
			throw new BeliefException("Error in constructing TemporalUnionBelief: parameters cannot be null or empty");
		} // end if  
		
		// constructing the spatio-temporal frame
		SpatioTemporalFrame frame = 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame(curPlace, curTime, curTime);
		
		// constructing the epistemic status
		EpistemicStatus status = 
			EpistemicStatusBuilder.createNewPrivateEpistemicStatus(EpistemicStatusBuilder.ROBOT_AGENT);
	
		// and creating the belief
		return new TemporalUnionBelief(frame,status,id, type, content,hist);
	}
	

	/**
	 * Construct a new mmbelief union belief, from a given mmbelief belief and its working memory address (CAST). 
	 * 
	 * @param mmbelief The mmbelief from which the union is to be created
	 * @param address The address to be used for creating the history 
	 * @param id The identifier for the new belief (union)
	 * @throws BinderException Thrown if any of the parameters are empty or null
	 */
	public static TemporalUnionBelief createNewSingleUnionBelief (MultiModalBelief mmbelief, 
			WorkingMemoryAddress address, String id)  
		throws BeliefException {

		if (mmbelief == null) {
			throw new BeliefException("Error in constructing TemporalUnionBelief: source belief is null");
		}
		
		if (address == null || address.id == null || address.subarchitecture == null || address.id.equals("") || address.subarchitecture.equals("")) {
			throw new BeliefException("Error in constructing TemporalUnionBelief: address for source belief is null or has empty information");
		}
		
		if (id == null || id.equals("")) { 
			throw new BeliefException("Error in constructing TemporalUnionBelief: id for union belief cannot be null or empty");
		}
		
		return new TemporalUnionBelief(mmbelief.frame, mmbelief.estatus, id, mmbelief.type, FeatureContentUtils.duplicateContent(mmbelief.content), createHistory(address));
	}
	
	
	
	/**
	 * Construct a new mmbelief union belief similar to a given mmbelief, 
	 * except for the existence distributed which is constructed with the provided existProb parameter. 
	 * 
	 * @param mmbelief 
	 * 			the mmbelief, which must have a discrete distribution with existence dependency
	 * @pram  address
	 * 			the working memory address of the percpet
	 * @param existProb 
	 * 			the existence probability
	 * @param id 
	 * 			the identifier for the new belief
	 * @throws BinderException 
	 */
	public static TemporalUnionBelief createNewSingleUnionBelief (MultiModalBelief mmbelief, 
			WorkingMemoryAddress address, float existProb, String id)  
		throws BeliefException {

		if (address == null || address.id == null || address.subarchitecture == null || address.id.equals("") || address.subarchitecture.equals("")) {
			throw new BeliefException("Error in constructing TemporalUnionBelief: address for source belief is null or has empty information");
		}
		
		if (id == null || id.equals("")) { 
			throw new BeliefException("Error in constructing TemporalUnionBelief: id for union belief cannot be null or empty");
		}

		if (mmbelief == null) {
			throw new BeliefException("Error in constructing TemporalUnionBelief: source belief is null");
		}
		
		if (mmbelief.content == null) {
			throw new BeliefException("Error in constructing TemporalUnionBelief: content in source belief is null");
		}
		else if (!(mmbelief.content instanceof DistributionWithExistDep)) {
			throw new BeliefException("Error in constructing TemporalUnionBelief: content in source belief does not include an existence distribution");
		}
		
		DistributionWithExistDep newDistrib = 
			BeliefContentBuilder.createNewDistributionWithExistDep(existProb, ((DistributionWithExistDep)mmbelief.content).Pc);
		
		return new TemporalUnionBelief(mmbelief.frame, mmbelief.estatus, id, mmbelief.type, newDistrib, createHistory(address));
	}

 
	/**
	 * Construct a new mmbelief union belief from the merge of a mmbelief and a union. These beliefs must have content, 
	 * of type DistributionWithExistDep. Each existence distribution must have a set of conditionally independent distributions.
	 * The distributions of the mmbelief and the union must model different features (i.e. no label appears in both distributions). 
	 * 
	 * @param mmbelief 			The mmbelief belief
	 * @param existingUnion 	The union belief
	 * @param id 				The identifier for the new belief
	 * @param addresses			The address of the mmbelief belief and the existing union
	 * @throws BinderException 	Thrown if content is null, or not of the correct type (DistributionWithExistDep and conditionally 
	 * 							independent distributions. 
	 */
	
	public static TemporalUnionBelief createNewDoubleUnionBelief (MultiModalBelief mmbelief, List<WorkingMemoryAddress> addresses,
			Belief existingUnion, float existProb, String id)
	throws BeliefException {

		// Check whether we have content to operate on
		if (mmbelief ==null || existingUnion == null) {
			throw new BeliefException("Error in constructing the merger of a union and a mmbelief: "+
					"mmbelief belief is null");
		} else if (mmbelief.content == null) {
			throw new BeliefException ("Error in constructing the merger of a union and a mmbelief: "+
					"mmbelief belief content is null");
		} else if (existingUnion.content == null) {
			throw new BeliefException ("Error in constructing the merger of a union and a mmbelief: "+
					"Union belief content is null");
		} // end if..else
		// Check whether we have the right content to operate on: 
		// Content of the mmbelief and union must be defined as a distribution with existence dependency
		if (!(mmbelief.content instanceof DistributionWithExistDep) ||
				!(existingUnion.content instanceof DistributionWithExistDep)) {
			throw new BeliefException("Error in constructing the merger of a union and a mmbelief: "+
					"mmbelief or union content is not structured as a distribution with existence dependency");
		} 
		// The distribution itself must be a set of conditionally independent distributions
		else if (!(((DistributionWithExistDep)mmbelief.content).Pc instanceof CondIndependentDistribs) ||
				!(((DistributionWithExistDep)existingUnion.content).Pc instanceof CondIndependentDistribs)) {
			throw new BeliefException("Error in constructing the merger of a union and a mmbelief: "+
					"mmbelief or union content does not include a conditionally independent distribution");
		} // end if..else

		
		// we create a new conditionally independent distrib
		CondIndependentDistribs newCIDistrib = BeliefContentBuilder.createNewCondIndependentDistribs();

		// extracting the conditionally independent distrib in the mmbelief
		CondIndependentDistribs mmbeliefCIDistrib = ((CondIndependentDistribs)((DistributionWithExistDep)mmbelief.content).Pc);

		// and inserting each subdistrib into the new belief content
		for (String featlabel : mmbeliefCIDistrib.distribs.keySet()) {
			
			String newKey = ((BasicProbDistribution)mmbeliefCIDistrib.distribs.get(featlabel)).key ; // + "_t1";
			DistributionValues distribValues = ((BasicProbDistribution)mmbeliefCIDistrib.distribs.get(featlabel)).values;
			BasicProbDistribution newSubDistrib = new BasicProbDistribution(newKey, distribValues);
			
			BeliefContentBuilder.putNewCondIndependentDistrib(newCIDistrib, newSubDistrib);
		} 

		// extracting the conditionally independent distrib in the union belief
		CondIndependentDistribs unionCIDistrib = ((CondIndependentDistribs)((DistributionWithExistDep)existingUnion.content).Pc);

		// and inserting each subdistrib into the new belief content
		for (String featlabel : unionCIDistrib.distribs.keySet()) {
			String newKey = ((BasicProbDistribution)unionCIDistrib.distribs.get(featlabel)).key  ; // + "_t2";
			DistributionValues distribValues = ((BasicProbDistribution)unionCIDistrib.distribs.get(featlabel)).values;
			BasicProbDistribution newSubDistrib = new BasicProbDistribution(newKey, distribValues);
			
			BeliefContentBuilder.putNewCondIndependentDistrib(newCIDistrib, newSubDistrib);
		}

		// creating the new, full belief content
		DistributionWithExistDep newDistrib = BeliefContentBuilder.createNewDistributionWithExistDep(existProb, newCIDistrib);

		// creating and returning the union belief
		return new TemporalUnionBelief(mmbelief.frame, mmbelief.estatus, id, mmbelief.type, newDistrib, createHistory(addresses));
	}

}
