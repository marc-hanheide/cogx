
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

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.ProbDistribution;

public class PerceptUnionBuilder extends AbstractBeliefBuilder {



	/**
	 * Construct a new percept union belief
	 * 
	 * @param percept the percept
	 * @param id the identifier for the new belief
	 * @throws BinderException 
	 */
	public static PerceptUnionBelief createNewSingleUnionBelief (PerceptBelief percept, String id)  
		throws BeliefException {

		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, percept.content, createHistory(percept));
	}


	/**
	 * Construct a new percept union belief from the merge of a percept and a union
	 * 
	 * @param percept the percept
	 * @param existingUnion the union
	 * @param id the identifier for the new belief
	 * @throws BinderException 
	 */
	public static PerceptUnionBelief createNewDoubleUnionBelief (PerceptBelief percept, 
			PerceptUnionBelief existingUnion, float existProb, String id)
	throws BeliefException {

		
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
					(BasicProbDistribution)perceptCIDistrib.distribs.get(featlabel));
		}

		// creating the new, full belief content
		DistributionWithExistDep newDistrib = BeliefContentBuilder.createNewDistributionWithExistDep(existProb, newCIDistrib);

		// creating and returning the union belief
		return new PerceptUnionBelief(percept.frame, percept.estatus, id, percept.type, newDistrib, createHistory(percept));
	}

}
