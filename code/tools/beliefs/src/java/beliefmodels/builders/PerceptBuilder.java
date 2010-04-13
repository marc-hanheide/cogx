
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

import java.util.ArrayList;
import java.util.List;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.epstatus.EpistemicStatus;
import beliefmodels.autogen.framing.SpatioTemporalFrame;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.EpistemicStatusBuilder;
import beliefmodels.builders.SpatioTemporalFrameBuilder;

public class PerceptBuilder extends AbstractBeliefBuilder {

  
	/**
	 * Construct a new perceptual belief
	 * 
	 * @param curPlace the current place
	 * @param curTime the curernt time
	 * @param content the belief content
	 * @param hist the percept history
	 * @return the resulting belief
	 * @throws BinderException 
	 */
	public static PerceptBelief createNewPerceptBelief (String id, String type, String curPlace, CASTTime curTime, ProbDistribution content, CASTBeliefHistory hist) 
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
		return new PerceptBelief(frame,status,id, type, content,hist);
	}
	
	
	public static CASTBeliefHistory createNewPerceptHistory (WorkingMemoryAddress origin) 
		throws BeliefException
	{	
		if (origin!= null) { 
			List<WorkingMemoryAddress> origins = new ArrayList<WorkingMemoryAddress>();
			origins.add(origin);
			return new CASTBeliefHistory(origins);
		} else {
			throw new BeliefException("Error when creating belief history: cannot create history for null");
		}
	}

}
