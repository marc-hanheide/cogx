
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

package binder.builders;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryPointer;
import binder.arch.BinderException;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.ProbDistribution;
import binder.autogen.epstatus.EpistemicStatus;
import binder.autogen.framing.SpatioTemporalFrame;
import binder.autogen.history.PerceptHistory;

public class PerceptBuilder {

	private static int increment = 0;

	
	
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
	public static PerceptBelief createNewPerceptBelief (String id, String curPlace, CASTTime curTime, ProbDistribution content, PerceptHistory hist) 
		throws BinderException {
		
		if (curPlace == null || curTime == null || content == null || hist == null) {
			throw new BinderException("error, one of the belief component is null");
		}
		
		// constructing the spatio-temporal frame
		SpatioTemporalFrame frame = 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame(curPlace, curTime, curTime);
		
		// constructing the epistemic status
		EpistemicStatus status = 
			EpistemicStatusBuilder.createNewPrivateEpistemicStatus(EpistemicStatusBuilder.ROBOT_AGENT);
	
		// and creating the belief
		return new PerceptBelief(frame,status,id, content,hist);
	}
	
	
	public static PerceptHistory createNewPerceptHistory (WorkingMemoryPointer origin) {
		return new PerceptHistory(origin);
	}

}
