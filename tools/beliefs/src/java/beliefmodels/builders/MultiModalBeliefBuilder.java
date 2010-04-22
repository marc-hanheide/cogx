
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

import cast.cdl.WorkingMemoryAddress;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.MultiModalBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.utils.FeatureContentUtils;


public class MultiModalBeliefBuilder  extends AbstractBeliefBuilder{

	   
	/**
	 * Construct a new multi-modal belief
	 * 
	 * @param union the percept union belief
	 * @param address the working memory address for the percept union belief
	 * @param id the 
	 * @return the resulting belief
	 * @throws BinderException 
	 */
	public static MultiModalBelief createNewMultiModalBelief (PerceptUnionBelief union, WorkingMemoryAddress address, String id) 
		throws BeliefException {
		
		return new MultiModalBelief(union.frame, union.estatus, id, union.type, FeatureContentUtils.duplicateContent(union.content), createHistory(address));
	}
	
}
