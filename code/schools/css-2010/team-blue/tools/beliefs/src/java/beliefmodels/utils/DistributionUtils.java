
// =================================================================                                                        
// Copyright (C) 2010 DFKI GmbH / 
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

// Package
package beliefmodels.utils;

// Beliefmodel imports
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.DistributionWithExistDep;

/**
 * The class specifies several utility methods for accessing (get/set) content in distributions, 
 * as part of the content of a belief. 
 * 
 * @author 	gj, pierre.lison
 * @version	100416
 */


public class DistributionUtils {

	/**
	 * For a given belief, if its content specifies a distribution with an existence probability, 
	 * then that probability is returned. Else, 1.0f is returned
	 * @param b
	 * @return Existence probability if specified, else 1.0f
	 */
	
	public static float getExistenceProbability (Belief b) 
	{
		if (b.content instanceof DistributionWithExistDep) {
			return ((DistributionWithExistDep)b.content).existProb;
		}
		return 1.0f;
	} // end method
	
	/**
	 * Sets the existence probability of a belief's content to the provided value. 
	 * If the belief content does not specify a distribution with an existence probability, this operation is 
	 * throws an exception. 
	 * 
	 * @param b The belief to be updated
	 * @param newExistProb The new value for the existence probability of the belief's content distribution
	 */
	
	public static void setExistenceProbability (Belief b, float newExistProb) 
		throws BeliefException 
	{
		if (b.content instanceof DistributionWithExistDep) {
			((DistributionWithExistDep)b.content).existProb = newExistProb;
		} else {
			throw new BeliefException ("Error in setting existence probability: Provided belief has no distribution "+
					"of type DistributionWithExistDep but is ["+b.content.getClass().getName()+"]");
		} // end if..else
	} // end method
	
	
	
	
	
} // end class
