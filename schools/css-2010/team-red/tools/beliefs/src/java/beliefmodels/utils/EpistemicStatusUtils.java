// =================================================================                                                        
// Copyright (C) 2010 Geert-Jan M. Kruijff / DFKI GmbH (gj@dfki.de)                                                               
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

// belief models
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.epstatus.AttributedEpistemicStatus;
import beliefmodels.autogen.epstatus.EpistemicStatus;
import beliefmodels.autogen.epstatus.PrivateEpistemicStatus;
import beliefmodels.autogen.epstatus.SharedEpistemicStatus;

/**
 * Utility methods for handling SLICE EpistemicStatus-type objects
 * 
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100415
 */

public class EpistemicStatusUtils {


	public static boolean isPrivate (EpistemicStatus status) 
		throws BeliefException
	{
		if (status != null) { 
			boolean result = false;
			if (status instanceof PrivateEpistemicStatus) { result = true; } 
			return result;
		} else { 
			throw new BeliefException("Epistemic status (private) cannot be checked for null");
		} // end if..else 
	} // end isPrivate
	
	
	
	
}
