
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

import beliefmodels.autogen.framing.SimpleSpatioTemporalFrame;
import beliefmodels.autogen.framing.TemporalInterval;
import cast.cdl.CASTTime;
import beliefmodels.arch.BeliefException;

public class SpatioTemporalFrameBuilder {
 

	/**
	 * Create a new spatio-temporal frame with a place, a start time and a end time
	 * 
	 * @param place a string describing the place
	 * @param startTime a starting time
	 * @param endTime an ending time
	 * @return the constructed spatio-temporal frame
	 * @throws BeliefException Thrown if any of the parameters is null or empty
	 */
	public static SimpleSpatioTemporalFrame createSimpleSpatioTemporalFrame 
		(String place, CASTTime startTime, CASTTime endTime) 
		throws BeliefException
	{
		if (place == null || place.equals("") || startTime == null || endTime == null) {
			throw new BeliefException("Error when creating ST frame: parameters cannot be null or empty"); 
		} // end if
		
		TemporalInterval interval = new TemporalInterval(startTime, endTime);
		return new SimpleSpatioTemporalFrame(place, interval);
	} // end method
	
}