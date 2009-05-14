//=================================================================
// Copyright (C) 2005 Geert-Jan M. Kruijff (gj@acm.org)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package org.cognitivesystems.comsys.monitors;

//=================================================================
// IMPORTS
//=================================================================

import BindingData.*;
import BindingFeatures.*;
import BindingFeaturesCommon.*;
import cast.core.CASTUtils;
import cast.cdl.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 The class <b>LocalToBindingMapping</b> implements a mapping 
 from locally used proxy structures to the structures used in
 the binding SA. 

@version 080616 (started 080616)
@author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class LocalToBindingMapping {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    boolean logging = true; 

	String subarchID = null;


    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================



    /** 
     *  The basic constructor initializes the internal variables
     */

    public LocalToBindingMapping () {

    } // end constructor


	public LocalToBindingMapping (String sid) { 
		subarchID = sid;
	} 

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	/** 
	The method <i>map</i> constructs a binding SA-relative Object for
	the given feature and value. If the mapping does not know how to 
	construct the Object, then the mapping causes a system exit (to 
	force updating the map). 

	@param	feature	The feature; this determines the object type
	@param	value	The value; this determines the value of the object

	*/ 
 
	public Object map(String feature, String value) 		
	{ 
		Object result = null; 

		boolean mapFound = false; 
		if (feature.equals("Age")) { 
		
			mapFound = true;
		} // end Aspect
		if (feature.equals("Aspect")) { 
		
			mapFound = true;
		} // end Aspect
		if (feature.equals("Attitude")) { 
		
			mapFound = true;
		} // end Aspect
		if (feature.equals("BindingSource")) { 
			SourceData srcData = new SourceData();
			srcData.m_type = CASTUtils.typeName(BindingProxy.class);
			srcData.m_address = new WorkingMemoryAddress(value,subarchID);
			srcData.m_comparable = false; 
			
			mapFound = true;
			return (Object) srcData;
		} // end Aspect		
		if (feature.equals("Cardinality")) { 
		
			mapFound = true;
		} // end Aspect		
		
		
		if (feature.equals("Colour")) { 
		    Colour colour = new Colour();
		    colour.m_colour = value;
			mapFound = true;
			return (Object) colour;
		} // end if check for Colour
		if (feature.equals("Concept")) { 
		    Concept concept = new Concept();
		    concept.m_concept = value;
			mapFound = true;
			return (Object) concept;
		} // end if check for Colour
		if (feature.equals("Delimitation")) { 
		
			mapFound = true;
		} // end Delimitation

		if (feature.equals("Delimination")) { 
			System.err.println("ERROR!!! delimination is a wrongly-named feature!!");
			mapFound = true;
		} // end Delimitation
		if (feature.equals("DiscRef")) { 
			DebugString debugDiscRef = new DebugString();
			debugDiscRef.m_debugString = "Discourse referent: "+value;
			mapFound = true;
			return (Object) debugDiscRef;
		} // end DiscRef

		if (feature.equals("Mood")) { 
		
			mapFound = true;
		} // end Num		
		if (feature.equals("Name")) { 
		    Name name = new Name();
		    name.m_name = value;
			mapFound = true;
			return (Object) name;
		} // end if check for Colour
		
		
		if (feature.equals("Num")) { 
		
			mapFound = true;
		} // end Num
		if (feature.equals("Polarity")) { 
		
			mapFound = true;
		} // end Polarity					
		if (feature.equals("Proposition")) { 
		
			mapFound = true;
		} // end Proposition
		if (feature.equals("Proximity")) { 
			DebugString proximityFeat = new DebugString();
			proximityFeat.m_debugString = "Proximity:"+value;
			mapFound = true;
			return (Object) proximityFeat;
		} // end Proximity		
		if (feature.equals("Quantification")) { 
			
			mapFound = true;
		} // end Quantification		
		if (feature.equals("Salience")) { 
			DebugString salienceFeat = new DebugString();
			salienceFeat.m_debugString = "Salience:"+value;
			mapFound = true;
			return (Object) salienceFeat;
		} // end Proximity			
		if (feature.equals("Shape")) { 
 		    Shape shapeFeat = new Shape();
		    shapeFeat.m_shape = value;
			mapFound = true;
		    return (Object) shapeFeat;
		} // end if check for Size		
		if (feature.equals("Size")) { 
 		    Size sizeFeat = new Size();
		    sizeFeat.m_size = value;
			mapFound = true;
		    return (Object) sizeFeat;
		} // end if check for Size	
		if (feature.equals("State")) { 
 		    AspectualState aspFeat = new AspectualState();
		    aspFeat.m_aspectualState = value;
			mapFound = true;
		    return (Object) aspFeat;
		} // end if check for Size	
		
		if (feature.equals("TemporalFrameType")) { 
			TemporalFrameType tmpFT = null; 
			if (value.equals("ASSERTED")) { tmpFT = TemporalFrameType.ASSERTED; } 
			if (value.equals("PERCEIVED")) { tmpFT = TemporalFrameType.PERCEIVED; } 
			if (value.equals("DESIRED")) {  tmpFT = TemporalFrameType.DESIRED; } 
			if (value.equals("PAST")) { tmpFT = TemporalFrameType.PAST; } 
			if (value.equals("NA")) { tmpFT = TemporalFrameType.NA; } 
			mapFound = true;			
			return (Object) tmpFT; 
		} // end if check for TemporalFrameType		
		if (feature.equals("Tense")) { 
		
			mapFound = true;
		} // end Tense			
		if (!mapFound) { 
			System.out.println("No mapping found for ["+feature+"]");
			System.exit(0); 
		} // end if. 
		return result;
	} // end map
 

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================


    //=================================================================
    // I/O METHODS
    //=================================================================




    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    private void log (String m) {
        if (logging) { System.out.println("[] "+m); }
    }


    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 




