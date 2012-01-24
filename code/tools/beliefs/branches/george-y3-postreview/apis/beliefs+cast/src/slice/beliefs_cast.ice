 
// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de), Pierre Lison (pierre.lison@dfki.de) 
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
   
#include <cast/slice/CDL.ice>
#include <beliefs.ice>

#ifndef BELIEF_CAST_ICE
#define BELIEF_CAST_ICE 
 
/**
 * API [ de.dfki.lt.tr.beliefs ] 
 * 
 * Extension of the belief definition to include CAST functionality
 *
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	25.05.2010
 * @started	13.03.2010
 */
 
module de {
module dfki {
module lt {
module tr {
module beliefs {
module slice { 

// ================================
// SPATIO-TEMPORAL FRAME
// ================================

 
module framing {

/**
 * Abstract class for a spatio-temporal frame
 */
class CASTTemporalInterval extends TemporalInterval {
	cast::cdl::CASTTime start;
	cast::cdl::CASTTime end;
 };

}; 
// end spatiotemporalframe


// LOGICAL CONTENT
module logicalcontent {

/**
 * Formula consisting of a pointer to another
 * belief, referenced by its identifier
 */
class PointerFormula extends dFormula {
      cast::cdl::WorkingMemoryAddress pointer;
      string type;
};


};


// ================================
// BELIEF DEVELOPMENT HISTORY
// ================================

module history {

["java:type:java.util.ArrayList<cast.cdl.WorkingMemoryAddress>"] sequence<cast::cdl::WorkingMemoryAddress> WorkingMemoryAddresses;
["java:type:java.util.ArrayList<cast.cdl.WorkingMemoryPointer>"] sequence<cast::cdl::WorkingMemoryPointer> WorkingMemoryPointers;


/**
 * Abstract class specifying the history of a given belief
 */
class CASTBeliefHistory extends AbstractBeliefHistory{
	WorkingMemoryPointers ancestors;
	WorkingMemoryPointers offspring;
 };

}; 
// end history



// ================================
// MODEL STATUS FOR GEORGE
// ================================

module modelstatus {

dictionary<string,logicalcontent::dFormula> ModelContent;

class CurrentModelStatus extends epobject::EpistemicObject {
      string id;
      ModelContent status;
};

};

// ================================
// END SLICE
// ================================

}; 
// end slice
}; 
// end beliefs
}; 
// end tr
}; 
// end lt
};
// end dfki
}; 
// end de



#endif



