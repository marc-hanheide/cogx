//=================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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

package org.cognitivesystems.comsys.processing;

//=================================================================
// IMPORTS
//=================================================================


// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Cache;

// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.TreeMap;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================

/**
	The interface <b>EventNucleusFactory</b> describes the methods
	which any method implementing a factory for producing event 
	nuclei for a particular type of verbal predicate should implement. 
	
	@started 071026
	@version 071026
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public interface EventNucleusFactory {


	/** Returns the nominal sort for which the factory can produce an event structure.  */
	public String getSort (); 
	
	/** Returns the proposition for which, in conjunction with the sort, the factory can produce 
		an event structure. The method returns <tt>null</tt> if the factory doesn't define a proposition 
		(i.e. only the sort determines whether the factory is applicable. */
	public String getProposition();

	/** Sets the cache with discourse referents to be used */ 
	public void setDiscRefCache (Cache dr); 


	/** 
		The method <i>produceEventNucleus</i> takes as input a reference to a nominal, and a map with all the 
		(packed) nominals in a packed logical form, to produce an event nucleus for that nominal. The type of 
		event nucleus being returned depends on the sort (and possibly, the proposition) of the nominal. 
		
		@param String	nomVar		The variable of the nominal for which the event structure should be produced
		@param String	nucleusId	The identifier to be used for the event nucleus
		@param TreeMap	packedNoms	A map from nominal variables to PackedNominal objects in the packed logical form
	*/
	public EventNucleusResults produceEventNucleus (String nomVar, String nucleusId, TreeMap packedNoms);

} // end interface EventStructureFactory
