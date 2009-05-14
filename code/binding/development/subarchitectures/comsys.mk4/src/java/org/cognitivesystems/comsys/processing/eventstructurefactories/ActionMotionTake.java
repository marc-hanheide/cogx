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

package org.cognitivesystems.comsys.processing.eventstructurefactories;

//=================================================================
// IMPORTS
//=================================================================

// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Event;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Nucleus;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.State;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.general.EventStructureUtils;
import org.cognitivesystems.comsys.processing.EventNucleusResults;
import org.cognitivesystems.comsys.processing.AbstractEventNucleusFactory;

// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.TreeMap;


// ----------------------------------------------------------------
// LF imports
// ----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.repr.lf.autogen.LFPacking.*;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/**
	The class <b>ActionMotionTake</b> provides a factory for producing event 
	nuclei for a "TAKE" verbal predicate. It uses the general production 
	method of the abstract class, only providing a factory-specific way to 
	generate the resultant state of the verbal predicate. 
	
	@started 071029
	@version 071028
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// JAVA CLASS 
//=================================================================

public class ActionMotionTake 
	extends		AbstractEventNucleusFactory

{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	//=================================================================
	// CONSTRUCTORS
	//=================================================================

	public ActionMotionTake () { 
		super();
		localInit();
	} // end constructor

	/** Initialization of class-local variables, and setting of 
		super-class variables. */ 

	protected void localInit () { 
		_sort = "action-non-motion";
		_prop = "take";
	} // end 

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================

	/** 
		The method <i>createState</i> creates the state part of the 
		event nucleus. For TAKE this is a HAS-predication over the 
		actor and the patient of the verbal predicate. This is modeled
		by the type of state ("HAS") and the list of StateDisRefRelations
		-- one reference to the actor, the other to the patient. Note that 
		these references are made using the discourse referents for these
		modifiers, not their nominal variables as per the PLF.  
		
		@param String type	The type of state
		@return State		The state part of the event nucleus
	*/ 	

	public State createState (String nucleusId, String nomVar, TreeMap packedNoms) { 
		// Initialize the result
		State result = new State();
		result.stateId = nucleusId+"-state";
		result.type = "HAS";
		// get the verb node
		PackedNominal takeVerb = (PackedNominal) packedNoms.get(nomVar);
		if (takeVerb != null) { 
			try { 
				// get the actor node
				PackedNominal actor = this.getDependentNominal(takeVerb,"Actor",packedNoms);
				// get the patient node
				PackedNominal patient = this.getDependentNominal(takeVerb,"Patient",packedNoms);		
				// Add the participants as StateDiscRefRelations, using their discourse referents
				String actorDiscRef	  = _discRefs.getDiscRef(actor.nomVar);
				String patientDiscRef = _discRefs.getDiscRef(patient.nomVar);
				result = this.addStateDiscourseReference(result, actorDiscRef, "Actor");
				result = this.addStateDiscourseReference(result, patientDiscRef, "Patient");			
			} catch (ComsysException ce) { 
				System.err.println(ce.getMessage());
				System.exit(0); 					
			} // end try .. catch 
		} // end if.. check for non-null verb
		// Return the result
		return result;
	} // end createState	


} // end class ActionMotionTake
