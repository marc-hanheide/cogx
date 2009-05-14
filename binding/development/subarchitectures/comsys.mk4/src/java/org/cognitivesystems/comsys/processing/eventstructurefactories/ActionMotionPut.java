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
	The class <b>ActionMotionPut</b> provides a factory for producing event 
	nuclei for a "PUT" verbal predicate. It uses the general production 
	method of the abstract class, only providing a factory-specific way to 
	generate the resultant state of the verbal predicate. 
	
	@started 071029
	@version 071029
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// JAVA CLASS 
//=================================================================

public class ActionMotionPut
	extends		AbstractEventNucleusFactory

{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	//=================================================================
	// CONSTRUCTORS
	//=================================================================

	public ActionMotionPut () { 
		super();
		localInit();
	} // end constructor

	/** Initialization of class-local variables, and setting of 
		super-class variables. */ 

	protected void localInit () { 
		_sort = "action-non-motion";
		_prop = "put";
	} // end 

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================

	/** 
		The method <i>createState</i> creates the state part of the 
		event nucleus. For PUT this is an "IS-"+location-type predication over the 
		patient and the destination of the verbal predicate. This is modeled
		by the type of state (determined by destination location) and the list of StateDisRefRelations
		-- one reference to the patient, the other to the anchor of the destination
		location. Note that these references are made using the discourse referents for these
		modifiers, not their nominal variables as per the PLF.  
		
		@param String type	The type of state
		@return State		The state part of the event nucleus
	*/ 	

	public State createState (String nucleusId, String nomVar, TreeMap packedNoms) { 
		// Initialize the result
		State result = new State();
		result.stateId = nucleusId+"-state";
		// Get the verb node
		PackedNominal putVerb = (PackedNominal) packedNoms.get(nomVar);
		if (putVerb != null) { 
			try { 
				// Determine the type on the basis of type of destination
				PackedNominal destination = this.getDependentNominal(putVerb,"Result",packedNoms);
				if (destination != null) {
					result.type = "IS-"+destination.prop.prop;
				
				// Get the anchor node under the destination 
				PackedNominal anchor = this.getDependentNominal(destination,"Anchor",packedNoms);
				// Get the Patient node
				PackedNominal patient = this.getDependentNominal(putVerb,"Patient",packedNoms);		
				// Add the participants as StateDiscRefRelations, using their discourse referents
				String anchorDiscRef  = _discRefs.getDiscRef(anchor.nomVar);
				String patientDiscRef = _discRefs.getDiscRef(patient.nomVar);
				result = this.addStateDiscourseReference(result, anchorDiscRef, "Anchor");
				result = this.addStateDiscourseReference(result, patientDiscRef, "Patient");			
				}
				} catch (ComsysException ce) { 
				System.err.println(ce.getMessage());
				System.exit(0); 				
			} // end try..catch
		} // end if.. check for non-null verb
		// Return the result
		return result;
	} // end createState	


} // end class ActionMotionPut
