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
import org.cognitivesystems.comsys.autogen.ComsysEssentials.AspectualRelation;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Event;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Nucleus;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.State;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.TemporalRelation;
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
	The class <b>ActionMotionMovePerson</b> provides a factory for producing event 
	nuclei for a "MOVE" verbal predicate, acting on persons -- e.g. "move to the kitchen , move aside". 
	This factory does not produce an event nucleus for another interpretation, namely
	for the case where the patient is the object of moving -- e.g. "move the box to the left."
	If the other interpretation is encountered, a <tt>null</tt> result is produced. 
	
	@started 071029
	@version 071029
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// JAVA CLASS 
//=================================================================

public class ActionMotionMovePerson
	extends		AbstractEventNucleusFactory

{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	//=================================================================
	// CONSTRUCTORS
	//=================================================================

	public ActionMotionMovePerson () { 
		super();
		localInit();
	} // end constructor

	/** Initialization of class-local variables, and setting of 
		super-class variables. */ 

	protected void localInit () { 
		_sort = "action-motion";
		_prop = "move";
	} // end 

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================

	/** 
		The method <i>createState</i> creates the state part of the 
		event nucleus. For MOVE this is an "IS-in" predication over the 
		actor and the destination of the verbal predicate. This is modeled
		by the type of state (determined by destination location) and the list of StateDisRefRelations
		-- one reference to the patient, the other to the anchor of the destination
		location. Note that these references are made using the discourse referents for these
		modifiers, not their nominal variables as per the PLF.  
		
		<p>
		
		If the method finds that there is a patient, then it returns a <tt>null</tt> result. This is
		used to distinguish the actor-moving interpretation from the patient-moving interpretation. 
		
		@param String type	The type of state
		@return State		The state part of the event nucleus
	*/ 	

	public State createState (String nucleusId, String nomVar, TreeMap packedNoms) { 
		// Initialize the result
		State result = new State();
		result.stateId = nucleusId+"-state";
		// Get the verb node
		PackedNominal moveVerb = (PackedNominal) packedNoms.get(nomVar);
		if (moveVerb != null) { 
			try { 
				// Check for the presence of a Patient
				if (!hasDependent(moveVerb,"Patient")) { 
					// Get the Actor node
					PackedNominal actor = this.getDependentNominal(moveVerb,"Actor",packedNoms);		
					// Determine the type on the basis of type of destination
					PackedNominal destination = this.getDependentNominal(moveVerb,"Modifier",packedNoms);
					if (destination != null) {
						result.type = "IS-in";
					
					// Get the anchor node under the destination 
					PackedNominal anchor = this.getDependentNominal(destination,"Anchor",packedNoms);
					// Add the participants as StateDiscRefRelations, using their discourse referents
					String anchorDiscRef  = _discRefs.getDiscRef(anchor.nomVar);
					String actorDiscRef = _discRefs.getDiscRef(actor.nomVar);
					result = this.addStateDiscourseReference(result, anchorDiscRef, "Anchor");
					result = this.addStateDiscourseReference(result, actorDiscRef, "Actor");	
				}
				} else { 
					result = null;
				} // end if..else check for right interpretation
			} catch (ComsysException ce) { 
				System.err.println(ce.getMessage());
				System.exit(0); 				
			} // end try..catch
		} // end if.. check for non-null verb
		// Return the result
		return result;
	} // end createState	

	/** 
		The method <i>produceEventNucleus</i> takes as input a reference to a nominal, and a map with all the 
		(packed) nominals in a packed logical form, to produce an event nucleus for that nominal. The type of 
		event nucleus being returned is "action-motion/move-person".  
		<p>
		The type of event is currently not further classified; the method puts there the sort and the proposition 
		of the verbal predicate. 
		<p>
		To distinguish different interpretations of the verb "move", the method returns <tt>null</tt> if the 
		factory encounters an interpretation of the verb not supported here. 
		
		@param String	nomVar		The variable of the nominal for which the event structure should be produced
		@param String	nucleusId	The identifier to be used for the event nucleus
		@param TreeMap	packedNoms	A map from nominal variables to PackedNominal objects in the packed logical form
	*/
	
	public EventNucleusResults produceEventNucleus (String nomVar, String nucleusId, TreeMap packedNoms) {
		// Initialize the results
		EventNucleusResults results = new EventNucleusResults (); 
		// Create the event
		Event event = createEvent(nucleusId, nomVar, _sort+"/"+_prop+"-person");
		// Create the state
		State state = createState(nucleusId,nomVar, packedNoms); 
		if (state != null) { 
			// Create the nucleus, and add the event and the state
			Nucleus nucleus = new Nucleus();
			nucleus = EventStructureUtils.addEvent(nucleus,event);
			nucleus = EventStructureUtils.addState(nucleus,state);
			// Add to the nucleus the aspectual and temporal relations between the event and the state
			AspectualRelation aspectRel = new AspectualRelation(event.eventId,state.stateId,"Consequence");
			TemporalRelation  tempRel   = new TemporalRelation(event.eventId,state.stateId,"Future");
			nucleus = EventStructureUtils.addAspectualRelation(nucleus,aspectRel);
			nucleus = EventStructureUtils.addTemporalRelation(nucleus,tempRel);
			// Set the results, and return
			results.eventNucleus = nucleus;
		} else { 
			results = null;
		} // end if..else check for right interpretation
		return results;
	} // end produceEventNucleus

} // end class ActionMotionMovePerson.
