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
import org.cognitivesystems.comsys.autogen.ComsysEssentials.AspectualRelation;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Cache;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Event;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.EventDiscRefRelation;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Nucleus;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.State;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.StateDiscRefRelation;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.TemporalRelation;
import org.cognitivesystems.comsys.general.CacheWrapper;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.general.EventStructureUtils;

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
	The abstract class <b>AbstractEventNucleusFactory</b> provides a 
	base implementation of the methods used by factories for producing event 
	nuclei for a particular type of verbal predicate should implement. 
	
	@started 071029
	@version 071028
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public abstract class AbstractEventNucleusFactory 
	implements EventNucleusFactory

{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	/** The ontological sort of the verbal predicate for which an 
		event nucleus is being produced */

	protected String _sort; 	
			
	/** The proposition of the verbal predicate for which an 
		event nucleus is being produced */

	protected String _prop;

	/** The discourse referents for the nodes in the packed logical form
		used for constructing the event nucleus */
		
	protected Cache _discRefsCache;

	// Wrapper for accessing the cache
	
	protected CacheWrapper _discRefs;


	protected boolean logging; 

	//=================================================================
	// CONSTRUCTORS
	//=================================================================

	public AbstractEventNucleusFactory () { 
		init();
	} // end constructor

	protected void init () { 
		_sort = "";
		_prop = "";
		_discRefsCache = null;
		_discRefs = null;
		logging = true;
	} // end 

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** Returns the nominal sort for which the factory can produce an event structure.  */

	public String getSort () { return _sort; }
	
	/** Returns the proposition for which, in conjunction with the sort, the factory can produce 
		an event structure. The method returns <tt>null</tt> if the factory doesn't define a proposition 
		(i.e. only the sort determines whether the factory is applicable. */

	public String getProposition() { return _prop; }

	/** Sets the cache with discourse referents to be used when creating the event nucleus */
	
	public void setDiscRefCache (Cache dr) { 
		_discRefsCache = dr; 
		_discRefs = new CacheWrapper(_discRefsCache);
	} // end setDiscRefCache


	public void setLogging (boolean l) { logging = l; }

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================
	
	/** 
		The method <i>addEventDiscourseReference</a> adds to the event 
		a relation between the discourse referent for the verb, and the
		identifier for the event. 
		
		@param Event event		The event to be updated
		@param String discRef	The identifier of the discourse referent
		@param String mode		The mode of the relation between the event and the referent
		@return Event			The updated event
	*/ 

	public Event addEventDiscourseReference (Event event, String discRef, String mode) { 
		// initialize the result
		Event result = event;
		EventDiscRefRelation drEvRel = new EventDiscRefRelation(event.eventId,discRef,mode);
		result = EventStructureUtils.addEventDiscRefRelation(result,drEvRel);
		return result;
	} // end addEventDiscourseReference

	/** 
		The method <i>addStateDiscourseReference</a> adds to the state
		a relation between the discourse referent a modifier, and the
		identifier for the state. 
		
		@param State  state		The state to be updated
		@param String discRef	The identifier of the discourse referent
		@param String mode		The mode of the relation between the state and the referent
		@return State			The updated state
	*/ 

	public State addStateDiscourseReference (State state, String discRef, String mode) { 
		// initialize the result
		State result = state;
		StateDiscRefRelation drStRel = new StateDiscRefRelation(state.stateId,discRef,mode);
		result = EventStructureUtils.addStateDiscRefRelation(result,drStRel);
		return result;
	} // end addStateDiscourseReference

	/** 
		The method <i>createEvent</i> creates the event part of the 
		event nucleus, based on the given sort and proposition. 
		
		@param String nucleusId	The base ID of the nucleus	
		@param String nomVar	The nominal variable for which the event is being created
		@param String type		The type of event
		@return Event			The event part of the event nucleus
	*/ 

	public Event createEvent (String nucleusId, String nomVar, String type) { 
		// Initialize the result;
		Event result = new Event();
		// Create the id and set up the type
		result.eventId = nucleusId+"-event";
		result.type = type;
		// Get the discourse referent for the given nominal variable
		String discRef = _discRefs.getDiscRef(nomVar);
		// Add discourse referent relation to the event, to bind it into the dialogue model
		result = this.addEventDiscourseReference(result,discRef,"event");				
		// Return the result
		return result;
	} // end createEvent


	/** 
		The method <i>createState</i> creates the state part of the 
		event nucleus, based on the given sort, proposition, and modifiers of the verbal 
		predicate
		
		@param String nucleusId	The base ID of the nucleus	
		@param String nomVar	The nominal variable for which the state is being created
		@param String type		The type of state
		@return Event			The event part of the event nucleus
	*/ 


	public abstract State createState (String nucleusId, String nomVar, TreeMap packedNoms);


	/** 
		The method <i>hasDependent</i> returns a boolean indicating whether the nominal has a dependent of the given type. 
		
		@param head		The head nominal
		@param relation	The type being looked for
		@return boolean	Indicating whether the head has a relation of the given type
	*/ 
	protected boolean hasDependent (PackedNominal head, String relation) { 
		boolean result = false; 
		if (head != null && head.rels != null) { 
			ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext() && !result) { 
				LFRelation rel = (LFRelation) relsIter.next();
				if (rel.mode.equals(relation)) { 
						result = true;
				} // end if.. check whether the right relation 
			} // end while over relations
		} // end check for presence of relations
		return result;
	} // end hasDependent

	/** 
		The method <i>getDependentNominal</i> returns the first relation-type dependent under the 
		given head. The method returns <tt>null</tt> if there is no such dependent. If the type
		is specified as "", simply the first dependent is returned (if any). 
		
		@param head		The head nominal
		@param relation	The type of dependent that is being looked for
		@param packedNoms	The map with the nominals in the packed logical form
		@return PackedNominal The (first) dependent under the head, of the given type 
	*/ 
	protected PackedNominal getDependentNominal (PackedNominal head, String relation, TreeMap packedNoms) 
		throws ComsysException
	{ 
		PackedNominal result = null; 
		boolean depFound = false; 
		if (head != null && head.rels != null) {
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
		if (relation.equals("") && relations.size() > 0) { 
			LFRelation rel = (LFRelation) relations.get(0); 
			result = (PackedNominal) packedNoms.get(rel.dep);
		} else { 
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext() && !depFound) { 
				LFRelation rel = (LFRelation) relsIter.next();
				if (rel.mode.equals(relation)) { 
					// Get the dependent nominal
					String depVar = rel.dep;
					if (packedNoms.containsKey(depVar)) { 
						result = (PackedNominal) packedNoms.get(depVar);
						depFound = true;
					} // end if check for availability of the nominal in the map
				} // end if.. check whether the right relation 
			} // end while over relations
			// If we still haven't found a dependent, cycle over
			// the packing edges
			if (!depFound) { 
				if (head.pEdges != null) { 
					ArrayList<PackingEdge> packingEdges = new ArrayList<PackingEdge>(Arrays.asList(head.pEdges));
					Iterator peIter = packingEdges.iterator();
					while (peIter.hasNext() && !depFound) { 
						PackingEdge packingEdge = (PackingEdge) peIter.next();
						if (packingEdge.mode.equals(relation)) { 
							ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
							if (targets.size() > 0) { 
								PackingNodeTarget target = targets.get(0);
								String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));								
								result = (PackedNominal) packedNoms.get(targetNV);
								log("Getting dependent nominal: scanning packing edge targets under head node ["+head.nomVar+"], taking the first of a total ["+targets.size()+"] with id ["+targetNV+"]");						
							} // end if.. check for available targets
						} // end if .. check for mode of packing edge
					} // end while
				} // end check for there being packing edges
			} // end if.. check whether to check the packing edges 
		} // end if..else check for type
		}
		return result;	
	} // end getDependentNominal


	/** 
		The method <i>produceEventNucleus</i> takes as input a reference to a nominal, and a map with all the 
		(packed) nominals in a packed logical form, to produce an event nucleus for that nominal. The type of 
		event nucleus being returned depends on the sort (and possibly, the proposition) of the nominal. 
		<p>
		The type of event is currently not further classified; the method puts there the sort and the proposition 
		of the verbal predicate. 
		
		@param String	nomVar		The variable of the nominal for which the event structure should be produced
		@param String	nucleusId	The identifier to be used for the event nucleus
		@param TreeMap	packedNoms	A map from nominal variables to PackedNominal objects in the packed logical form
	*/
	
	public EventNucleusResults produceEventNucleus (String nomVar, String nucleusId, TreeMap packedNoms) {
		// Initialize the results
		EventNucleusResults results = new EventNucleusResults (); 
		// Create the event
		Event event = createEvent(nucleusId, nomVar, _sort+"/"+_prop);
		// Create the state
		State state = createState(nucleusId,nomVar, packedNoms); 
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
		return results;
	} // end produceEventNucleus


	public void log(String m) {
		if (logging) { System.out.println("[AbstractEventNucleusFactory] "+m);} 
	} 


} // end interface EventStructureFactory
