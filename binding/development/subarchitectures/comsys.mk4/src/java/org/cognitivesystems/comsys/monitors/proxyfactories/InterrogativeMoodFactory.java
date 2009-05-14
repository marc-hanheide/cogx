//=================================================================
// Copyright (C) 2008 Geert-Jan M. Kruijff (gj@dfki.de)
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

package org.cognitivesystems.comsys.monitors.proxyfactories;

//=================================================================
// IMPORTS
//=================================================================

// ----------------------------------------------------------------
// BINDING imports
// ----------------------------------------------------------------
import binding.common.BindingComponentException;

// ----------------------------------------------------------------
// CAST imports
// ----------------------------------------------------------------
import cast.architecture.subarchitecture.SubarchitectureProcessException;

// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.monitors.AbstractProxyFactory;
import org.cognitivesystems.comsys.monitors.PendingProxyRelation;
import org.cognitivesystems.comsys.monitors.ProxyFactoryResults;
import org.cognitivesystems.comsys.monitors.LocalProxyStructure;

// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;


// ----------------------------------------------------------------
// LF imports
// ----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.repr.lf.autogen.LFPacking.*;

/**
 The class <b>InterrogativeMoodFactory</b> provides a method for producing structural 
 representations associated with the interrogative mood of a verb. The factory is 
 independent of the type of verb it is dealing with, making it the single point of 
 access for handling such structures. 
<p>
 The factory handles the following types of interrogative ("question") structures: 
 
 <ul> 
 <li> Factual Type 0: question after an event occurrence, as perceived (perception class) or event-oriented (event class)</li> 
 </ul>
 
 
 
 Every relation produced here will have "intentional" content status.
 
 @started	080818
 @version	080819 
 @author	Geert-Jan M. Kruijff (gj@dfki.de)
 */ 

public class InterrogativeMoodFactory 
	extends AbstractProxyFactory
{

	// ===========================================================================
	//	CLASS-GLOBAL DATA STRUCTURES
	// ===========================================================================	
	
	
	// The following vectors are initialized in the method classifyQuestionType
	
	Vector WHRestrs;		// a vector with packed nominals for Wh-Restr nodes 
	Vector WHdepTypes;		// a vector of String values for the types of dependents to which the Wh-Restr nom variable poins
	
	
	// ===========================================================================
	//	COMPUTATION METHODS
	// ===========================================================================	
	
	/** 
	The method <i>classifyQuestionType</i> returns an integer indicating the question type associated with the action in the given packed logical form. 
	 
	Decision method:  
	<ul>  
	<li> Type 0: perception with WH pointing to Patient entity or event </li>  
	<li> Type 3: event with WH pointing to Actor entity </li> 
	<li> Type 4: action with WH pointing to an Actor or a Patient </li>
	<li> Type 5: action or event with WH pointing to a spatial circumstance </li> 
	<li> Type 6: action or event with WH indicating a temporal circumstance </li> 	 
	<li> Type 7: action or event with WH pointing to the manner of an action or event  </li> 	 	 
	<li> Type 10: perception as polar, no WH </li>
	 
	</ul>  
	 
	 @param	action		The action to which the question is associated
	 @param	plf			The packed logical form
	 @param	packedNoms	An index to the packed nominals in the plf
	 @returns int		An integer value indicating the question type
	*/ 
	
	private int	classifyQuestionType (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		int type = -1; 
		// Initialize base decision variables
		String actionSort = getSort(action);
		WHRestrs = new Vector();
		WHdepTypes = new Vector(); 
		// -------------------------------------------
		// Check whether polar or factual
		// -------------------------------------------
		String qBaseType = "polar";
		// Check whether we can establish a factual question
		// Trying to get a list of Wh-Restr dependents, either in LF or PLF form
		TreeSet<PackedNominal> whrestrictors = getDependentNominals(action, "Wh-Restr", packedNoms); 
		if (whrestrictors != null) { 
			for (Iterator<PackedNominal> whsIter = whrestrictors.iterator(); whsIter.hasNext(); ) { 
				PackedNominal WHRestr = whsIter.next();
				WHRestrs.add(WHRestr);
				qBaseType = "factual";
				String sort = getSort(WHRestr);
				String WHdepType = null;
				if (sort.equals("specifier") || sort.equals("m-location")) { 
					PackedNominal scope = getDependentNominal(WHRestr,"Scope",packedNoms);
					if (hasDependent(scope,"Scope")) { 
						PackedNominal embscope = getDependentNominal(scope,"Scope",packedNoms);
						WHdepType = getDependentType(action,embscope.nomVar);
					} else { 
						log("Checking action ["+action.nomVar+"] against scope ["+scope.nomVar+"]");
						if (action.nomVar.equals(scope.nomVar)) { 
							WHdepType = "action-root";
						} else { 
							WHdepType = getDependentType(action,scope.nomVar);
						}
					} // end if..else check for further embedding
				} else if (sort.equals("m-dynamic")) {
					if (actionSort.equals("perception") && hasDependent(action,"Event")) { 
						WHdepType = "ObservedEvent";
					} 
				} else if (sort.equals("m-time-point")) { 
					WHdepType = "m-time-point";
				} else if (sort.equals("m-manner")) { 
					WHdepType = "m-manner";
				} else { 
					WHdepType = getDependentType(action,WHRestr.nomVar);	
				} // end if..else check for embedded nomvar
				WHdepTypes.add(WHdepType);
			} // end ... for over restrictors
		} else { 
			TreeSet<PackedNominal> modifiers = getDependentNominals(action, "Modifier", packedNoms); 
			if (modifiers != null) { 
				for (Iterator<PackedNominal> modsIter = modifiers.iterator(); modsIter.hasNext(); ) { 
					PackedNominal modifier = modsIter.next();
					String modSort = getSort(modifier);
					if (modSort.equals("modal")) { 
						qBaseType = "modal"; 
						break; 
					}
				} // end for over modifiers
			}
		} // end if..else check for types other than default polar	
		log("Establish question type as ["+qBaseType+"]");
		log("WH dependency types "+WHdepTypes);
		// -------------------------------------------		
		// Check for question types
		// -------------------------------------------		
		// Check for type 0
		// Type 0: what do you see, what X do you see, what P X do you see
		if (actionSort.equals("perception") && WHdepTypes.contains("Patient") && qBaseType.equals("factual")) { type = 0; }
		else if (actionSort.equals("perception") && WHdepTypes.contains("Event") && qBaseType.equals("factual")) { type = 0; }
		// Type 3: what happened
		else if (actionSort.equals("event") && WHdepTypes.contains("Actor") && qBaseType.equals("factual")) { type = 3; }	
		// Type 4: who ACTION the SOMETHING? who NON-MOTION the SOMETHING? 
		else if (actionSort.equals("action-motion") && WHdepTypes.contains("Actor") && qBaseType.equals("factual")) { type = 4; }
		else if (actionSort.equals("action-non-motion") && WHdepTypes.contains("Actor") && qBaseType.equals("factual")) { type = 4; }
		// Type 4: what did I ACTION? what did I NON-MOTION? 
		else if (actionSort.equals("action-motion") && WHdepTypes.contains("Patient") && qBaseType.equals("factual")) { type = 4; }		
		else if (actionSort.equals("action-non-motion") && WHdepTypes.contains("Patient") && qBaseType.equals("factual")) { type = 4; }
		// Type 5: where did we ACTION? where did something happen? 
		else if (actionSort.startsWith("action-") && WHdepTypes.contains("Result") && qBaseType.equals("factual")) { type = 5; }
		else if (actionSort.equals("event") && WHdepTypes.contains("Result") && qBaseType.equals("factual")) { type = 5; }	
		else if (actionSort.equals("perception") && WHdepTypes.contains("ObservedEvent") && qBaseType.equals("factual")) { type = 5; }			
		else if (actionSort.equals("event") && WHdepTypes.contains("action-root") && qBaseType.equals("factual")) { type = 5; }			
		// Type 6: when did we do ACTION? when did something happen?
		else if (WHdepTypes.contains("m-time-point") && qBaseType.equals("factual")) { type = 6; }		
		else if (actionSort.startsWith("action-") && WHdepTypes.contains("action-root") && qBaseType.equals("factual")) { type = 6; }
		// Type 7: how did something happen? how did we do X? 
		else if (WHdepTypes.contains("m-manner") && qBaseType.equals("factual")) { type = 7; }		
		// Type 10: did you see that 
		else if (actionSort.equals("perception") && qBaseType.equals("polar")) { type = 10; } 
		// Type 11: modal questions, could you would you should you 
		else if (qBaseType.equals("modal")) { type = 11; } 
		else { 	
			log("WARNING: cannot classify question type in the Interrogative Mood Factory by semantic content");
		} // end if..else for question types
		if (type == -1) { 
			log("WARNING: cannot classify question type in the Interrogative Mood Factory, no applicable conditions");
		} // end if.. check for assigned type
		// return the result
		return type; 
	} // end classifyQuestionType
	
	/** 
	 The method <i>produceType0</i> produces the structures for questions after an entity or event occurrence, as perceived (perception class). The 
	 method covers several constructions for Type 0, which vary in specificity: 
	 
	 <ul> 
	 <li> Wh-Restr is an entity ("What did you see") or an animate ("Who did you see"): SPEAKER-KNOWS:Concept?SORT
	 <li> Wh-Restr is a specifier, asking after the identity of one or more entities ("What SOMETHINGS do you see"): SPEAKER-KNOWS:Identity
	 <li> Wh-Restr is a specifier, scoping over a MATERIAL PROPERTY scoping over one or more entities ("What PROPERTY SOMETHINGS do you see"): SPEAKER-KNOWS:PROPERTY
	 <li> Wh-Restr is a specifier, scoping over a TEMPORAL or SPATIAL property of one or more entities ("Where/When did you see the balls"): SPEAKER-KNOWS:Time/Location
	 </ul> 
	 
	 The method returns a list of "FACT-Q" and "SPEAKER-KNOWS" relations. For first two types these point from the action/event to the dependent. 
	 For the time- and location-constructions they point from the action back to the action. 
	*/  
	 
	private ProxyFactoryResults produceType0 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 	
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Cycle over the WHRestr nodes in the WHRestrs vector, and produce the relations
		for (Iterator<PackedNominal> whIter = WHRestrs.iterator(); whIter.hasNext(); ) { 
			PackedNominal WHRestr = whIter.next();
			// Initialize the Speaker-Knows relation
			PendingProxyRelation speakerQ = new PendingProxyRelation();
			speakerQ.addContentStatus("intentional");				
			// Initialize the type of question relation
			PendingProxyRelation typeQ    = new PendingProxyRelation();
			typeQ.addContentStatus("intentional");				
			// Both relations start from the event
			speakerQ.headNomVar = action.nomVar;
			typeQ.headNomVar	= action.nomVar;
			typeQ.relMode		= "Fact-Q";
			// Get the sort of WH-Restr
			String whSort = getSort(WHRestr);
			log("Creating mood for Type 0 given whSort ["+whSort+"]");								
			if (whSort.equals("entity") || whSort.equals("animate") || whSort.equals("event")) { 
				// what did you see, who did you see, who saw the game
				// entity, no further dependents of scoping, just set the relations to point to the nomVar
				speakerQ.depNomVar	= WHRestr.nomVar;
				typeQ.depNomVar		= WHRestr.nomVar;
				speakerQ.relMode	= "SPEAKER-KNOWS:Concept?"+whSort;
			} else if (whSort.equals("specifier")) { 
				PackedNominal scope = getDependentNominal(WHRestr,"Scope",packedNoms);
				if (hasDependent(scope,"Scope")) {
					// what PROPERTY THINGS did you see
					speakerQ.relMode = "SPEAKER-KNOWS:"+scope.prop.prop;
					scope = getDependentNominal(scope,"Scope",packedNoms);
					speakerQ.depNomVar = scope.nomVar;
					speakerQ.depNomVar = scope.nomVar;
				} else { 	
					String scopeSort = getSort(scope);
					if (scopeSort.equals("e-measure")) { 
						PackedNominal kindOwner = getDependentNominal(scope,"Owner",packedNoms);
						String kindSort = getSort(kindOwner);
						speakerQ.relMode = "SPEAKER-KNOWS:Concept?"+kindSort;
						speakerQ.depNomVar	= scope.nomVar;
						typeQ.depNomVar		= scope.nomVar;	
					} else { 
						// what THINGS dids you see
						speakerQ.relMode = "SPEAKER-KNOWS:Identity";
						speakerQ.depNomVar	= scope.nomVar;
						typeQ.depNomVar		= scope.nomVar;					
					} 
				} // end if..else check for embedding of scoping
				// update the list of scoped over variables
			} else if (whSort.equals("m-location")) { 
				// where, scoping back to the event
				speakerQ.depNomVar	= action.nomVar;
				typeQ.depNomVar		= action.nomVar;					
				speakerQ.relMode	= "SPEAKER-KNOWS:Location";
			} else if (whSort.equals("m-time-point")) { 
				// when, scoping back to the event
				speakerQ.depNomVar	= action.nomVar;
				typeQ.depNomVar		= action.nomVar;					
				speakerQ.relMode	= "SPEAKER-KNOWS:Time";				
			} // end if..else				
			// Add the relations to the pending proxy relations list
			pendingRelations.add(typeQ);
			pendingRelations.add(speakerQ);			
		} // end for over WHRestr objects		
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 
	} // end produce type 0
	
	/** 
	 The method <i>produceType3</i> produces the structures for questions after an entity or event occurrence, centered on the event (event class). The 
	 method covers the following constructions for Type 3:
	 
	 <ul> 
	 <li> Wh-Restr is an event ("What happened"): SPEAKER-KNOWS:Identity
	 </ul> 
	*/
	
	private ProxyFactoryResults produceType3 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Cycle over the WHRestr nodes in the WHRestrs vector, and produce the relations
		for (Iterator<PackedNominal> whIter = WHRestrs.iterator(); whIter.hasNext(); ) { 
			PackedNominal WHRestr = whIter.next();
			// Initialize the Speaker-Knows relation
			PendingProxyRelation speakerQ = new PendingProxyRelation();
			speakerQ.addContentStatus("intentional");				
			// Initialize the type of question relation
			PendingProxyRelation typeQ    = new PendingProxyRelation();
			typeQ.addContentStatus("intentional");				
			// Both relations start from the event
			speakerQ.headNomVar = action.nomVar;
			typeQ.headNomVar	= action.nomVar;
			typeQ.relMode		= "Fact-Q";
			// Get the sort of WH-Restr
			String whSort = getSort(WHRestr);
			log("Creating mood for Type 3 given whSort ["+whSort+"]");		
			if (whSort.equals("entity")) { 
				speakerQ.depNomVar	= WHRestr.nomVar;
				typeQ.depNomVar		= WHRestr.nomVar;
				speakerQ.relMode	= "SPEAKER-KNOWS:Identity";
			} else { 
				log("ERROR: unknown wh-restrictor sort ["+whSort+"] in establishing Type 3 question structure");
			} // end if..else check for sort of WH-Restr scoped entity
			// Add the relations to the pending proxy relations list
			pendingRelations.add(typeQ);
			pendingRelations.add(speakerQ);	
		} // end for over WHRestr objects
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 		
	} // end produceType3
	
	
	/** 
	 The method <i>produceType4</i> produces the structures for questions after the Actor or Patient of an action. The action can be a "motion" or a "non-motion." 
	 The method covers the following constructions for Type 4:
	 
	 <ul> 
	 <li> Wh-Restr regards the Actor of an action-motion or action-non-motion: SPEAKER-KNOWS:Identity
	 <li> Wh-Restr regards the Patient of an action-motion or action-non-motion, given as an entity: SPEAKER-KNOWS:Identity
	 <li> Wh-Restr regards the Patient of an action-motion or action-non-motion, given as a specifier with a type of entity: SPEAKER-KNOWS:Concept?proposition-of-entity. This type covers
		  constructions for utterances like "what game did I play" as well as kind-of's like "what kind of game did I play" 
	 <li> 
	 </ul> 
	 */
	
	private ProxyFactoryResults produceType4 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Cycle over the WHRestr nodes in the WHRestrs vector, and produce the relations
		for (Iterator<PackedNominal> whIter = WHRestrs.iterator(); whIter.hasNext(); ) { 
			PackedNominal WHRestr = whIter.next();
			// Initialize the Speaker-Knows relation
			PendingProxyRelation speakerQ = new PendingProxyRelation();
			speakerQ.addContentStatus("intentional");				
			// Initialize the type of question relation
			PendingProxyRelation typeQ    = new PendingProxyRelation();
			typeQ.addContentStatus("intentional");				
			// Both relations start from the event
			speakerQ.headNomVar = action.nomVar;
			typeQ.headNomVar	= action.nomVar;
			typeQ.relMode		= "Fact-Q";
			// Get the sort of WH-Restr
			String whSort = getSort(WHRestr);
			log("Creating mood for Type 4 given whSort ["+whSort+"]");		
			if (whSort.equals("animate") || whSort.equals("entity")) { 
				speakerQ.depNomVar	= WHRestr.nomVar;
				typeQ.depNomVar		= WHRestr.nomVar;
				speakerQ.relMode	= "SPEAKER-KNOWS:Identity";
			} else if (whSort.equals("specifier")) { 
				PackedNominal scope = getDependentNominal(WHRestr,"Scope",packedNoms); 
				String scopeSort = getSort(scope);
				if (scopeSort.equals("e-measure")) { 
					PackedNominal kindOfType = getDependentNominal(scope,"Owner",packedNoms);
					if (kindOfType != null) { 
						speakerQ.depNomVar	= WHRestr.nomVar;
						typeQ.depNomVar		= WHRestr.nomVar;
						speakerQ.relMode	= "SPEAKER-KNOWS:Concept?"+kindOfType.prop.prop;								
					} else { 	
						log("ERROR: Cannot establish what kind-of entity this question Type 4 is about");
					} // end if. else verify kind	
				} else { 
					speakerQ.depNomVar	= WHRestr.nomVar;
					typeQ.depNomVar		= WHRestr.nomVar;
					speakerQ.relMode	= "SPEAKER-KNOWS:Concept?"+scope.prop.prop;					
				} // end if..else check for "kind of" constructions
			} else { 
				log("ERROR: unknown wh-restrictor sort ["+whSort+"] in establishing Type 4 question structure");
			} // end if..else check for sort of WH-Restr scoped entity			
			// Add the relations to the pending proxy relations list
			pendingRelations.add(typeQ);
			pendingRelations.add(speakerQ);	
		} // end for over WHRestr objects
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 		
	} // end produceType4
	
	
	/** 
	 The method <i>produceType5</i> produces the structures for questions after the spatial circumstances of an event. The 
	 method covers the following constructions for Type 5:
	 
	 <ul> 
	 <li> Wh-Restr is ranging over an m-dynamic, which is a Result of an action: SPEAKER-KNOWS:Location
	 <li> Wh-Restr is ranging over an m-location, which is the location where an event took place: SPEAKER-KNOWS:Location
	 </ul> 
	 */
	
	private ProxyFactoryResults produceType5 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Cycle over the WHRestr nodes in the WHRestrs vector, and produce the relations
		for (Iterator<PackedNominal> whIter = WHRestrs.iterator(); whIter.hasNext(); ) { 
			PackedNominal WHRestr = whIter.next();
			// Initialize the Speaker-Knows relation
			PendingProxyRelation speakerQ = new PendingProxyRelation();
			speakerQ.addContentStatus("intentional");				
			// Initialize the type of question relation
			PendingProxyRelation typeQ    = new PendingProxyRelation();
			typeQ.addContentStatus("intentional");				
			// Both relations start from the event
			speakerQ.headNomVar = action.nomVar;
			typeQ.headNomVar	= action.nomVar;
			typeQ.relMode		= "Fact-Q";
			// Get the sort of WH-Restr
			String whSort = getSort(WHRestr);
			log("Creating mood for Type 5 given whSort ["+whSort+"]");		
			if (whSort.equals("m-dynamic") || whSort.equals("m-location")) { 
				if (hasDependent(WHRestr,"Scope")) {
					PackedNominal scope = getDependentNominal(WHRestr,"Scope",packedNoms);
					speakerQ.depNomVar	= scope.nomVar;
					typeQ.depNomVar		= scope.nomVar;
					String actionSort = getSort(action);
					if (actionSort.startsWith("action")) { 
						speakerQ.relMode	= "SPEAKER-KNOWS:Destination";
					} else { 
						speakerQ.relMode	= "SPEAKER-KNOWS:Location";
					}
				} else {
					speakerQ.depNomVar	= WHRestr.nomVar;
					typeQ.depNomVar		= WHRestr.nomVar;
					String actionSort = getSort(action);
					if (actionSort.startsWith("action")) { 
						speakerQ.relMode	= "SPEAKER-KNOWS:Destination";
					} else { 
						if (actionSort.equals("perception") && hasDependent(action,"Event")) { 
							PackedNominal observedEvent = getDependentNominal(action,"Event",packedNoms);
							if (observedEvent != null) { 
								String eventSort = getSort(observedEvent);
								if (eventSort.equals("action-motion")) { 
									speakerQ.depNomVar	= observedEvent.nomVar;
									typeQ.depNomVar		= observedEvent.nomVar;
									speakerQ.relMode	= "SPEAKER-KNOWS:Destination";																	
								} else if (eventSort.equals("action-non-motion")) { 
									speakerQ.depNomVar	= observedEvent.nomVar;
									typeQ.depNomVar		= observedEvent.nomVar;
									speakerQ.relMode	= "SPEAKER-KNOWS:Destination";		
								} else { 	
									speakerQ.relMode	= "SPEAKER-KNOWS:Location";								
								} // end if.. check for type of observed event
							} else { 
								speakerQ.relMode	= "SPEAKER-KNOWS:Location";									
							} // end if..else check for observed event
						} else { 
							speakerQ.relMode	= "SPEAKER-KNOWS:Location";
						} // end 
					}
				} // end if..else check for explicit scope
			} else { 
				log("ERROR: unknown wh-restrictor sort ["+whSort+"] in establishing Type 5 question structure");
			} // end if..else check for sort of WH-Restr scoped entity			
			// Add the relations to the pending proxy relations list
			pendingRelations.add(typeQ);
			pendingRelations.add(speakerQ);	
		} // end for over WHRestr objects
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 		
	} // end produceType5
	
	/** 
	 The method <i>produceType6</i> produces the structures for questions after the temporal circumstances of an event or an action. The 
	 method covers the following constructions for Type 6:
	 
	 <ul> 
	 <li> Wh-Restr is ranging over an m-time-point, which is pointing back to the event or the action: SPEAKER-KNOWS:Time
	 </ul> 
	 */
	
	private ProxyFactoryResults produceType6 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Cycle over the WHRestr nodes in the WHRestrs vector, and produce the relations
		for (Iterator<PackedNominal> whIter = WHRestrs.iterator(); whIter.hasNext(); ) { 
			PackedNominal WHRestr = whIter.next();
			// Initialize the Speaker-Knows relation
			PendingProxyRelation speakerQ = new PendingProxyRelation();
			speakerQ.addContentStatus("intentional");				
			// Initialize the type of question relation
			PendingProxyRelation typeQ    = new PendingProxyRelation();
			typeQ.addContentStatus("intentional");				
			// Both relations start from the event
			speakerQ.headNomVar = action.nomVar;
			typeQ.headNomVar	= action.nomVar;
			typeQ.relMode		= "Fact-Q";
			// Get the sort of WH-Restr
			String whSort = getSort(WHRestr);
			log("Creating mood for Type 6 given whSort ["+whSort+"]");		
			if (whSort.equals("m-time-point")) { 
				PackedNominal scope = getDependentNominal(WHRestr,"Scope",packedNoms);
				speakerQ.depNomVar	= scope.nomVar;
				typeQ.depNomVar		= scope.nomVar;
				speakerQ.relMode	= "SPEAKER-KNOWS:Time";				
			} else { 
				log("ERROR: unknown wh-restrictor sort ["+whSort+"] in establishing Type 6 question structure");
			} // end if..else check for sort of WH-Restr scoped entity			
			// Add the relations to the pending proxy relations list
			pendingRelations.add(typeQ);
			pendingRelations.add(speakerQ);	
		} // end for over WHRestr objects
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 		
	} // end produceType6	
	
	
	/** 
	 The method <i>produceType7</i> produces the structures for questions after the manner in which an action was performed, or an event happened. The 
	 method covers the following constructions for Type 7:
	 
	 <ul> 
	 <li> Wh-Restr is of semantic sort "m-manner", scoping over the event or the action. 
	 </ul> 
	 */
	
	private ProxyFactoryResults produceType7 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Cycle over the WHRestr nodes in the WHRestrs vector, and produce the relations
		for (Iterator<PackedNominal> whIter = WHRestrs.iterator(); whIter.hasNext(); ) { 
			PackedNominal WHRestr = whIter.next();
			// Initialize the Speaker-Knows relation
			PendingProxyRelation speakerQ = new PendingProxyRelation();
			speakerQ.addContentStatus("intentional");				
			// Initialize the type of question relation
			PendingProxyRelation typeQ    = new PendingProxyRelation();
			typeQ.addContentStatus("intentional");				
			// Both relations start from the event
			speakerQ.headNomVar = action.nomVar;
			typeQ.headNomVar	= action.nomVar;
			typeQ.relMode		= "Fact-Q";
			// Get the sort of WH-Restr
			String whSort = getSort(WHRestr);
			log("Creating mood for Type 7 given whSort ["+whSort+"]");		
			if (whSort.equals("m-manner")) { 
				PackedNominal scope = getDependentNominal(WHRestr,"Scope",packedNoms);
				if (scope != null) { 
					speakerQ.depNomVar	= scope.nomVar;
					typeQ.depNomVar		= scope.nomVar;
					speakerQ.relMode	= "SPEAKER-KNOWS:Manner";
				} else { 
					log("ERROR: cannot determine scope under WH ["+WHRestr.nomVar+"/"+whSort+"] in question Type 7");
				} // end if..else 
			} else { 
				log("ERROR: unknown wh-restrictor sort ["+whSort+"] in establishing Type TEMPLATE question structure");
			} // end if..else check for sort of WH-Restr scoped entity			
			// Add the relations to the pending proxy relations list
			pendingRelations.add(typeQ);
			pendingRelations.add(speakerQ);	
		} // end for over WHRestr objects
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 		
	} // end produceType7
	
	
	/** 
	 The method <i>produceType10</i> produces the structures for polar questions about perceptions. The 
	 method covers the following constructions for Type 10:
	 
	 <ul> 
	 <li> Perception event, interrogative: SPEAKER-KNOWS:Perception
	 </ul> 
	 */
	
	private ProxyFactoryResults produceType10 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
			PendingProxyRelation speakerQ = new PendingProxyRelation();
			speakerQ.addContentStatus("intentional");				
			// Initialize the type of question relation
			PendingProxyRelation typeQ    = new PendingProxyRelation();
			typeQ.addContentStatus("intentional");				
			// Both relations start from the event
			speakerQ.headNomVar = action.nomVar;
			typeQ.headNomVar	= action.nomVar;
			typeQ.relMode		= "Polar-Q";
			log("Creating mood for Type 10");		
			speakerQ.depNomVar	= action.nomVar;
			typeQ.depNomVar		= action.nomVar;
			speakerQ.relMode	= "SPEAKER-KNOWS:Perception";
			// Add the relations to the pending proxy relations list
			pendingRelations.add(typeQ);
			pendingRelations.add(speakerQ);	
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 		
	} // end produceType10
	
	/** 
	 The method <i>produceType11</i> produces the structures for modal questions about any type of action or perception. The 
	 method covers the following constructions for Type 11:
	 
	 <ul> 
	 <li> Event or action with modal "could", interrogative: SPEAKER-KNOWS:Ability
	 <li> Event or action with modal "would", interrogative: SPEAKER-KNOWS:Possibility
	 <li> Event or action with modal "should", interrogative: SPEAKER-KNOWS:Necessity
	 </ul> 
	 */
	
	private ProxyFactoryResults produceType11 (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		PendingProxyRelation speakerQ = new PendingProxyRelation();
		speakerQ.addContentStatus("intentional");				
		// Initialize the type of question relation
		PendingProxyRelation typeQ    = new PendingProxyRelation();
		typeQ.addContentStatus("intentional");				
		// Both relations start from the event
		speakerQ.headNomVar = action.nomVar;
		typeQ.headNomVar	= action.nomVar;
		String modalitySort = null; 
		TreeSet<PackedNominal> modifiers = getDependentNominals(action, "Modifier", packedNoms); 
		for (Iterator<PackedNominal> modsIter = modifiers.iterator(); modsIter.hasNext(); ) { 
			PackedNominal modifier = modsIter.next();
			String modSort = getSort(modifier);
			if (modSort.equals("modal")) { 
				modalitySort = modifier.prop.prop; 
				if (modalitySort.equals("can")) { modalitySort = "could"; }
				excludes.add(modifier.nomVar);
				break; 
			}
		} // end for over modifiers		
		typeQ.relMode		= "Modal-Q:"+modalitySort;
		log("Creating mood for Type 11");		
		speakerQ.depNomVar	= action.nomVar;
		typeQ.depNomVar		= action.nomVar;
		if (modalitySort.equals("could") || modalitySort.equals("can")) { 
			speakerQ.relMode	= "SPEAKER-KNOWS:Ability";
		} else if (modalitySort.equals("would")) { 
			speakerQ.relMode	= "SPEAKER-KNOWS:Possibility";			
		} else if (modalitySort.equals("should")) { 
			speakerQ.relMode	= "SPEAKER-KNOWS:Necessity";	
		} 	
		// Add the relations to the pending proxy relations list
		pendingRelations.add(typeQ);
		pendingRelations.add(speakerQ);	
		// Set the final results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);		
		return results; 		
	} // end produceType11
	
	
	
	public ProxyFactoryResults produceProxies (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		ProxyFactoryResults results = null;
		// Classify the question type
		int questionType = this.classifyQuestionType(action,plf,packedNoms); 
		log("WH Restrictors: "+WHRestrs);
		log("WH Restrictor types: "+WHdepTypes);		
		
		
		// Produce the appropriate structures
		if (questionType == 0) { 
			results = produceType0(action,plf,packedNoms);
		} else if (questionType == 3) {
			results = produceType3(action,plf,packedNoms);			
		} else if (questionType == 4) {
			results = produceType4(action,plf,packedNoms);				
		} else if (questionType == 5) {
			results = produceType5(action,plf,packedNoms);	
		} else if (questionType == 6) {
			results = produceType6(action,plf,packedNoms);	
		} else if (questionType == 7) {
			results = produceType7(action,plf,packedNoms);				
		} else if (questionType == 10) {
			results = produceType10(action,plf,packedNoms);	
		} else if (questionType == 11) {
			results = produceType11(action,plf,packedNoms);	
		}// end if..else over question type
		
		log("Pending proxy relations for resulting interrogative structure: "+results.getPendingProxyRelations());
		
		// Return the results
		return results;
	} // end produceProxies
	
	
	
	
	
	
	
} // end class
