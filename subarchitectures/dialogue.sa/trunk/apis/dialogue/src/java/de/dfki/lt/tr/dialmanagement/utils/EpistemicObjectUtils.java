// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@dfki.de)                                                                
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


package de.dfki.lt.tr.dialmanagement.utils;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Comparator;

import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.framing.TemporalInterval;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * Utility functions for creating and manipulating epistemic objects 
 * (beliefs, events, intentions)
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 07/10/2010
 *
 */
 
public class EpistemicObjectUtils {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// incremental counter for forging identifiers
	static int incrCounter = 0;
	
		
	
	/**
	 * Create a simple attributed communicative intention with a single postcondition 
	 * associated with a probability value
	 * 
	 * @param postcondition the postcondition
	 * @param prob the probability of the postcondition
	 * @return the newly constructed attributed intention
	 * @throws DialogueException if postcondition is ill-formed
	 */
	public static CommunicativeIntention createSimpleAttributedCommunicativeIntention 
		(dFormula postcondition, float prob) throws DialogueException {
		
		HashMap<dFormula,Float> postconditions = new HashMap<dFormula,Float>();
		postconditions.put(postcondition, prob);
		return createAttributedCommunicativeIntention(postconditions);
	}
	 
	
	/**
	 * Create a simple private communicative intention with a single postcondition 
	 * associated with a probability value
	 * 
	 * @param postcondition the postcondition
	 * @param prob the probability of the postcondition
	 * @return the newly constructed private intention
	 * @throws DialogueException if postcondition is ill-formed
	 */
	public static CommunicativeIntention createSimplePrivateCommunicativeIntention 
		(dFormula postcondition, float prob) throws DialogueException {
		
		HashMap<dFormula,Float> postconditions = new HashMap<dFormula,Float>();
		postconditions.put(postcondition, prob);
		return createPrivateCommunicativeIntention(postconditions);
	}
	
	
	/**
	 * Create an attributed communicative intention containing a list of postconditions 
	 * (each of which is associated with a probability value)
	 * 
	 * @param postconditions a hashmap mapping each postcondition to its probability value
	 * @return the newly constructed attributed intention
	 * @throws DialogueException if postcondition is ill-formed
	 */
	public static CommunicativeIntention createAttributedCommunicativeIntention 
	(HashMap<dFormula,Float> postconditions) throws DialogueException {
		
	    SpatioTemporalFrame frame = new SpatioTemporalFrame ("here", new TemporalInterval(),1.0f);
		AttributedEpistemicStatus attrib = new AttributedEpistemicStatus ("robot", Arrays.asList("human"));
		
		List<IntentionalContent> intents = new LinkedList<IntentionalContent>();
		
		for (dFormula formula: postconditions.keySet()) {
			intents.add((new IntentionalContent(Arrays.asList("robot"), 
					FormulaUtils.constructFormula(""), formula, postconditions.get(formula))));
		}
		return new CommunicativeIntention(new Intention(frame, attrib, forgeNewId(), intents));
	}
	
	
	
	/**
	 * Create an attributed intention containing a list of postconditions (each of which is associated
	 * with a probability value)
	 * 
	 * @param postconditions a list of <form,prob> pairs
	 * @throws DialogueException 
	 * @return the newly constructed attributed intention
	 */
	public static CommunicativeIntention createAttributedCommunicativeIntention 
	(List<FormulaProbPair> pairs) throws DialogueException {
		
	    SpatioTemporalFrame frame = new SpatioTemporalFrame ("here", new TemporalInterval(), 1.0f);
		AttributedEpistemicStatus attrib = new AttributedEpistemicStatus ("robot", Arrays.asList("human"));
		
		List<IntentionalContent> intents = new LinkedList<IntentionalContent>();
		
		for (FormulaProbPair pair: pairs) {
			intents.add((new IntentionalContent(Arrays.asList("robot"), FormulaUtils.constructFormula(""), pair.val, pair.prob)));
		}
		return new CommunicativeIntention(new Intention(frame, attrib, forgeNewId(), intents));
	}
	

	/**
	 * Create a private intention containing a list of postconditions (each of which is associated
	 * with a probability value)
	 * 
	 * @param postconditions a hashmap mapping each postcondition to its probability value
	 * @return the newly constructed private intention
	 * @throws DialogueException if formulae are ill-formed
	 */
	public static CommunicativeIntention createPrivateCommunicativeIntention 
	(HashMap<dFormula,Float> postconditions) throws DialogueException {
		
	    SpatioTemporalFrame frame = new SpatioTemporalFrame ("here", new TemporalInterval(),1.0f);
		PrivateEpistemicStatus priv = new PrivateEpistemicStatus ("robot");
		
		List<IntentionalContent> intents = new LinkedList<IntentionalContent>();
		
		for (dFormula formula: postconditions.keySet()) {
			intents.add((new IntentionalContent(Arrays.asList("robot"), 
					FormulaUtils.constructFormula(""), formula, postconditions.get(formula))));
		}
		return new CommunicativeIntention(new Intention(frame, priv, forgeNewId(), intents));
	}
	
	
	/**
	 * Create a new event with a list of alternative event descriptions, with associated
	 * probabilities
	 * 
	 * @param events a hashmap mapping each event description to its probability value
	 * @return the newly constructed event
	 * @throws DialogueException if descriptions are ill-formed
	 */
	public static Event createEvent (HashMap<dFormula,Float> events) throws DialogueException {
		
	    SpatioTemporalFrame frame = new SpatioTemporalFrame ("here", new TemporalInterval(),1.0f);
		PrivateEpistemicStatus priv = new PrivateEpistemicStatus ("robot");
		
		List<FormulaProbPair> content = new LinkedList<FormulaProbPair>();
		
		for (dFormula formula: events.keySet()) {
			content.add((new FormulaProbPair(formula, events.get(formula))));
		}
		return new Event(frame, priv, forgeNewId(), new BasicProbDistribution("content", new FormulaValues(content)));
	}
	
	
	/**
	 * Create a new simple event with a unique event description and associated probability
	 * 
	 * @param event the event description
	 * @param prob the probability
	 * @return the newly constructed event 
	 * @throws DialogueException if the event description is ill-formed
	 */
	public static Event createSimpleEvent (dFormula event, float prob) throws DialogueException {
		HashMap<dFormula,Float> events = new HashMap<dFormula,Float>();
		events.put(event, prob);
		return createEvent(events);
	}
	
	
	public static Event createSimpleEvent (String eventstr, float prob) throws DialogueException {
		HashMap<dFormula,Float> events = new HashMap<dFormula,Float>();
		events.put(FormulaUtils.constructFormula(eventstr), prob);
		return createEvent(events);
	}
	

	/**
	 * Sort the list of <formula,prob> pairs by decreasing probability (i.e. the pair
	 * with the highest probability comes first, then the second, und so weiter)
	 * 
	 * @param initContent the initial, unsorted list
	 * @return the sorted list
	 */
	public static List<FormulaProbPair> sortFormulaValues (List<FormulaProbPair> formProbPairs) {
		
		Collections.sort(formProbPairs, new FormulaProbPairsComparator());
		return formProbPairs;
	}
	
	
	/**
	 * Forge a new identifier
	 * @return
	 */
	public static String forgeNewId () {
		incrCounter++;
		return "id" + incrCounter;
	}
	

	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[epobjectutils] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[epobjectutils] " + s);
		}
	}
 }


/**
 * Comparator for intentional content
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 * 
 */
final class IntentionalContentComparator implements Comparator<IntentionalContent> {

	
	/**
	 * Returns a positive value if o2 > o1, 0 if o2 == o1, and a negative value if o2 < 01
	 * 
	 */
	@Override
	public int compare(IntentionalContent o1, IntentionalContent o2) {
		if (o1 != null && o2 != null) {
			return (int)(o2.probValue - o1.probValue)*10000 ;
		}
		return 0;
	}
	
}



/**
 * Comparator for intentional content
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 * 
 */
final class FormulaProbPairsComparator implements Comparator<FormulaProbPair> {

	/**
	 * Returns a positive value if o2 > o1, 0 if o2 == o1, and a negative value if o2 < 01
	 * 
	 */
	@Override
	public int compare(FormulaProbPair o1, FormulaProbPair o2) {
		if (o1 != null && o2 != null) {
			return (int)(o2.prob - o1.prob)*10000 ;
		}
		return 0;
	}
	
	


}
