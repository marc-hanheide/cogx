// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
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
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.framing.TemporalInterval;
import de.dfki.lt.tr.beliefs.slice.history.AbstractBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * Utility functions for creating and manipulating epistemic objects 
 * (beliefs, events, intentions)
 *  
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class EpistemicObjectUtils {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// incremental counter for forging identifiers
	static int incrCounter = 0;

	
	// a few constants
	public static final SpatioTemporalFrame curFrame = new SpatioTemporalFrame ("here", new TemporalInterval(),1.0f);
	public static final List<String> robotAgent = Arrays.asList("self");
	public static final List<String> humanAgent = Arrays.asList("human");	
	public static final EpistemicStatus privateStatus = new PrivateEpistemicStatus("self");
	public static final EpistemicStatus attributedStatus = new AttributedEpistemicStatus(robotAgent.get(0), humanAgent);
	 
		
	
	// ==============================================================
	// METHODS FOR CREATING NEW EVENTS
	// ==============================================================
	
	 
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
	

	
	
	/**
	 * Create a simple event with a formula description and a probability
	 * 
	 * @param eventstr the string representation of the event formula
	 * @param prob the probability of the event
	 * @return the created event
	 * @throws DialogueException if the creation failed
	 */
	public static Event createSimpleEvent (String eventstr, float prob) throws DialogueException {
		HashMap<dFormula,Float> events = new HashMap<dFormula,Float>();
		events.put(FormulaUtils.constructFormula(eventstr), prob);
		return createEvent(events);
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
		PrivateEpistemicStatus priv = new PrivateEpistemicStatus ("self");
		
		List<FormulaProbPair> content = new LinkedList<FormulaProbPair>();
		
		for (dFormula formula: events.keySet()) {
			content.add((new FormulaProbPair(formula, events.get(formula))));
		}
		return new Event(frame, priv, forgeNewId(), new BasicProbDistribution("content", new FormulaValues(content)));
	}
	
	
	
	// ==============================================================
	// METHODS FOR CREATING NEW BELIEFS
	// ==============================================================
	
	 
	
	/**
	 * Create a new simple belief with a unique belief description and associated probability
	 * 
	 * @param belief the belief description
	 * @param featurelabel the feature label attached to the belief
	 * @param prob the probability
	 * @return the newly constructed belief 
	 * @throws DialogueException if the belief description is ill-formed
	 */
	public static dBelief createSimpleBelief (dFormula content, float prob, String featurelabel) throws DialogueException {
		HashMap<dFormula,Float> altcontent = new HashMap<dFormula,Float>();
		altcontent.put(content, prob);
		return createBelief(altcontent,featurelabel);
	}
	
	
	/**
	 * Create a new belief with a list of alternative content and associated
	 * probabilities
	 * 
	 * @param altcontent a hashmap mapping each content description to its probability value
	 * @param featurelabel the feature label attached to the belief
	 * @return the newly constructed belief
	 * @throws DialogueException if descriptions are ill-formed
	 */
	public static dBelief createBelief (HashMap<dFormula,Float> altcontent, String featurelabel) throws DialogueException {
		
	    SpatioTemporalFrame frame = new SpatioTemporalFrame ("here", new TemporalInterval(),1.0f);
		PrivateEpistemicStatus priv = new PrivateEpistemicStatus ("self");
		
		List<FormulaProbPair> content = new LinkedList<FormulaProbPair>();
		
		for (dFormula formula: altcontent.keySet()) {
			content.add((new FormulaProbPair(formula, altcontent.get(formula))));
		}
		return new dBelief(frame, priv, forgeNewId(), "", 
				new BasicProbDistribution(featurelabel, new FormulaValues(content)), 
				new AbstractBeliefHistory());
	}
	
	
	

	// ==============================================================
	// METHODS FOR CREATING NEW INTENTIONS
	// ==============================================================
	


	/**
	 * Create a simple (intentional) observation with a unique alternative with p = 1.0
	 * 
	 * @param s the string describing the observation
	 * @return the constructed observation
	 * @throws DialogueException if formatting problem
	 */
	public static CommunicativeIntention createSimpleCommunicativeIntention (String s) throws DialogueException {
		return createSimpleCommunicativeIntention(s, 1.0f);
	}
	
	/**
	 * Create a simple (intentional) observation with a unique alternative with p = f
	 * 
	 * @param s the string describing the observation
	 * @param f the probability for the description
	 * @return the constructed observation
	 * @throws DialogueException if formatting problem
	 */
	public static CommunicativeIntention createSimpleCommunicativeIntention (String s, float f) throws DialogueException {
		
		dFormula postCondition = FormulaUtils.constructFormula(s);
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(postCondition, EpistemicObjectUtils.robotAgent, f);
			
		return new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));
		
	}
	
	
	/**
	 * Extract the intentional content of a formula with two modal operators <pre> and <post>
	 * 
	 * @param fullFormula the formula containing the two modal formulae
	 * @param agents the agents of the intentional content
	 * @param prob the probability
	 * @return the resulting intentional content
	 */
	public static IntentionalContent createIntentionalContent (dFormula fullFormula, List<String> agents, float prob) {
		
		dFormula precondition = new UnknownFormula(0);
		dFormula postcondition = fullFormula;
		
		if (postcondition instanceof ModalFormula) {
			postcondition = new ComplexFormula(0, Arrays.asList(postcondition), BinaryOp.conj);
		}
		
		if (fullFormula instanceof ComplexFormula) {
			
			for (dFormula subFormula : ((ComplexFormula)fullFormula).forms) {
				
				if (subFormula instanceof ModalFormula && ((ModalFormula)subFormula).op.equals("pre")) {
					precondition = ((ModalFormula)subFormula).form;
				}
				if (subFormula instanceof ModalFormula && ((ModalFormula)subFormula).op.equals("post")) {
					postcondition = ((ModalFormula)subFormula).form;
				}
			} 
		}
		return new IntentionalContent (agents, precondition, postcondition, prob);
	}
	
	
	// ==============================================================
	// METHODS FOR COPYING INTENTIONS
	// ==============================================================

	
	/**
	 * Create a copy of the communicative intention
	 * 
	 * @param initCI the comm. intention to copy
	 * @return the copied communicative intention
	 * @throws DialogueException if something went wrong in the copy
	 */
	public static CommunicativeIntention copy (CommunicativeIntention initCI) throws DialogueException {
		return new CommunicativeIntention(copy(initCI.intent))	;	
	}
	
	
	/**
	 * Create a copy of the intention
	 * 
	 * @param initIntention the intention to copy
	 * @return the copied intention
	 * @throws DialogueException
	 */
	public static Intention copy (Intention initIntention) throws DialogueException {
		
		List<IntentionalContent> newContent = new LinkedList<IntentionalContent>();
		for (IntentionalContent existingIntent : initIntention.content) {
			dFormula copiedPreconditions = FormulaUtils.copy(existingIntent.preconditions);
			dFormula copiedPostconditions = FormulaUtils.copy(existingIntent.postconditions);
			newContent.add(new IntentionalContent (existingIntent.agents, copiedPreconditions,copiedPostconditions, existingIntent.probValue));
		}
		return new Intention(initIntention.frame, initIntention.estatus, initIntention.id, initIntention.content);
	}
	
	
	
	
	
	// ==============================================================
	// EXTRACTING THE CONTENT OF A BELIEF
	// ==============================================================

	
	/**
	 * Extract the content of the belief and express it as a single ComplexFormula
	 * 
	 * NOTE: ugly code...
	 * 
	 * @param b the belief
	 * @return the complex formula representing the belief
	 * @throws DialogueException
	 */
	public static HashMap<ComplexFormula,Float> getBeliefContent 
	(dBelief b, List<String> featuresToExtract) throws DialogueException {
		
		HashMap<ComplexFormula,Float> results = new HashMap<ComplexFormula,Float>();
		
		debug("type of distrib: " + b.content.getClass().getCanonicalName());

		if (b.content instanceof CondIndependentDistribs) {

			results.put(new ComplexFormula(0,new LinkedList<dFormula>(), BinaryOp.conj), 1.0f);

			for (String key : ((CondIndependentDistribs)b.content).distribs.keySet()) {
				debug("key: " + key);
				if (featuresToExtract.contains(key) && 
						((CondIndependentDistribs)b.content).distribs.get(key) instanceof BasicProbDistribution) {

					debug("type of values: " + 
							((BasicProbDistribution)((CondIndependentDistribs)b.content).distribs.get(key)).values);
					if (((BasicProbDistribution)((CondIndependentDistribs)b.content).
							distribs.get(key)).values instanceof FormulaValues) {

						if ((((FormulaValues)((BasicProbDistribution)((CondIndependentDistribs)
								b.content).distribs.get(key)).values).values.size() > 0)) {

							HashMap<ComplexFormula,Float> newResults = new HashMap<ComplexFormula,Float>();
							
							for (ComplexFormula existingFormula : results.keySet()) {
								float curProb = results.get(existingFormula);

								for (FormulaProbPair pair: (((FormulaValues)((BasicProbDistribution)((CondIndependentDistribs)
										b.content).distribs.get(key)).values).values)) {

									dFormula newFormula = FormulaUtils.copy(existingFormula);
									((ComplexFormula)newFormula).forms.add(new ModalFormula(0, key, pair.val));
									debug("adding formula: " + FormulaUtils.getString(newFormula));
									newResults.put((ComplexFormula)newFormula, new Float(curProb*pair.prob));

								}
							}
							results = newResults;
						}

					}
				}
			}	
		}
				
		else if (b.content instanceof BasicProbDistribution) {
						
			if  (((BasicProbDistribution)b.content).values instanceof FormulaValues) {
				for (FormulaProbPair pair: ((FormulaValues)((BasicProbDistribution)b.content).values).values) {
					if (pair.val instanceof ComplexFormula) {
						results.put((ComplexFormula)pair.val, pair.prob);
					}
					else {
						results.put(new ComplexFormula(0,Arrays.asList(pair.val), BinaryOp.conj), pair.prob);
					}
				}
			}
		}
		else {
			throw new DialogueException("WARNING: belief content of " + b.id + " could not be extracted");
		}
		
		
		debug("final created formulae: ");
		for (ComplexFormula existingFormula : results.keySet()) {
			debug("form: " + FormulaUtils.getString(existingFormula) + ": " + results.get(existingFormula));
		}
		
		return results;
	}
	
 
	
	// ==============================================================
	// OTHER METHODS
	// ==============================================================

	

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
	 * Transform an intentional content into a single formula, with two modal
	 * formulae <pre> and <post>
	 * 
	 * @param content the intentional content
	 * @return the resulting formula
	 */
	public static dFormula translateIntoFormula (IntentionalContent content) {
		
		if (content.preconditions != null && !(content.preconditions instanceof UnknownFormula) &&
				!(content.preconditions instanceof ComplexFormula && 
						((ComplexFormula)content.preconditions).forms.size() == 0)) {
			List<dFormula> formulae = new LinkedList<dFormula>();
			formulae.add(new ModalFormula(0, "pre", content.preconditions));
			formulae.add(new ModalFormula(0, "post", content.postconditions));
			return new ComplexFormula(0,formulae, BinaryOp.conj);
		}
		else {
			return content.postconditions;
		}
		
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
 * @author Pierre Lison (plison@ifi.uio.no)
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
 * @author Pierre Lison (plison@ifi.uio.no)
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
