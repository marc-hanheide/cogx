//
//  ContinualCollaborativeActivity.java
//  
//
//  Created by Geert-Jan Kruijff on 9/23/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

// ---------------------------------------------------------
// PACKAGE
// ---------------------------------------------------------

package comsys.processing.cca;

// ---------------------------------------------------------
// BELIEFMODEL imports
// ---------------------------------------------------------


// ---------------------------------------------------------
// COMSYS imports
// ---------------------------------------------------------

import Abducer.*;

import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import comsys.datastructs.comsysEssentials.BoundReadings;
import comsys.datastructs.comsysEssentials.ProofBlock;
import comsys.datastructs.comsysEssentials.RefBinding;
import comsys.datastructs.comsysEssentials.Anchor;
import comsys.datastructs.comsysEssentials.ReadingBindings;
import comsys.datastructs.lf.LogicalForm; 
import comsys.lf.utils.ArrayIterator;

import comsys.processing.cca.AbducerUtils;
import comsys.processing.cca.StackUtils;

// ---------------------------------------------------------
// CAST imports
// ---------------------------------------------------------

import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

// ---------------------------------------------------------
// JAVA imports
// ---------------------------------------------------------

import java.lang.*;
import java.util.*;

// ----------------------------------------------------------
// The class <b>ContinualCollaborativeActivity</b> implements
// a continual algorithm for managing dialogue as a 
// collaborative activity. 
// 
// @author	Geert-Jan M. Kruijff (gj@dfki.de)
// @started	090923
// @version 090923
// ----------------------------------------------------------

public class ContinualCollaborativeActivity {

	/** Activity modes */
	
	public static final String UNDERSTAND = "uttered";
	public static final String GENERATE   = "produce";
	
	/** The abduction engine */ 
	
    private AbducerServerPrx abducer;

	/** Ice engine */
	
    private Ice.Communicator ic;
	
	/** path names to abduction rules and facts; class provides get-/set-methods */ 
	
    private String rulesFilename = "/dev/null";
    private String factsFilename = "/dev/null";	

    private ProofBlock[] stack = new ProofBlock[0];
	
	boolean logging = true;
	
	
	/** 
	 the constructor initializes the abduction server. 
	*/ 
	
	public ContinualCollaborativeActivity () { 
		init();
	} // end constructor
	
	private void init () { 
    	// connect to the server
        log("connecting to the server");
        try { 
            ic = Ice.Util.initialize(); 
            Ice.ObjectPrx base = ic.stringToProxy("AbducerServer:default -p 10000"); 
            abducer = AbducerServerPrxHelper.checkedCast(base); 
            if (abducer == null)
            	throw new Error("Invalid proxy"); 
        }
        catch (Ice.LocalException e) { 
            e.printStackTrace(); 
        }
        catch (Exception e) { 
            System.err.println(e.getMessage()); 
        }	
	} // end init

	protected void finalize() {
        if (ic != null) { 
        	try { 
        		ic.destroy(); 
        	}
        	catch (Exception e) { 
        		System.err.println(e.getMessage()); 
        	} 
        }
	} // end finalize

	/**
	 the method initializes the abduction engine. before this method is called, set-methods can 
	 be called to set which facts- and rules-files to load. 
	 */ 
	 
	public void initAbducer() {
		try {
			abducer.clearFacts();
			abducer.clearRules();
			abducer.clearAssumables();
			abducer.loadFactsFromFile(factsFilename);
			abducer.loadRulesFromFile(rulesFilename);
		}
		catch (AbducerException e) {
			System.err.println(e.message);
		}
	} // end 
	
	/** returns the filename for the facts file for the abducer */
	public String getFactsFileName () { return factsFilename; } 	

	/** returns the filename for the rules file for the abducer */	
	public String getRulesFileName () { return rulesFilename; }

	/** sets the filename for the facts file for the abducer */	
	public void setFactsFileName (String fn) { factsFilename = fn; }
	
	/** sets the filename for the rules file for the abducer */		
	public void setRulesFileName (String fn) { rulesFilename = fn; } 
	
	/**
	 constructProof constructs an abductive proof for a given 
	 logical form. 	 
	 
	 @param activityMode The mode in which a proof is constructed
	 @param lf The logical form for which a proof is constructed
	 @return The abductive proof
	 */ 
	
	public MarkedQuery[] constructProof (String activityMode, LogicalForm lf) {
		Abducer.UnsolvedQuery goal = new Abducer.UnsolvedQuery();
		goal.mark = Abducer.Marking.Unsolved;

		goal.body = AbducerUtils.modalisedFormula(
				new Modality[] {
					AbducerUtils.eventModality()
				},
				AbducerUtils.predicate(activityMode, new Term[] {
					AbducerUtils.term("h"),
					AbducerUtils.term(lf.root.nomVar)
				}));

		goal.isConst = false;
		goal.costFunction = "true";
		goal.constCost = 0.0f;

		MarkedQuery[] goals = new MarkedQuery[] {goal};
		
		String listGoalsStr = "";
		for (int i = 0; i < goals.length; i++) {
			listGoalsStr += MercuryUtils.modalisedFormulaToString(goals[0].body);
			if (i < goals.length - 1) listGoalsStr += ", ";
		}
		log("proving: [" + listGoalsStr + "]");
		
		ProveResult result = abducer.prove(goals);
		if (result == Abducer.ProveResult.ProofFound) {
			log("seems we've got a proof");
			try {
				MarkedQuery[] p = abducer.getBestProof();
				return p;
			}
			catch (NoProofException e) {
				e.printStackTrace();
				return null;
			}
		} else { 
			return null; 
		} // end if.. else
	} // end method
	
	/**
	 * Add the logical form to the set of abducer's facts.
	 * 
	 * @param lf the logical form
	 */
	public void addFactualContext (LogicalForm lf) {
		ModalisedFormula[] facts = AbducerUtils.lfToFacts(new Modality[] {AbducerUtils.infoModality()}, lf);
		for (int i = 0; i < facts.length; i++) {
			abducer.addFact(facts[i]);
			log("add fact: " + MercuryUtils.modalisedFormulaToString(facts[i]));
		}
	}
	
	/**
	 * Add information about anchoring of restrictive references to the abducer.
	 * This instantiates a assumability function in the abducer based on different readings of the LF and
	 * their probability.
	 * 
	 * @param boundReadings anchoring information
	 */
	public void addAnchoringContext (BoundReadings boundReadings) {			
		for (ArrayIterator readingsIter = new ArrayIterator(boundReadings.bindings); readingsIter.hasNext(); ) { 
			// get all the bindings for one reading
			ReadingBindings readingBindings = (ReadingBindings) readingsIter.next();
			// cycle over the bindings for the individual nominals, within this reading
			for (ArrayIterator bindingsIter = new ArrayIterator(readingBindings.bindings); bindingsIter.hasNext(); ) { 
				RefBinding binding = (RefBinding) bindingsIter.next(); 
				// the binding(s) are for the following nominal variable: 
				String nomVar = binding.nomVar; 
				// cycle over the anchorings for this nominal, ordered by costs -- we have all, not just the maximum
				for (ArrayIterator anchorings = new ArrayIterator(binding.antecedents); anchorings.hasNext(); ) { 
					Anchor anchor = (Anchor) anchorings.next();
					// now we can get the entityID, and the float -- i.e. the cost of this one being bound / anchored
					float cost = 1.5f - anchor.probExists;

					ModalisedFormula f = AbducerUtils.modalisedFormula(
							new Modality[] {
								AbducerUtils.attStateModality()
							},
							AbducerUtils.predicate("refers_to", new Term[] {
								AbducerUtils.term(nomVar),
								AbducerUtils.term(anchor.entityID)
							}));			
					abducer.addAssumable("whatif_binding", f, cost);
					
					log("add assumable: " + MercuryUtils.modalisedFormulaToString(f)
							+ " / whatif_binding = "
							+ new Float(cost).toString());
					
				} // end for over the antecedents for the nominal variable
			} // end for over bindings for an individual reading
		} // end for over readings 		
		
	} // end method
	
	/**
	 * Perform verifiable update.
	 * 
	 * @param proof the proof to be considered
	 * @param model current belief model
	 * @return beliefs that are consistent with the belief model, and with which this model needs to be updated
	 */
	public Belief[] verifiableUpdate(MarkedQuery[] proof, BeliefModel model) {
		ProofBlock pi = StackUtils.construct(proof, "");
		ProofBlock piPrime = null;
		ArrayList<Belief> consistentUpdates = new ArrayList<Belief>();
		boolean verified;

		if (StackUtils.isEmpty(stack)) {
			StackUtils.push(stack, pi);
			return new Belief[0];
		}
		else {
			piPrime = StackUtils.pop(stack);
		}
		verified = true;
		
		for (int i = 0; i < piPrime.assertions.length; i++) {
			switch (VerifiableUpdate.consistent(model, piPrime.assertions[i], pi.assumptions)) {
			
				case Consistent:
					// assertion verified -> change its continual status to "proposition"
					for (int j = 0; j < pi.assumptions.length; j++) {
						consistentUpdates.add(pi.assumptions[j]);
					}
					break;
				
				case Inconsistent:
					// assertion falsified
					verified = false;
					// TODO: look here
					break;					
			}
		}
		
		StackUtils.push(stack, pi);
		if (verified == false) {
			StackUtils.push(stack, piPrime);
		}
		
		return consistentUpdates.toArray(new Belief[0]);
	}
	
	private void log(String str) {
		if (logging)
			System.out.println("\033[32m[CCA]\t" + str  + "\033[0m");
	}
	
	
	
	
	
	
	
	
	
	
		

} // end class
