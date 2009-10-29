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
import comsys.processing.cca.ProofStack;
import comsys.processing.cca.abduction.ModalityFactory;
import comsys.processing.cca.abduction.PredicateFactory;

import beliefmodels.domainmodel.cogx.*;
import binder.utils.BeliefModelUtils;

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
	public static final String CLARIFY    = "clarify";
	public static final String GENERATE   = "produce";
	
	/** The abduction engine */ 
	
    public AbducerServerPrx abducer;

	/** Ice engine */
	
    private Ice.Communicator ic;
	
	/** path names to abduction rules and facts; class provides get-/set-methods */ 
	
    private Vector<String> files = new Vector<String>();

    public ProofStack stack = new ProofStack();
    
    private Stack<RefBinding> refStack = new Stack<RefBinding>();
	
    private Map<String, Belief> localBeliefs = new HashMap<String, Belief>(); 
    
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
		abducer.clearFacts();
		abducer.clearRules();
		abducer.clearAssumables();
		try {
			Iterator<String> it = files.iterator();
			while (it.hasNext()) {
				abducer.loadFile(it.next());
			}
		}
		catch (AbducerException e) {
			e.printStackTrace();
		}
	} // end 
	
	public void addFileToLoad(String filename) {
		files.add(filename);
	}
		
	/**
	 constructProof constructs an abductive proof for a given 
	 logical form. 	 
	 
	 @param activityMode The mode in which a proof is constructed
	 @param lf The logical form for which a proof is constructed
	 @return The abductive proof
	 */ 
	
	public MarkedQuery[] understandProof (String activityMode, Term nomTerm) {
		Abducer.UnsolvedQuery goal = new Abducer.UnsolvedQuery();
		goal.mark = Abducer.Marking.Unsolved;

		goal.body = AbducerUtils.modalisedFormula(
				new Modality[] {
					ModalityFactory.understandingModality(),
					ModalityFactory.eventModality()
				},
				PredicateFactory.predicate(activityMode, new Term[] {
					PredicateFactory.term("h"),
					nomTerm
					//AbducerUtils.term(lf.root.nomVar)
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
	 * Find the most appropriate feedback to a given event.
	 * @param activityMode
	 * @param nomTerm
	 * @return
	 */
	public MarkedQuery[] generateProof (ModalisedFormula[] evts) {

		UnsolvedQuery[] goals = new UnsolvedQuery[evts.length];
		for (int i = 0; i < evts.length; i++) {
			goals[i] = new UnsolvedQuery();
			goals[i].mark = Abducer.Marking.Unsolved;
			goals[i].body = evts[i];
			goals[i].isConst = false;
			goals[i].costFunction = "true";
			goals[i].constCost = 0.0f;
		}
		
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
		log("expanding LF into facts");
		ModalisedFormula[] facts = AbducerUtils.lfToFacts(new Modality[] {ModalityFactory.infoModality()}, lf);
		for (int i = 0; i < facts.length; i++) {
			abducer.addFact(facts[i]);
			log("  add fact: " + MercuryUtils.modalisedFormulaToString(facts[i]));
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
		log("adding anchoring context");
//		log("got " + boundReadings.bindings.length + " reading(s)");
		for (ArrayIterator readingsIter = new ArrayIterator(boundReadings.bindings); readingsIter.hasNext(); ) { 
			// get all the bindings for one reading
			ReadingBindings readingBindings = (ReadingBindings) readingsIter.next();
			// cycle over the bindings for the individual nominals, within this reading
//			log("got " + readingBindings.bindings.length + " binding(s)");
			
			for (ArrayIterator bindingsIter = new ArrayIterator(readingBindings.bindings); bindingsIter.hasNext(); ) { 
				RefBinding binding = (RefBinding) bindingsIter.next(); 
				// the binding(s) are for the following nominal variable: 
				String nomVar = binding.nomVar; 
				// cycle over the anchorings for this nominal, ordered by costs -- we have all, not just the maximum

				Anchor[] antecedents = new Anchor[0];
				
//				log("got " + binding.antecedents.length + " binding anchor(s)");
				if (binding.antecedents.length == 0) {
					log("! no binding anchors for " + binding.nomVar);
					
					if (refStack.empty()) {
						log("  refStack empty");
						return;
					}
					else {
						if (nomVar.startsWith("it")) {
							log("     the nominal seems to be an \"it\"");
							log("  -> using the last RefBinding");
							antecedents = refStack.peek().antecedents;
						}
						else {
							log("     the nominal doesn't seem to be an \"it\"");
							log("  -> leaving anchor set empty");
						}
						
					}
					
				}
				else {
					// use the bound readings
					antecedents = binding.antecedents;
					
					// and add them to the stack for eventual later use
					refStack.push(binding);
				}
				
				for (ArrayIterator anchorings = new ArrayIterator(antecedents); anchorings.hasNext(); ) { 
					Anchor anchor = (Anchor) anchorings.next();
					float cost = 1.5f - anchor.probExists;

					ModalisedFormula f = AbducerUtils.modalisedFormula(
							new Modality[] {
								ModalityFactory.attStateModality()
							},
							PredicateFactory.predicate("refers_to", new Term[] {
								PredicateFactory.term(nomVar),
								PredicateFactory.term(anchor.entityID)
							}));			
					abducer.addAssumable("ref_resolution", f, cost);
					
					log("  add assumable: " + MercuryUtils.modalisedFormulaToString(f)
							+ " / whatif_binding = "
							+ new Float(cost).toString());
					
				} // end for over the antecedents for the nominal variable
			} // end for over bindings for an individual reading
		} // end for over readings 		
		
	} // end method
	
	public void addCRContext(beliefmodels.clarification.ClarificationRequest cr) {
		Modality[] mod = new Modality[] {ModalityFactory.infoModality()};
		ModalisedFormula mfModality = AbducerUtils.modalisedFormula(mod,
				PredicateFactory.twoPlacePredicate("cr_modality", cr.id, cr.sourceModality));
		ModalisedFormula mfSourceId = AbducerUtils.modalisedFormula(mod,
				PredicateFactory.twoPlacePredicate("cr_entity", cr.id, cr.sourceEntityID));
		abducer.addFact(mfModality);
		abducer.addFact(mfSourceId);
		log("adding fact: " + MercuryUtils.modalisedFormulaToString(mfModality));
		log("adding fact: " + MercuryUtils.modalisedFormulaToString(mfSourceId));

		if (cr.clarificationNeed instanceof ComplexFormula) {
			ComplexFormula cplxF = (ComplexFormula) cr.clarificationNeed;
			log("ignoring the logicalOp for the moment");
			SuperFormula[] needFs = cplxF.formulae;
			for (int i = 0; i < needFs.length; i++) {
				
				if (needFs[i] instanceof ContinualFormula) {
					String[] args = ClarificationUtils.valueTermArgs((ContinualFormula) needFs[i]);

					Term[] tArgs = null;
					if (args.length == 1) {
						tArgs = new Term[] {PredicateFactory.term(cr.id), PredicateFactory.term(args[0])};
					}
					else if (args.length == 2) {
						tArgs = new Term[] {PredicateFactory.term(cr.id), PredicateFactory.term(args[0]), PredicateFactory.term(args[1])};
					}
					ModalisedFormula mfNeed = AbducerUtils.modalisedFormula(mod, PredicateFactory.predicate("cr_need", tArgs));
					
					abducer.addFact(mfNeed);
					log("adding fact: " + MercuryUtils.modalisedFormulaToString(mfNeed));
				}
			}
		}
	}
	
/*
	public String[] findRelevantBeliefs(ProofStack stack, Predicate intention) {
		return null;
	}
*/

	public void printStack() {
		log("printing current stack status, top first (" + stack.blocks.length + " items total):");
		for (int i = 0; i < stack.blocks.length; i++) {
			log(PrettyPrinting.proofBlockToString(stack.blocks[i]));
		}
		log("done printing stack");
	}
	
	private void log(String str) {
		if (logging)
			System.out.println("\033[32m[CCA]\t" + str  + "\033[0m");
	}

} // end class
