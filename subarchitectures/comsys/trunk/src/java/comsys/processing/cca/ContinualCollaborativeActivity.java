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

import beliefmodels.domainmodel.cogx.*;

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
	
    private String understandRulesFilename = "/dev/null";
    private String understandFactsFilename = "/dev/null";
    private String generateRulesFilename = "/dev/null";
    private String generateFactsFilename = "/dev/null";

    private ProofBlock[] stack = new ProofBlock[0];
    
    private Stack<RefBinding> refStack = new Stack<RefBinding>();
	
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
			abducer.loadFactsFromFile(understandFactsFilename);
			abducer.loadFactsFromFile(generateFactsFilename);
			abducer.loadRulesFromFile(understandRulesFilename);
			abducer.loadRulesFromFile(generateRulesFilename);
		}
		catch (AbducerException e) {
			e.printStackTrace();
		}
	} // end 
	
	public void setUnderstandFactsFileName (String fn) { understandFactsFilename = fn; }
	public void setUnderstandRulesFileName (String fn) { understandRulesFilename = fn; }
	public void setGenerateFactsFileName (String fn) { generateFactsFilename = fn; }
	public void setGenerateRulesFileName (String fn) { generateRulesFilename = fn; }
		
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
					AbducerUtils.eventModality()
				},
				AbducerUtils.predicate(activityMode, new Term[] {
					AbducerUtils.term("h"),
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
		ModalisedFormula[] facts = AbducerUtils.lfToFacts(new Modality[] {AbducerUtils.infoModality()}, lf);
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
		for (ArrayIterator readingsIter = new ArrayIterator(boundReadings.bindings); readingsIter.hasNext(); ) { 
			// get all the bindings for one reading
			ReadingBindings readingBindings = (ReadingBindings) readingsIter.next();
			// cycle over the bindings for the individual nominals, within this reading
			for (ArrayIterator bindingsIter = new ArrayIterator(readingBindings.bindings); bindingsIter.hasNext(); ) { 
				RefBinding binding = (RefBinding) bindingsIter.next(); 
				// the binding(s) are for the following nominal variable: 
				String nomVar = binding.nomVar; 
				// cycle over the anchorings for this nominal, ordered by costs -- we have all, not just the maximum

				Anchor[] antecedents = null;
				
				if (binding.antecedents.length == 0) {
					log("! no binding anchors for " + binding.nomVar);
					
					if (refStack.empty()) {
						log("  refStack empty");
						return;
					}
					else {
						log("  -> using the last RefBinding");
						antecedents = refStack.peek().antecedents;
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
								AbducerUtils.attStateModality()
							},
							AbducerUtils.predicate("refers_to", new Term[] {
								AbducerUtils.term(nomVar),
								AbducerUtils.term(anchor.entityID)
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
		Modality[] mod = new Modality[] {AbducerUtils.infoModality()};
		ModalisedFormula mfModality = AbducerUtils.modalisedFormula(mod,
				AbducerUtils.predicate("cr_modality", new Term[] {
					AbducerUtils.term(cr.id),
					AbducerUtils.term(cr.sourceModality)
				}));
		ModalisedFormula mfSourceId = AbducerUtils.modalisedFormula(mod,
				AbducerUtils.predicate("cr_entity", new Term[] {
						AbducerUtils.term(cr.id),
						AbducerUtils.term(cr.sourceEntityID)
					}));
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
					String[] args = valueTermArgs((ContinualFormula) needFs[i]);

					Term[] tArgs = null;
					if (args.length == 1) {
						tArgs = new Term[] {AbducerUtils.term(cr.id), AbducerUtils.term(args[0])};
					}
					else if (args.length == 2) {
						tArgs = new Term[] {AbducerUtils.term(cr.id), AbducerUtils.term(args[0]), AbducerUtils.term(args[1])};
					}
					ModalisedFormula mfNeed = AbducerUtils.modalisedFormula(mod, AbducerUtils.predicate("cr_need", tArgs));
					
					abducer.addFact(mfNeed);
					log("adding fact: " + MercuryUtils.modalisedFormulaToString(mfNeed));
				}
			}
		}
	}
	
	public static FunctionTerm continualFormulaToTerm(ContinualFormula cf) {
		String functor = "";
		String arg = "";
		
		
		if (cf instanceof ObjectTypeProperty) {
			ObjectTypeProperty pf = (ObjectTypeProperty) cf;
			functor = "objecttype";
			arg = pf.typeValue.toString();
		}
		if (cf instanceof ColorProperty) {
			ColorProperty pf = (ColorProperty) cf;
			functor = "color";
			arg = pf.colorValue.toString();
		}
		if (cf instanceof ShapeProperty) {
			ShapeProperty pf = (ShapeProperty) cf;
			functor = "shape";
			arg = pf.shapeValue.toString();
		}
		
		if (!functor.equals("")) {
			return AbducerUtils.term(functor, new Term[] {AbducerUtils.term(arg)});
		}
		else {
			return null;
		}
	}
	
	public String[] valueTermArgs(ContinualFormula cf) {
		String propName = "";
		String propValue = null;
		
		if (cf instanceof ColorProperty) {
			propName = "color";
			Color val = ((ColorProperty)cf).colorValue;
			if (val != Color.unknownColor)
				propValue = val.toString();
		}
		if (cf instanceof ShapeProperty) {
			propName = "shape";
			Shape val = ((ShapeProperty)cf).shapeValue;
			if (val != Shape.unknownShape)
				propValue = val.toString();
		}
		if (cf instanceof ObjectTypeProperty) {
			propName = "objecttype";
			ObjectType val = ((ObjectTypeProperty)cf).typeValue;
			if (val != ObjectType.unknownObjectType)
				propValue = val.toString();
		}
		if (propValue != null) {
			return new String[] {propName, propValue};
		}
		else {
			return new String[] {propName};
		}
	}
	
	/**
	 * Perform verifiable update.
	 * 
	 * @param proof the proof to be considered
	 * @param model current belief model
	 * @return beliefs that are consistent with the belief model, and with which this model needs to be updated
	 */
	public Belief[] verifiableUpdate(MarkedQuery[] proof, BeliefModel model) {
		ProofBlock pi = StackUtils.construct(proof, "proof-X");
		ProofBlock piPrime = null;
		ArrayList<Belief> consistentUpdates = new ArrayList<Belief>();
		boolean verified;

		if (StackUtils.isEmpty(stack)) {
			StackUtils.push(stack, pi);
			//return new Belief[0];
			return pi.assertions;
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
