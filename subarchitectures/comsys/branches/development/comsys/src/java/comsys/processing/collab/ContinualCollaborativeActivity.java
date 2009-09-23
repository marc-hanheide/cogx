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

package comsys.processing.collab;

// ---------------------------------------------------------
// BELIEFMODEL imports
// ---------------------------------------------------------


// ---------------------------------------------------------
// COMSYS imports
// ---------------------------------------------------------

import Abducer.*;


import comsys.datastructs.comsysEssentials.BoundReadings;
import comsys.datastructs.lf.LogicalForm; 


import comsys.processing.collab.AbdUtils;


// ---------------------------------------------------------
// JAVA imports
// ---------------------------------------------------------



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
	
	static const String UNDERSTAND = "uttered";
	static const String GENERATE   = "produce";
	
	/** The abduction engine */ 
	
    private AbducerServerPrx abducer;

	/** Ice engine */
	
    private Ice.Communicator ic;
	
	/** path names to abduction rules and facts; class provides get-/set-methods */ 
	
    private String rulesFilename = "./subarchitectures/comsys/abduction/rules.txt";
    private String factsFilename = "./subarchitectures/comsys/abduction/facts.txt";	
	
	
	
	
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

	/**
	 the method initializes the abduction engine. before this method is called, set-methods can 
	 be called to set which facts- and rules-files to load. 
	 */ 
	 
	public void initAbducer() {
		abducer.clearFacts();
		abducer.loadFactsFromFile(factsFilename);
		abducer.clearRules();
		abducer.loadRulesFromFile(rulesFilename);	
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
	 @returns AbductiveProof The abductive proof
	 */ 
	
	public AbductiveProof constructProof (String activityMode, LogicalForm lf) {
		AbdUtils.addLFAsExplicitFacts(abducer, lf);
		Abducer.UnsolvedQuery goal = new Abducer.UnsolvedQuery();
		goal.mark = Abducer.Marking.Unsolved;
		goal.body = new ModalisedFormula();
		goal.body.m = new Modality[] {AbdUtils.modEvent()};
		goal.body.p = AbdUtils.predicate(activityMode, new Term[] {
										 AbdUtils.term("h"),
										 AbdUtils.term(slf.lf.root.nomVar)
										 });
		goal.isConst = true;
		goal.costFunction = "";
		goal.constCost = 50.0f;
		
		Abducer.MarkedQuery[] goals = new Abducer.MarkedQuery[] {goal};
		
		log("proving: " + MercuryUtils.modalisedFormulaToString(goals[0].body));
		
		//           	goal.body.termString = LFUtils.lfToMercString(slf.lf);
		
		ProveResult result = abducer.prove(goals);
		//            	ProofResult result = abducer.proveGoal("e(now) : uttered(h, " + LFUtils.lfToMercString(slf.lf) + ").");
		if (result == Abducer.ProveResult.SUCCESS) {
			log("seems we've got a proof");
			AbductiveProof p = abducer.getBestProof();
			return p; 
		} else { 
			return null; 
		} // end if.. else
	} // end method

	
	/** 
	 addAnchoringContext can be called to add (cost) information about anchoring restrictive LF content, to the abducer. this
	 cost information is based on different readings the LF might have. 

	 MIRA DO SOMETHING SMART HERE WITH THE BOUND READINGS	 
	 GJ: provided a loop already	 
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
					float probability = anchor.probExists; 
					
				} // end for over the antecedents for the nominal variable
			} // end for over bindings for an individual reading
		} // end for over readings 		
		
	} // end method
		
	
	
	
	
	
	
	
	
	
	
		

} // end class
