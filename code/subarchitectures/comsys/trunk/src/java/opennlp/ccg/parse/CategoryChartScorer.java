//=================================================================
//Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION 
//=================================================================

package opennlp.ccg.parse;

//=================================================================
//IMPORTS
//=================================================================

import opennlp.ccg.unify.FeatureStructure;

//-----------------------------------------------------------------
//JAVA
//-----------------------------------------------------------------
import java.util.Iterator;
import java.util.Vector;

//-----------------------------------------------------------------
//OPENCCG
//-----------------------------------------------------------------
import opennlp.ccg.synsem.*;
import opennlp.ccg.unify.*;

/**
	The class <b>CategoryChartScorer</b> implements a basic 
	pruning strategy on charts. It looks at the SignHash in the current 
	top cell (given the string position) and prunes Signs from the hash, 
	given their categorical structure. Given the list of pruned Signs, 
	we then prune the chart down to the lexical entries that gave rise
	to these (possibly composite) Signs. We use the DerivationHistory
	of the Sign for this. We remove all signs that appear in the derivations of 
	unwanted signs, and which are in sign hashes in the rightmost column
	in the chart (i.e. the stringPos column). (The only time we do not do this is when we
	are already at the end of the utterance. In this case we only prune
	the top-most cell.)

	@version 080703 
	@started 070914
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
 */ 

public class CategoryChartScorer 
implements ChartScorer
{

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	/** The beam width function */ 
	BeamWidthFunction beamWidthFunction; 

	/** Logging boolean */ 
	boolean logging;

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	public CategoryChartScorer () { 
		init();
	} // end constr


	protected void init () { 
		beamWidthFunction = null; 
		logging = false; 
	} // end init

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** Sets the beam width function to be used in controlling the number of prefered analyses included in the resulting top-most SignHash */

	public void setBeamWidthFunction (BeamWidthFunction bf) { 
		beamWidthFunction = bf;
	} // end setBeamWidthFunction

	/** Sets whether the method should be outputting log statements */

	public void setLogging(boolean l) { logging = l; }

	//=================================================================
	// COMPUTATION METHODS
	//=================================================================

	/** Returns a Object[] array with a SignHash (element 0) and a list of deleted signs (element1), 
		based on a simple heuristic filtering on categories:
		<ul> 
		<li> At the beginning of the utterance (currentPosition=1), 
			 filter out all incomplete categories with active
			 leftwards slashes, or infinitives (sinf) </li>
		<li> In the middle of the utterance, filter out all 
			 incomplete categories with active leftward slashes.</li>
		<li> At the end of the utterance (completeness=true), filter
			 out all incomplete categories </li>
		</ul> 


		@param hash The hash with signs to be considered
		@param completeness Whether signs are supposed to be complete (indicated end-of-utterance)
		@return Object[] The filtered hash, and the vector of deleted signs
	 */ 

	public Object[] score (SignHash hash, int currentPosition, boolean completeness) { 
		log("Scoring on currentPosition ["+currentPosition+"] with completeness ["+completeness+"]");
		Object[] returnResult = new Object[2];
		Vector<Sign> signsToRemove = new Vector<Sign>();
		SignHash result = new SignHash();
		Slash leftSlash = new Slash('\\');		
		SignPreferenceOrder ordering = new SignPreferenceOrder();
		// check whether to do anything it all, 
		// we do not iterate over cats globally, just to save time!
		if (currentPosition == 0 && !completeness) { 
			Iterator signsIter = hash.asSignSet().iterator();
			log("Going into 0 loop");

			while (signsIter.hasNext()) {  
				Sign sign = (Sign)signsIter.next();
				Category cat = sign.getCategory();
				TargetCat target = cat.getTarget();
				log("Got the category and target of the sign: ["+sign.toString()+"]");

				if (!((AtomCat)target).getType().equals("sinf")) { 
					log("Getting the class");
					String catClass = cat.getClass().getName(); 
					if (catClass.indexOf("ComplexCat") != -1) { 
						log("Complex category");					
						ComplexCat category = (ComplexCat) cat;   
						// Cycle through the argument stack, any 
						// Sign with one or more left-ward arguments are removed. 
						int numArgs = category.getArgStack().size();
						boolean keep = true;
						for (int argsIter = 0; argsIter < numArgs-1 ; argsIter++) { 
							// get the argument
							Arg currArg = category.getArgStack().get(argsIter);
							if (currArg.getClass().getName().indexOf("BasicArg") != -1) {
								Slash slash = ((BasicArg)currArg).getSlash();
								if (slash.equals(leftSlash)) {
									keep = false; 
								} // end check for last argument being a leftward argument
							} // end if.. check for basic arguments						
						} // end for over arguments
						if (keep) { 
							log("Add to signhash");					
							ordering.add(sign);							
						} else {
							log("Prune from signhash");													
							signsToRemove.addElement(sign);						
						} // end if..check for pruning
					} else {
						log("Add to signhash");										
						ordering.add(sign);					
					} 
				} // end if.. check for anything but an infinite

			} // end while over signs
			result = beamWidthFunction.restrict(result,ordering); 
		} else if (completeness) { 
			log("Pruning: completeness is true");
			Iterator signsIter = hash.asSignSet().iterator();
			while (signsIter.hasNext()) { 
				Sign sign = (Sign)signsIter.next();
				Category cat = sign.getCategory();
				TargetCat target = cat.getTarget();
				String catClass = cat.getClass().getName();

				if (catClass.indexOf("AtomCat") != -1) {
					log("!!! Looking at target category ["+target.toString()+"]");
					log("!!! Type of target: ["+((AtomCat)target).getType()+"]");
					FeatureStructure catFS = target.getFeatureStructure();					
					log("!!! Feature structure MOOD value: ["+catFS.getValue("MOOD")+"]");
					if (((AtomCat)target).getType().equals("s")) { 
						log("Looking at S target ==> removing");

						signsToRemove.addElement(sign);
					} 

					// ADDED BY PLISON
					else if (((AtomCat)target).getType().equals("du")) {
							ordering.add(sign);
							log("well-formed discourse unit, is kept");
					}

					else { 
						log("non-S category target, so add");
						signsToRemove.addElement(sign);
						//	ordering.add(sign); 
					} // end if..else
				} else { 
					signsToRemove.addElement(sign);
				}// end if.. check for complete category
			} // end while over signs
			result = beamWidthFunction.restrict(result,ordering); 	
		} else {
			log("Pruning in the middle of the utterance");
			Iterator signsIter = hash.asSignSet().iterator();
			while (signsIter.hasNext()) { 
				Sign sign = (Sign)signsIter.next();
				Category cat = sign.getCategory();
				TargetCat target = cat.getTarget();
				String catClass = cat.getClass().getName(); 
				if (catClass.indexOf("ComplexCat") != -1) { 
					ComplexCat category = (ComplexCat) cat; 
					// get the last argument
					Arg lastArg = category.getArgStack().getLast();
					if (lastArg.getClass().getName().indexOf("BasicArg") != -1) {
						Slash slash = ((BasicArg)lastArg).getSlash();
						if (!slash.equals(leftSlash)) {
							ordering.add(sign);
						} else {
							signsToRemove.addElement(sign);
							log("Remove sign: "+sign.toString()+"\n"+sign.getDerivationHistory().toString());
						} // end check for last argument being a leftward argument
					} // end if.. check for basic arguments
				} else {
					ordering.add(sign);
				} // end if..else check for complex categories
			} // end while over signs
			result = beamWidthFunction.restrict(result,ordering); 			
		} // end 
		log("Size of the returned resulting sign hash: "+result.size());
		log("Size of the vector with signs to be removed: "+signsToRemove.size());		
		returnResult[0] = result;
		returnResult[1] = signsToRemove;
		return returnResult;
	} // end score

	/** The method <i>score</i> returns a Chart, based on a scoring over the Signs in the top-most 
		SignHash given the string position. 

		@param chart The chart to be scored / pruned
		@param stringPos The current string position
		@return Chart The pruned chart
	 */ 

	public Chart score (Chart chart, int stringPos) { 
		// Initialize the return result
		Chart result = chart; 
		// Obtain the pruning results: the sign hash (0), and the signs to be removed (1) 
		Object[] pruningResults = this.score(result.getSigns(0,stringPos), stringPos, (stringPos == (chart._size-1))); 
		// Get the vector with top cell signs to be removed
		Vector signsToRemove = new Vector();
		if (pruningResults[1] != null) signsToRemove = (Vector) pruningResults[1];





		// Get the signhash, and check whether there's actually any signs in it;
		// we only update (and further prune) the chart with non-empty sign hashes, 
		// if we pruned down to 0 we just leave the sign hash (at the moment no further 
		// indication of a problem). 
		SignHash prunedSH =  (SignHash)pruningResults[0]; 
		// Initialize the vector with signs to be pruned, 
		// as figuring in the derivations of the top-cell signs to be removed		
		Vector derivSignsToRemove = new Vector();		
		log("Size of the pruned sign hash: "+prunedSH.size());


		/**
		if (prunedSH.size() > 0) { 
			// Update the top cell at the current position with the pruned SignHash
			result.set(0, stringPos,prunedSH);
			// Iterate over the signs
			Iterator rmSignsIter = signsToRemove.iterator();
			while (rmSignsIter.hasNext()) { 
				Sign rmSign = (Sign)rmSignsIter.next();

				DerivationHistory history = rmSign.getDerivationHistory();
				// Check for the history complexity. If it is zero, then the Sign has no derivation history (i.e. is lexical), 
				// effectively meaning it has already been pruned from the SignHash. Only if the complexity is non-zero we need
				// to prune on the basis of derivation history. 
				log("Derivation history complexity "+history.complexity()+" for sign "+rmSign.toString()); 

				if (history.complexity() > 0) { 
					Sign[] inputs = history.getInputs();
					if (inputs != null) { 	
						log("Sign to be pruned: "+rmSign.toString()+"\n"
							+"Derivation history to be pruned: \n"+history.toString());
						// Hypothesis: we will only ever need to prune signs at the frontier, i.e. the column under the topcell,
						// at the string position ... gather all the signs to be removed, by descending over the derivation histories
						// of the signs to be pruned. 

						// for (int inpC = 0; inpC < inputs.length; inpC++) { 
						//	derivSignsToRemove.addAll(recGetDerivationSigns(inputs[inpC].getDerivationHistory())); 
						// } // end for over input signs

					} // end if.. check for there being inputs
				} else { 
					// the sign is already lexical 
					derivSignsToRemove.addElement(rmSign); 
				} // end check for complexity of derivation history
			} // end if.. check for non-empty topcell
		} // end while

		 */

		// By now we have the list of signs that were involved in the derivations leading up to the signs removed 
		// from the (now pruned) top cell SignHash
		// Cycle over the cells on the right frontier for pruning to the top-cell at the string position, 
		// starting at the lexical cell (stringPos,stringPos) "up" to the top-cell (0,stringPos)

		/**
		log("Derivation history signs to be removed: ");
		Iterator derivSignsIter = derivSignsToRemove.iterator();
		while (derivSignsIter.hasNext()) { Sign s = (Sign) derivSignsIter.next(); log(s.toString()); }
		log("----");
		 */

		/**
		int derivPruneC = stringPos; 
		// for (int derivPruneC = stringPos; derivPruneC > 0; derivPruneC--) { 
			// get the hash in the column at the stringpos
			SignHash hash = result.get(derivPruneC,stringPos); 
			// initialize the vector with the signs to be included in the pruned sign hash
			Vector<Sign> inclSigns = new Vector<Sign>(); 
			// iterate over the signs in the sign hash
			Iterator signsIter = hash.asSignSet().iterator();
			while (signsIter.hasNext()) {
				Sign sign = (Sign) signsIter.next(); 
				if (this.containsSign(derivSignsToRemove,sign)) { 
					log("Removing sign: ["+sign.toString()+"] in cell ["+derivPruneC+","+stringPos+"]");					
				} else { 
					log("NOT removing sign: ["+sign.toString()+"] in cell ["+derivPruneC+","+stringPos+"]");				
					inclSigns.addElement(sign);
				} // end if..else check for sign to remove
			} // end while over signs in the sign hash
			// if the list of signs to be included isn't empty, use it 
			// to prune the sign hash; otherwise use the original sign 
			// hash, for conservative safety reasons ... 
			if (inclSigns.size() > 0) { 
				result.set(derivPruneC,stringPos,new SignHash(inclSigns));
			} // end if.. only use a pruned sign hash if it has signs
		// } // end for over sign hashes to prune for signs

		 */

		result.set(0,stringPos,prunedSH);

		return result; 
	} // end score


	/** The method <i>containsSign</i> checks whether the given sign occurs in the 
		vector (containing signs). The method uses the "equals()" method on signs to 
		check for equality against signs in the vector.  

		@param v The vector with signs
		@param sign The sign which we should check for
		@return boolean Whether the sign occurs in the vector
	 */

	public boolean containsSign (Vector<Sign> v, Sign sign) { 
		boolean found = false;
		Iterator vIter = v.iterator();
		while (vIter.hasNext() && !found) { 
			Sign vSign = (Sign) vIter.next();
			if (sign.equals(vSign)) { found = true; }
		} // end while
		return found;
	} // end containsSign

	/**		
		The method <i>recGetDerivationSigns</i> recursively descends a derivation history, to collect the signs involved in it. 

		@param history The derivation history to be descended down to lexical signs
		@return Vector A vector with signs obtained from the derivation history
	 */ 

	public Vector recGetDerivationSigns (DerivationHistory history) { 
		Vector result = new Vector(); 
		Sign[] inputs = history.getInputs(); 
		if (inputs != null) { 
			// cycle over the input signs, get the signs (possibly recursively)
			for (int inputC = 0; inputC < inputs.length; inputC++) {  
				Sign inputSign = inputs[inputC]; 
				DerivationHistory derivH = inputSign.getDerivationHistory();
				result.addElement(inputSign);
				Vector recResult = recGetDerivationSigns(derivH);
				result.addAll(recResult);
			} // end for
		} else { 
			log("Inputs to derivation history empty: \n "+history.toString());
		} // end	
		return result;
	} // end recGetDerivationSigns

	//=================================================================
	// LOG METHODS
	//=================================================================

	public void log (String msg) { 
		if (logging) System.out.println(">>>>> [CatChartScorer] "+msg); 
	} // end log


} // end class
