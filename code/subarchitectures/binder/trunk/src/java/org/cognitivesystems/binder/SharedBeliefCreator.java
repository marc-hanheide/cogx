
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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


package org.cognitivesystems.binder;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Vector;


import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.GenericPointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;


/**
 * Simple CAST component listening to the local working memory for pairs of beliefs
 * which are (1) respectively private and attributed, and (2) contain a non-empty subset
 * of identical features.  If such pair of belief is found, a new shared belief is created 
 * with the subsumed feature(s), pointing to the private belief.
 * 
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 23/08/2010
 * 
 */
public class SharedBeliefCreator extends ManagedComponent {




	/**
	 * Adds two change filters on beliefs
	 */
	@Override
	public void start() {

		// add a filter on newly inserted beliefs
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(dBelief.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(
							WorkingMemoryChange _wmc) {
						try {
							dBelief belief = getMemoryEntry(_wmc.address, dBelief.class);

							// if the belief is attributed, check a possible match with a private belief
							if (belief.estatus instanceof AttributedEpistemicStatus) {
								checkMatchWithPrivateBelief(belief);
							}
						} catch (DoesNotExistOnWMException e) {
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							e.printStackTrace();
						}
					}
				});

		// add a filter on overwritten beliefs
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(dBelief.class,  WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(
							WorkingMemoryChange _wmc) {
						try {
							dBelief belief = getMemoryEntry(_wmc.address, dBelief.class);
							if (belief.estatus instanceof AttributedEpistemicStatus) {
								// if the belief is attributed, check a possible match with a private belief
								checkMatchWithPrivateBelief(belief);
							}
						} catch (DoesNotExistOnWMException e) {
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							e.printStackTrace();
						}
					}
				});	

	}

	
	

	/**
	 * Check whether the attributed belief has any common features with the
	 * private belief it points towards
	 * 
	 * @param belief the attributed belief
	 */
	protected void checkMatchWithPrivateBelief(dBelief attrBelief) {

		debug("checking match with associated private belief...");

		if (!(attrBelief.content instanceof CondIndependentDistribs)) {
			debug("only cond independent distribs are supported at the moment");
			return ;
		}

		// extracting the pointer
		GenericPointerFormula pointer = extractPointer (attrBelief);

		if (pointer != null) {	

			debug("pointer to private belief successfully extracted");

			try {
				// extracting the belief itself
				dBelief privateBelief = getMemoryEntry(pointer.pointer,dBelief.class);

				// checking if the belief is indeed private
				if (privateBelief.estatus instanceof PrivateEpistemicStatus) {

					// comparing the two belief and determining which features are shared
					Vector<BasicProbDistribution> sharedFeats = compareBeliefs (attrBelief, privateBelief);

					if (sharedFeats.size() > 0) {

						// creating a shared belief
						Vector<String> agents = getAgents(attrBelief);
						dBelief sharedBelief = createSharedBelief (agents, pointer, sharedFeats);

						try {
							// if an shared belief already exists, overwrite it
							String existingSharedBeliefID = getExistingSharedBelief(pointer.pointer);
							if (existingSharedBeliefID != null) {
								overwriteWorkingMemory (existingSharedBeliefID, sharedBelief);
							}

							// else, add it into the working memory
							else {
								addToWorkingMemory(newDataID(), sharedBelief);
							}
						} 
						catch (Exception e) {
							e.printStackTrace();
						}
					}
				}
			}
			catch (DoesNotExistOnWMException e) {
				debug("Warning: belief: " + pointer.pointer + " does not seem to exist");
			}
		}
		else {
			debug ("warning, no pointer to a private belief");
		}

	}



	/**
	 * Extract the generic pointer formula contained in the belief, if any 
	 * (else, returns null)
	 * 
	 * @param belief the belief
	 * @return the generic pointer formula if any is provided, null otherwise
	 */
	private GenericPointerFormula extractPointer (dBelief belief) {

		if (!(belief.content instanceof CondIndependentDistribs)) {
			debug("only cond independent distribs are supported at the moment");
			return null;
		}

		CondIndependentDistribs attrContent = (CondIndependentDistribs) belief.content;

		debug("attributed belief contains pointer feature: " + attrContent.distribs.containsKey(POINTERLABEL.value));
		
		if (attrContent.distribs.containsKey(POINTERLABEL.value)) { 
					
			if (attrContent.distribs.get(POINTERLABEL.value) instanceof BasicProbDistribution) {
				DistributionValues pointerVals = ((BasicProbDistribution)attrContent.distribs.get(POINTERLABEL.value)).values;
				dFormula mostLikelyVal = getHighestProbValue(pointerVals);
				
				debug("type of pointer value: " + mostLikelyVal.getClass().getSimpleName());
				
				if (mostLikelyVal instanceof GenericPointerFormula) {
					return(GenericPointerFormula) mostLikelyVal;
				}

			}
		}
		return null;
	}




	/**
	 * Compare two beliefs, and try to find the features which are shared.
	 * The outcome is a vector of features (defined as BasicProbDistributions) 
	 * which are subsumed by both beliefs
	 * 
	 * @param belief1 the first belief
	 * @param belief2 the second belief
	 * @return
	 */
	private Vector<BasicProbDistribution> compareBeliefs (dBelief belief1, dBelief belief2) {

		// initialise the vector
		Vector<BasicProbDistribution> sharedFeatures = new Vector<BasicProbDistribution>();

		// check if the two belief content are cond. independent distributions
		if (belief1.content instanceof CondIndependentDistribs && 
				belief2.content instanceof CondIndependentDistribs) {

			CondIndependentDistribs content1 = (CondIndependentDistribs) belief1.content;
			CondIndependentDistribs content2 = (CondIndependentDistribs) belief2.content;

			// looping on the features of the first belief
			for (String feature : content1.distribs.keySet()) {

				// checking if the second belief also contains the feature
				if (content2.distribs.containsKey(feature) && 
						content1.distribs.get(feature) instanceof BasicProbDistribution &&
						content2.distribs.get(feature) instanceof BasicProbDistribution) {

					// extracting the feature values
					DistributionValues values1 = ((BasicProbDistribution)content1.distribs.get(feature)).values;
					dFormula mostLikelyVal1 = getHighestProbValue (values1);

					DistributionValues values2 = ((BasicProbDistribution)content2.distribs.get(feature)).values;
					dFormula mostLikelyVal2 = getHighestProbValue (values2);

					// and checking if the feature values match
					if (isEqualTo(mostLikelyVal1, mostLikelyVal2)) {
						
						// create a new basic distribution with the subsumed feature
						FormulaValues newValues = new FormulaValues(new LinkedList<FormulaProbPair>());				
						float prob = Math.min(getProbability(values1,mostLikelyVal1), getProbability(values1,mostLikelyVal1));				
						newValues.values.add(new FormulaProbPair(mostLikelyVal1, prob));
						BasicProbDistribution newDistrib = new BasicProbDistribution(feature, newValues);
						
						// and add it to the set
						sharedFeatures.add(newDistrib);
					}						
				}

			}
		}

		return sharedFeatures;
	}


	

	/**
	 * Create a new shared belief given a set of agents, a pointer to the private belief,
	 * and a set of shared features
	 * 
	 * @param agents the set of agents
	 * @param pointer the pointer
	 * @param sharedFeatures the set of shared features
	 * @return a new belief
	 */
	private dBelief createSharedBelief (Vector<String> agents, GenericPointerFormula pointer, 
			Vector<BasicProbDistribution> sharedFeatures) {

		dBelief sharedBelief = new dBelief();

		sharedBelief.estatus = new SharedEpistemicStatus(agents);

		sharedBelief.content = new CondIndependentDistribs();
		((CondIndependentDistribs)sharedBelief.content).distribs = new HashMap<String,ProbDistribution>();	

		for (BasicProbDistribution distrib : sharedFeatures) {
			((CondIndependentDistribs)sharedBelief.content).distribs.put(distrib.key, distrib);
		}

		addPointerFeature(sharedBelief, pointer);
		
		return sharedBelief;
	}
	
	
	
	/**
	 * If an existing shared belief is already associated to the given private
	 * belief, return its memory id
	 * 
	 * @param privBeliefId the memory id of the private belief
	 * @return the memory id if an existing shared belief is found, or null otherwise
	 */
	public String getExistingSharedBelief (String privBeliefId) {
		
		try {
			CASTData<dBelief>[] beliefs = getWorkingMemoryEntries(dBelief.class);
			
			for (int i = 0 ; i < beliefs.length ; i++) {
				
				dBelief curBelief = beliefs[i].getData();
				
				if (curBelief.estatus instanceof SharedEpistemicStatus) {
					
					GenericPointerFormula pointer = extractPointer (curBelief);
					
					if (pointer != null && pointer.pointer.equals(privBeliefId)) {
						return beliefs[i].getID();
					}
				}
			}
			
		} catch (SubarchitectureComponentException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
			
		
		return null;
	}
	
	

	/**
	 * Returns the probability of a formula specified in a FormulaValues
	 * (0.0f otherwise)
	 * 
	 * @param values the values
	 * @param form the formula to search
	 * @return
	 */
	private float getProbability (DistributionValues values, dFormula form) {
		
		if (values instanceof FormulaValues) {
		for (FormulaProbPair pair: ((FormulaValues)values).values) {
			if (pair.val.equals(form)) {
				return pair.prob;
			}
		}
		}
		return 0.0f;
	}
	
	
	
	/**
	 * Extract the agents name (both private and attributed) in a given
	 * attributed belief 
	 * 
	 * @param attrBelief the belief
	 * @return a vector containing the name of all agents
	 */
	private Vector<String> getAgents (dBelief attrBelief) {
		
		Vector<String> agents = new Vector<String>();
		
		if (attrBelief.estatus instanceof AttributedEpistemicStatus) {
			
			agents.add(((AttributedEpistemicStatus)attrBelief.estatus).agent);
			agents.addAll(((AttributedEpistemicStatus)attrBelief.estatus).attribagents);
		}
		
		return agents;
	}
	
	
	 
	/**
	 * Returns true if the content of form1 is equal to the content form2, 
	 * and false otherwise
	 *  
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if contents are equal, false otherwise
	 */
	public static boolean isEqualTo (dFormula form1, dFormula form2)  {
		
		if (form1 instanceof ElementaryFormula && form2 instanceof ElementaryFormula) {
			if (((ElementaryFormula)form1).prop.toLowerCase().equals(((ElementaryFormula)form2).prop.toLowerCase())) {
				return true;
			}
		}
		else if (form1 instanceof ComplexFormula && form2 instanceof ComplexFormula) {
			return compare ((ComplexFormula)form1, (ComplexFormula)form2);
		}
		else if (form1 instanceof ModalFormula && form2 instanceof ModalFormula) {
			return compare ((ModalFormula)form1, (ModalFormula)form2);
		}
		
		else if (form1 instanceof IntegerFormula && form2 instanceof IntegerFormula) {
			return ((IntegerFormula)form1).val == ((IntegerFormula)form2).val;
		}
		
		else if (form1 instanceof FloatFormula && form2 instanceof FloatFormula) {
			return ((FloatFormula)form1).val == ((FloatFormula)form2).val;
		}
		
		else if (form1 instanceof GenericPointerFormula && form2 instanceof GenericPointerFormula) {
			return ((GenericPointerFormula)form1).pointer.equals(((GenericPointerFormula)form2).pointer);
		}
		
		else if (form1 instanceof BooleanFormula && form2 instanceof BooleanFormula) {
			return ((BooleanFormula)form1).val == ((BooleanFormula)form2).val;
		}
		
		else if (form1 instanceof UnderspecifiedFormula || form2 instanceof UnderspecifiedFormula) {
			return true;
		}
		else if (form1 instanceof UnknownFormula && form2 instanceof UnknownFormula) {
			return true;
		}
		
		return form1.toString().equals(form2.toString());
	}
	
	
	/**
	 * returns true if the content of form1 is equal to the content of form2,
	 * false otherwise
	 * 
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if contents are equal, false otherwise
	 */
	private static boolean compare (ModalFormula form1, ModalFormula form2) {
		
		if (!form1.op.equals(form2.op)) {
			return false;
		}
		
		return (isEqualTo(form1.form, form2.form));
	}
	
	/**
	 * returns true if the content of form1 is equal to the content of form2,
	 * false otherwise
	 * 
	 * @param form1 the first formula
	 * @param form2 the second formula
	 * @return true if contents are equal, false otherwise
	 */
	private static boolean compare (ComplexFormula form1, ComplexFormula form2) {
		
		if ((form1.forms.size() != form2.forms.size()) || !(form1.op.equals(form2.op))) {
			return false;
		}
		
		for (dFormula subform1 : form1.forms) {		
			boolean foundMatch = false;
			for (dFormula subform2 : form2.forms) {
				if (isEqualTo(subform1, subform2)) {
					foundMatch = true;
				}
			}
			if (!foundMatch) {
				return false;
			}
		}
		return true;
	}
	
	
	
	/**
	 * Add a new pointer feature to the given belief
	 * 
	 * @param belief the belief
	 * @param pointer the pointer
	 */
	private void addPointerFeature (dBelief belief, GenericPointerFormula pointer) {
		FormulaValues pointerValues = new FormulaValues(new LinkedList<FormulaProbPair>());
		pointerValues.values.add(new FormulaProbPair(pointer, 1.0f));
		BasicProbDistribution pointerDistrib = new BasicProbDistribution (POINTERLABEL.value, pointerValues);
		
		((CondIndependentDistribs)belief.content).distribs.put(POINTERLABEL.value, pointerDistrib);
	}

	
	
	/**
	 * Returns the formula value with the highest probability in the set of 
	 * possible values
	 * 
	 * @param values
	 * @return
	 */
	private dFormula getHighestProbValue (DistributionValues values) {

		if (! (values instanceof FormulaValues)) {
			return null;
		}

		float curMaxProb = 0.0f;
		dFormula bestForm = null;
		for (FormulaProbPair pair : ((FormulaValues)values).values) {
			if (pair.prob > curMaxProb) {
				bestForm = pair.val;
			}
		}

		return bestForm;
	}


}
