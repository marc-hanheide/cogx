package binder.abstr;


import java.util.Vector;

import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UnionRefProperty;
import binder.utils.BinderUtils;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;


/**
 * Interface for retrieving and updating belief models on the binder working memory
 * 
 * @author Pierre Lison
 * @version 06/10/2009
 * @started 05/10/2009
 */

public class BeliefModelInterface extends ManagedComponent{

	// The current belief model
	BeliefModel currentBeliefModel;
	
	
	/**
	 * Apply a change filter on belief model
	 * 
	 */
	@Override
	public void start() {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(BeliefModel.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					currentBeliefModel = getMemoryEntry(_wmc.address, BeliefModel.class);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
	}
	
	 
	/**
	 * Retrieve the current belief model
	 * 
	 * @return the belief model
	 */
	
	public BeliefModel getCurrentBeliefModel ()  {
		return currentBeliefModel;
	}
	
	/**
	 * Add a new belief in the belief model, and update the working memory
	 * 
	 * @pre no belief with the same id has already been inserted into the WM
	 *		(if precondition is not satisfied, an exception is thrown)
	 * 
	 * @param belief the new belief to insert
	 * 
	 */
	 
	public void addNewBelief (Belief belief) {
		
		String[] newBeliefSet = new String[currentBeliefModel.k.length + 1];
		
		for (int i = 0; i < currentBeliefModel.k.length ; i++) {
			newBeliefSet[i] = currentBeliefModel.k[i];
		}
		newBeliefSet[currentBeliefModel.k.length] = belief.id;
		
		try {
			addToWorkingMemory(belief.id, BinderUtils.BINDER_SA, belief);
			overwriteWorkingMemory(currentBeliefModel.id, BinderUtils.BINDER_SA, currentBeliefModel);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Update an existing belief in the working memory
	 * 
	 * @pre the identifier in both beliefs must be the same (else, exception is thrown)
	 * 
	 * @param belief the new belief
	 */
	
	public void updateExistingBelief (Belief belief) {
		try {
			if (existsOnWorkingMemory(belief.id, BinderUtils.BINDER_SA)) {
				overwriteWorkingMemory(belief.id, BinderUtils.BINDER_SA, belief);
			}
			else {
				log("WARNING: belief not currently in working memory");
				addToWorkingMemory(belief.id, BinderUtils.BINDER_SA, belief);
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	/**
	 * Remove a belief from the working memory, based on its id
	 * 
	 * @param beliefID the identifier of the belief
	 */
	
	public void removeBelief (String beliefID) {
		try {
			deleteFromWorkingMemory(beliefID, BinderUtils.BINDER_SA);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Merge a formula into an existing belief, and update the working memory
	 * 
	 * @param initBelief the initial belief
	 * @param formula the formula to add to the existing belief
	 * 
	 */
	public void mergeFormulaIntoBelief (Belief initBelief, SuperFormula formula) {
		
		Belief newBelief = new Belief();
		newBelief.ags = initBelief.ags;
		newBelief.id = initBelief.id;
		newBelief.sigma = initBelief.sigma;
		
		if (initBelief.phi instanceof ComplexFormula) {
			Vector<SuperFormula> formulae = new Vector<SuperFormula>();
			
			for (int i = 0; i < ((ComplexFormula)initBelief.phi).formulae.length; i++) {
				formulae.add(((ComplexFormula)initBelief.phi).formulae[i]);
			}
			
			if (formula instanceof ComplexFormula) {
			for (int i = 0; i < ((ComplexFormula)formula).formulae.length ; i++) {
				if (!formulae.contains(((ComplexFormula)formula).formulae[i])) {
					formulae.add(((ComplexFormula)formula).formulae[i]);
				}
			}
			}
			else {
				if (!formulae.contains(formula)) {
					formulae.add(formula);
				}
			}
			
			newBelief.phi = new ComplexFormula();
			((ComplexFormula)newBelief.phi).formulae = new SuperFormula[formulae.size()];
			((ComplexFormula)newBelief.phi).formulae = formulae.toArray(((ComplexFormula)newBelief.phi).formulae);
			((ComplexFormula)newBelief.phi).op = ((ComplexFormula)initBelief.phi).op;
			((ComplexFormula)newBelief.phi).prob = ((ComplexFormula)newBelief.phi).prob;
			newBelief.phi.id = initBelief.phi.id;
		}
		
		updateExistingBelief(newBelief);
		
	}
	
	
	/**
	 * Check if the belief if referring to the union entity ID
	 * 
	 * @param b the belief
	 * @param entityID the entity ID
	 * @return true if the reference is found, false otherwise
	 */
	
	private boolean isBeliefReferringToUnion (Belief b, String entityID) {
		
		if (b.phi instanceof ComplexFormula) {
			
			for (int i = 0; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula formula = ((ComplexFormula)b.phi).formulae[i];
				
				if (formula instanceof UnionRefProperty) {
					String unionRef = ((UnionRefProperty)formula).unionRef;
					if (unionRef.equals(entityID)) {
						return true;
					}
				}
			}
		}
		return false;
	}
 	
	/**
	 * Get the set of all beliefs in the working memory which refer to the
	 * given union entity ID
	 * 
	 * @param entityID the identifier of the union
	 * @return the set of related beliefs
	 * 
	 */
	
	public Vector<Belief> getBeliefsByUnionEntityId (String entityID) {
		
		Vector<Belief> relatedBeliefs = new Vector<Belief>();
		try {
		for (int i = 0 ; i < currentBeliefModel.k.length ; i++) {
			Belief b = getMemoryEntry(currentBeliefModel.k[i], BinderUtils.BINDER_SA, Belief.class);
			if (isBeliefReferringToUnion(b, entityID)) {
				relatedBeliefs.add(b);
			}
		}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return relatedBeliefs;
	 }

	public Belief getBelief(String beliefId) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		for (int i = 0 ; i < currentBeliefModel.k.length ; i++) {
			Belief b = getMemoryEntry(currentBeliefModel.k[i], BinderUtils.BINDER_SA, Belief.class);
			if (b.id.equals(beliefId)) {
				return b;
			}
		}
		return null;
	}
	
}
