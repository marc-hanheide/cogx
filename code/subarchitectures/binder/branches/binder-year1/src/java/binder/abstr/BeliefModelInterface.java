package binder.abstr;


import java.util.HashMap;
import java.util.Vector;

import beliefmodels.adl.Agent;
import beliefmodels.adl.AgentStatus;
import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import beliefmodels.adl.Formula;
import beliefmodels.adl.Perspective;
import beliefmodels.adl.SpatialInterval;
import beliefmodels.adl.SpatioTemporalFrame;
import beliefmodels.adl.SpatioTemporalModel;
import beliefmodels.adl.TemporalInterval;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.LogicalOp;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UnionRefProperty;
import binder.components.Binder;
import binder.utils.BinderUtils;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


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
					log("current belief model has been updated!");
					currentBeliefModel = getMemoryEntry(_wmc.address, BeliefModel.class);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
	}

	
	

	public final Agent robotAgent = new Agent("robot");
	public final Agent humanAgent = new Agent("human");
	public SpatioTemporalFrame curFrame = new SpatioTemporalFrame();
	
	// =================================================================
	// INITIALISATION
	// =================================================================

 
	public void initializeAgentAndFrame () {
		robotAgent.id = "robot";

		String id = "here-and-now";
		Agent[] ags = new Agent[1];
		ags[0] = robotAgent;
		Perspective persp = new Perspective("robotperspective", ags);
		SpatialInterval spatialint = new SpatialInterval("here");
		TemporalInterval tempint = new TemporalInterval("now", getCASTTime().toString(), getCASTTime().toString());
		
		curFrame = new SpatioTemporalFrame(id, spatialint, tempint, persp);
	}

	/**
	 * Retrieve the current belief model
	 * 
	 * @return the belief model
	 */

	public BeliefModel getCurrentBeliefModel ()  {

		if (currentBeliefModel != null) {
			return currentBeliefModel;
		}
		else {
			SpatioTemporalModel s = new SpatioTemporalModel();		
			s.frames = new SpatioTemporalFrame[1];
			s.frames[0] = curFrame;

			String[] k = new String[0];

			Agent[] a = new Agent[] { robotAgent, humanAgent };
			String[] t = new String[0];
			String[] f = new String[0];

			BeliefModel bmodel = new BeliefModel(newDataID(), a, s, k, t, f);
			return bmodel;
		}
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
		debug("adding new belief: " + belief.id);
		
		if (currentBeliefModel != null) {
		try {
		currentBeliefModel = getMemoryEntry(currentBeliefModel.id, Binder.BINDER_SA, BeliefModel.class);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		}
		String[] newBeliefSet = new String[currentBeliefModel.k.length + 1];
		
		for (int i = 0; i < currentBeliefModel.k.length ; i++) {
			newBeliefSet[i] = currentBeliefModel.k[i];
		}
		newBeliefSet[currentBeliefModel.k.length] = belief.id;
		currentBeliefModel.k = newBeliefSet;
		
		try {
			addToWorkingMemory(belief.id, Binder.BINDER_SA, belief);		
			overwriteWorkingMemory(currentBeliefModel.id, Binder.BINDER_SA, currentBeliefModel);
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
		debug("updating belief: " + belief.id);
		try {
			if (existsOnWorkingMemory(belief.id, Binder.BINDER_SA)) {
				overwriteWorkingMemory(belief.id, Binder.BINDER_SA, belief);
			}
			else {
				log("WARNING: belief not currently in working memory");
				addToWorkingMemory(belief.id, Binder.BINDER_SA, belief);
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
			deleteFromWorkingMemory(beliefID, Binder.BINDER_SA);
			
			// TODO: also update the belief model
			
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
		
		AgentStatus ags = initBelief.ags;
		String id = initBelief.id;
		SpatioTemporalFrame sigma = initBelief.sigma;
		
		SuperFormula phi;

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
			
			SuperFormula[] formulaeArray = new SuperFormula[formulae.size()];
			formulaeArray = formulae.toArray(formulaeArray);
			LogicalOp op = ((ComplexFormula)initBelief.phi).op;
			float prob = ((ComplexFormula)initBelief.phi).prob;
			String phiId = initBelief.phi.id;
			
			phi = new ComplexFormula(phiId, prob, op, formulaeArray);
		}
		else {
			SuperFormula[] formulaeArray = new SuperFormula[2];
			formulaeArray[0] = (SuperFormula) initBelief.phi;
			formulaeArray[1] = formula;
			String phiId = initBelief.phi.id;
			
			phi = new ComplexFormula(phiId, 1.0f, LogicalOp.and, formulaeArray);
		}
		
		Belief newBelief = new Belief(id, sigma, ags, phi, getCASTTime());

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
 	 * Return the union entity ID of the union the belief refers to.
 	 * @param b
 	 * @return
 	 */
    public static String referringUnionId(Belief b) {
    	if (b.phi instanceof ComplexFormula) {
			
			for (int i = 0; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula formula = ((ComplexFormula)b.phi).formulae[i];
				
				if (formula instanceof UnionRefProperty) {
					return ((UnionRefProperty)formula).unionRef;
				}
			}
		}
		return null;
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
			Belief b = getMemoryEntry(currentBeliefModel.k[i], Binder.BINDER_SA, Belief.class);
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


	/**
	 * Get a belief by belief ID.
	 * 
	 * @param beliefId
	 * @return
	 */

	public Belief getBelief(String beliefId) {
		Belief b;
		try {
			b = getMemoryEntry(beliefId, Binder.BINDER_SA, Belief.class);
			return b;
		}
		catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		}
		catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		}
		return null;

	}
	
	
	
}
