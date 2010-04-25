package binder.abstr;

import java.util.LinkedList;
import java.util.List;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTData;

public abstract class FakeComponent extends BeliefWriter {

	
	protected <Type extends Belief> void updatePointers
		(Belief newlyConstructedBelief, Class<Type> cls) {
		
		updatePointersInCurrentBelief(newlyConstructedBelief);
	//	updatePointersInOtherBeliefs(newlyConstructedBelief,cls);

	}

	
	protected void addOffspring (Belief oldBelief, String newBeliefId) {
		addOffspring(oldBelief, new WorkingMemoryAddress(newBeliefId, 
				BindingWorkingMemory.BINDER_SA));
	}
	

	protected void addOffspring (Belief oldBelief, List<String> newBeliefIds) {
		for (String newBeliefId : newBeliefIds) {
		addOffspring(oldBelief, new WorkingMemoryAddress(newBeliefId, 
				BindingWorkingMemory.BINDER_SA));
		}
	}
	
	
	protected void addOffspring (Belief oldBelief, WorkingMemoryAddress addressUnion) {

		if (oldBelief.hist != null && oldBelief.hist instanceof CASTBeliefHistory) {
			((CASTBeliefHistory)oldBelief.hist).offspring.add(addressUnion);
		}
		else {
			log("WARNING: offspring of percept is ill-formed");
			oldBelief.hist = new CASTBeliefHistory(new LinkedList<WorkingMemoryAddress>(), new LinkedList<WorkingMemoryAddress>());
			((CASTBeliefHistory)oldBelief.hist).offspring.add(addressUnion);
		}
	}


	protected void updatePointersInCurrentBelief (Belief newBelief) {
		try {

			for (FeatureValueProbPair pointer : FeatureContentUtils.getAllPointerValuesInBelief(newBelief)) {

				WorkingMemoryAddress point = ((PointerValue)pointer.val).beliefId ;
				if (existsOnWorkingMemory(point)) {
				Belief belief = getMemoryEntry(point, Belief.class);
				if (((CASTBeliefHistory)belief.hist).offspring.size() > 0) {
					((PointerValue)pointer.val).beliefId = ((CASTBeliefHistory)belief.hist).offspring.get(0);	
				}
				else {
					log("(updating pointers) WARNING: belief " + belief.hist + " has no offspring");
				}
				}
				else {
					log("(updating pointers) WARNING, belief " + point.id + " not yet in working memory");
				}

			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	



	
	protected ProbDistribution mergeBeliefContent (ProbDistribution distribA, ProbDistribution distribB) {
		
		try {
			ProbDistribution distrib1 = FeatureContentUtils.duplicateContent(distribA);
			ProbDistribution distrib2 = FeatureContentUtils.duplicateContent(distribB);

			if (distrib1 instanceof CondIndependentDistribs && distrib2 instanceof CondIndependentDistribs) {
			CondIndependentDistribs merge = BeliefContentBuilder.createNewCondIndependentDistribs();
			for (String key: ((CondIndependentDistribs)distrib1).distribs.keySet()) {
				BeliefContentBuilder.putNewCondIndependentDistrib
					(merge, (BasicProbDistribution)((CondIndependentDistribs)distrib1).distribs.get(key));
			}
			for (String key: ((CondIndependentDistribs)distrib2).distribs.keySet()) {
				BeliefContentBuilder.putNewCondIndependentDistrib
					(merge, (BasicProbDistribution)((CondIndependentDistribs)distrib2).distribs.get(key));
			}
			return merge;
		}
		else if (distrib1 instanceof DistributionWithExistDep && distrib2 instanceof DistributionWithExistDep) {
			CondIndependentDistribs merge = BeliefContentBuilder.createNewCondIndependentDistribs();
			for (String key: ((CondIndependentDistribs)distrib1).distribs.keySet()) {
				BeliefContentBuilder.putNewCondIndependentDistrib
					(merge, (BasicProbDistribution)((CondIndependentDistribs)distrib1).distribs.get(key));
			}
			for (String key: ((CondIndependentDistribs)distrib2).distribs.keySet()) {
				BeliefContentBuilder.putNewCondIndependentDistrib
					(merge, (BasicProbDistribution)((CondIndependentDistribs)distrib2).distribs.get(key));
			}
			DistributionWithExistDep mergeWithExist = BeliefContentBuilder.createNewDistributionWithExistDep(((DistributionWithExistDep)distrib1).existProb, merge);
			return mergeWithExist;
		}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return distribA;

	}
	
	private <T extends Belief> void updatePointersInOtherBeliefs 
		(Belief newBelief, Class<T> cls) {
		try {

			if (newBelief.hist != null && newBelief.hist instanceof CASTBeliefHistory) {

				for (WorkingMemoryAddress ancestor: ((CASTBeliefHistory)newBelief.hist).ancestors) {

					CASTData<T>[] existingBeliefs = getWorkingMemoryEntries(cls);

					for (int i = 0; i < existingBeliefs.length ; i++)  {
						T existingBelief = existingBeliefs[i].getData();
						for (FeatureValueProbPair pointerValueInBelief : FeatureContentUtils.getAllPointerValuesInBelief(existingBelief)) {
							PointerValue val = (PointerValue)pointerValueInBelief.val;
							if (val.beliefId.equals(ancestor)) {
								((PointerValue)pointerValueInBelief.val).beliefId = 
									new WorkingMemoryAddress(newBelief.id, BindingWorkingMemory.BINDER_SA);
								updateBeliefOnWM(existingBelief);
							}
						}
					}
				} 
			}


		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
}
