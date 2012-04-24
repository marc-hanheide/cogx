package de.dfki.lt.tr.dialogue.intentions;

import java.util.Map;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.beliefs.slice.VerifiedBelief;

public class VerifiedBeliefUpdateEffect implements CASTEffect {

	private final WorkingMemoryAddress aboutBeliefAddr;

	private final Map<String, dFormula> featuresToSet;

	public VerifiedBeliefUpdateEffect(WorkingMemoryAddress aboutBeliefAddr,
			Map<String, dFormula> featuresToSet) {
		this.aboutBeliefAddr = aboutBeliefAddr;
		this.featuresToSet = featuresToSet;
	}

	@Override
	public void makeItSo(ManagedComponent component) {

		try {

			component.getLogger().info(
					"about to update the verified belief for ["
							+ aboutBeliefAddr.id + ":"
							+ aboutBeliefAddr.subarchitecture + "]");

			String s = "";
			for (String feature : featuresToSet.keySet()) {
				s += "  "
						+ feature
						+ " -> "
						+ BeliefIntentionUtils.dFormulaToString(featuresToSet
								.get(feature)) + "\n";
			}
			component.getLogger().info(
					"will set the following features in the verified belief: {\n"
							+ s + "}");

			MergedBelief mergedBelief = component.getMemoryEntry(
					aboutBeliefAddr, MergedBelief.class);
			CASTBeliefHistory hist = (CASTBeliefHistory) mergedBelief.hist;
			WorkingMemoryPointer verifiedBeliefWMP = null;

			for (WorkingMemoryPointer ancestor : hist.ancestors) {
				if (ancestor.type.equals(VerifiedBelief.class.getName())) {
					verifiedBeliefWMP = ancestor;
					break;
				}
			}

			if (verifiedBeliefWMP != null) {

				component.log("found verified belief ancestor, updating");

				VerifiedBelief vb = component.getMemoryEntry(
						verifiedBeliefWMP.address, VerifiedBelief.class);
				CASTIndependentFormulaDistributionsBelief<VerifiedBelief> verifiedBelief = CASTIndependentFormulaDistributionsBelief
						.create(VerifiedBelief.class, vb);

				// update the verified belief in memory
				for (String feature : featuresToSet.keySet()) {
					component.getLogger().info(
							"setting "
									+ feature
									+ " -> "
									+ BeliefIntentionUtils
											.dFormulaToString(featuresToSet
													.get(feature)) + "...");

					// TODO: do the actual update here


				}
				component.getLogger().info("done setting up things");

				component.overwriteWorkingMemory(verifiedBeliefWMP.address,
						verifiedBelief.get());

			}
		} catch (CASTException e) {
			component.logException(e);
		}

	}

}
