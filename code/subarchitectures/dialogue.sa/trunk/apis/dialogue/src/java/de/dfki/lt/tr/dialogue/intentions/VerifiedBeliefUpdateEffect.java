package de.dfki.lt.tr.dialogue.intentions;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import java.util.Map;

public class VerifiedBeliefUpdateEffect implements CASTEffect {

	private final WorkingMemoryAddress groundedBelief;
	
	private final Map<String, dFormula> featuresToSet;
	
	public VerifiedBeliefUpdateEffect(WorkingMemoryAddress groundedBelief, Map<String, dFormula> featuresToSet) {
		this.groundedBelief = groundedBelief;
		this.featuresToSet = featuresToSet;
	}
	
	@Override
	public void makeItSo(ManagedComponent component) {
		component.getLogger().info("about to update the verified belief for ["
				+ groundedBelief.id + "," + groundedBelief.subarchitecture + "]");

		String s = "";
		for (String feature : featuresToSet.keySet()) {
			s += "  " + feature + " -> " + BeliefIntentionUtils.dFormulaToString(featuresToSet.get(feature)) + "\n";
		}
		component.getLogger().info("will set the following features in the verified belief: {\n" + s + "}");

		// TODO: retrieve the verified belief
		
		// update the verified belief in memory
		for (String feature : featuresToSet.keySet()) {
			component.getLogger().info("setting " + feature
					+ " -> "
					+ BeliefIntentionUtils.dFormulaToString(featuresToSet.get(feature)) + "...");

			// TODO: do the actual update here
		}
		component.getLogger().info("done setting up things");
		
		// TODO: write the verified belief back to WM
	}

	
}
