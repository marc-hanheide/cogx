package de.dfki.lt.tr.dialogue.intentions;

import java.util.Map;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.NegatedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import eu.cogx.beliefs.slice.VerifiedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;

/**
 * General update effect that sets belief values. Very similar to
 * {@link VerifiedBeliefUpdateEffect} except this doesn't do the extra "-prob"
 * value. TODO These should probably be refactored later.
 * 
 * @author nah
 * 
 */
public class FeatureValueUpdateEffect implements CASTEffect {

	private final WorkingMemoryAddress aboutBeliefAddr;

	private final Map<String, dFormula> featuresToSet;

	public FeatureValueUpdateEffect(WorkingMemoryAddress aboutBeliefAddr,
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

			WorkingMemoryPointer verifiedBeliefWMP = BeliefUtils
					.recurseAncestorsForType(component, aboutBeliefAddr,
							VerifiedBelief.class);

			if (verifiedBeliefWMP != null) {

				component.log("found verified belief ancestor, updating");

				VerifiedBelief vb = component.getMemoryEntry(
						verifiedBeliefWMP.address, VerifiedBelief.class);
				CASTIndependentFormulaDistributionsBelief<VerifiedBelief> verifiedBelief = CASTIndependentFormulaDistributionsBelief
						.create(VerifiedBelief.class, vb);

				// update the verified belief in memory
				for (String feature : featuresToSet.keySet()) {

					final boolean isPositive;

					// TODO: determine the polarity
					dFormula givenFormula = featuresToSet.get(feature);
					if (givenFormula instanceof NegatedFormula) {
						NegatedFormula nf = (NegatedFormula) givenFormula;
						isPositive = false;
						givenFormula = nf.negForm;
					} else {
						isPositive = true;
					}

					final String value;
					if (givenFormula instanceof ElementaryFormula) {
						ElementaryFormula ef = (ElementaryFormula) givenFormula;
						value = ef.prop;
					} else {
						value = null;
					}

					final double prob = isPositive ? 1.0 : 0.0;

					if (value != null) {

						component.log("setting " + feature + " -> " + value
								+ String.format(" @ p=%.2f", prob));

						// Not allowing more than one value

						// FormulaDistribution distr =
						// verifiedBelief.getContent()
						// .get(feature);
						// if (distr == null) {
						// distr = FormulaDistribution.create();
						// }

						FormulaDistribution distr = FormulaDistribution
								.create();
						distr.add(value, prob);
						verifiedBelief.getContent().put(feature, distr);

					} else {
						component.log("don't know how do the effect for "
								+ feature
								+ " -> "
								+ BeliefIntentionUtils
										.dFormulaToString(featuresToSet
												.get(feature)));
					}

				}

				component.log("done setting up things");

				component.overwriteWorkingMemory(verifiedBeliefWMP.address,
						verifiedBelief.get());
			}
		} catch (CASTException e) {
			component.logException(e);
		}

	}
}
