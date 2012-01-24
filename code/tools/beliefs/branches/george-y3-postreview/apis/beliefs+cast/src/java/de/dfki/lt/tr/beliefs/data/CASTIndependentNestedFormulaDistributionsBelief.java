/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentNestedFormulaDistributions;
import de.dfki.lt.tr.beliefs.factories.specific.IndependentNestedFormulaDistributionsFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribList;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class CASTIndependentNestedFormulaDistributionsBelief<T extends dBelief> extends
		CASTSuperBelief<T, IndependentNestedFormulaDistributions> {

	public static <T2 extends dBelief> CASTIndependentNestedFormulaDistributionsBelief<T2> create(
			Class<? extends T2> type, dBelief o) {
		return new CASTIndependentNestedFormulaDistributionsBelief<T2>(type, o);
	}

	public static <T2 extends dBelief> CASTIndependentNestedFormulaDistributionsBelief<T2> create(
			Class<? extends T2> type) {
		T2 internal = createDefault(type);
		internal.content = new CondIndependentDistribList(
				new LinkedList<ProbDistribution>());
		internal.frame = CASTFrame.create().get();
		return create(type, internal);
	}

	protected CASTIndependentNestedFormulaDistributionsBelief(Class<? extends T> class1,
			dBelief content) {
		super(class1, IndependentNestedFormulaDistributionsFactory.get(), content);
	}

	
}
