/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import java.util.HashMap;

import cast.cdl.CASTTime;

import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.factories.specific.IndependentFormulaDistributionsFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class CASTIndependentFormulaDistributionsBelief<T extends dBelief> extends
		GenericBelief<T, IndependentFormulaDistributions> {

	public static <T2 extends dBelief> CASTIndependentFormulaDistributionsBelief<T2> create(
			Class<? extends T2> type, dBelief o) {
		return new CASTIndependentFormulaDistributionsBelief<T2>(type, o);
	}

	public static <T2 extends dBelief> CASTIndependentFormulaDistributionsBelief<T2> create(
			Class<? extends T2> type) {
		T2 internal = createDefault(type);
		internal.content = new CondIndependentDistribs(
				new HashMap<String, ProbDistribution>());
		internal.frame = CASTFrame.create().get();
		return create(type, internal);
	}

	protected CASTIndependentFormulaDistributionsBelief(Class<? extends T> class1,
			dBelief content) {
		super(class1, IndependentFormulaDistributionsFactory.get(), content);
	}

	/* (non-Javadoc)
	 * @see de.dfki.lt.tr.beliefs.data.genericproxies.GenericBelief#getFrame()
	 */
	@Override
	public CASTFrame getFrame() {
		return CASTFrame.create(super.getFrame().get());
	}

	/**
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#age()
	 */
	public long age() {
		return getFrame().age();
	}

	/**
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#duration()
	 */
	public long duration() {
		return getFrame().duration();
	}

	/**
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#getPlace()
	 */
	public String getPlace() {
		return getFrame().getPlace();
	}

	/**
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#getStartTime()
	 */
	public CASTTime getStartTime() {
		return getFrame().getStartTime();
	}

	/**
	 * @return
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#getEndTime()
	 */
	public CASTTime getEndTime() {
		return getFrame().getEndTime();
	}

	/**
	 * @param s
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#setPlace(java.lang.String)
	 */
	public void setPlace(String s) {
		getFrame().setPlace(s);
	}

	/**
	 * @param start
	 * @param end
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#setTime(cast.cdl.CASTTime, cast.cdl.CASTTime)
	 */
	public void setTime(CASTTime start, CASTTime end) {
		getFrame().setTime(start, end);
	}
}
