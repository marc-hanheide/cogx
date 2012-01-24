/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import cast.cdl.CASTTime;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Nick Hawes, but the person who really understands it (I hope) is Marc Hanheide (marc@hanheide.de)
 * 
 */
public class CASTSuperBelief<T extends dBelief, C extends Distribution<?>>
		extends GenericBelief<T, C> {

	protected CASTSuperBelief(Class<? extends T> class1,
			ProxyFactory<ProbDistribution, ? extends C> factory, dBelief content) {
		super(class1, factory, content);
	}

	/*
	 * (non-Javadoc)
	 * 
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
	 * @see de.dfki.lt.tr.beliefs.data.CASTFrame#setTime(cast.cdl.CASTTime,
	 *      cast.cdl.CASTTime)
	 */
	public void setTime(CASTTime start, CASTTime end) {
		getFrame().setTime(start, end);
	}
}
