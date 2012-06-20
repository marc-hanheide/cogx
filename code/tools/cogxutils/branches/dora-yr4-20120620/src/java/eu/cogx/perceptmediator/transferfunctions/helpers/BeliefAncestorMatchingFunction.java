package eu.cogx.perceptmediator.transferfunctions.helpers;

import cast.cdl.WorkingMemoryPointer;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * Matches the object belief created from the
 * 
 * @author nah
 * 
 */
public class BeliefAncestorMatchingFunction implements
		ContentMatchingFunction<dBelief> {

	private WorkingMemoryPointer m_ancestorPointer;

	public BeliefAncestorMatchingFunction(WorkingMemoryPointer _ancestorPointer) {
		m_ancestorPointer = _ancestorPointer;
	}

	/**
	 * Equals method for {@link WorkingMemoryPointer}s.
	 * 
	 * @param _lhs
	 * @param _rhs
	 * @return
	 */
	public static boolean equals(WorkingMemoryPointer _lhs,
			WorkingMemoryPointer _rhs) {
		return _lhs.address.equals(_rhs.address) && _lhs.type.equals(_rhs.type);
	}

	/**
	 * Matches the belief based on whether the stored ancestor pointer appears
	 * in the input belief's ancestor list;
	 */
	@Override
	public boolean matches(dBelief _input) {

		CASTBeliefHistory hist = (CASTBeliefHistory) _input.hist;

		for (WorkingMemoryPointer ancestorPointer : hist.ancestors) {
			if (equals(ancestorPointer, m_ancestorPointer)) {
				return true;
			}
		}

		return false;
	}

}
