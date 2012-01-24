/**
 * 
 */
package eu.cogx.beliefs.utils;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;

/**
 * 
 * Helper methods for some operations on belief structures.
 * 
 * @author nah
 * 
 */
public class BeliefUtils {

	public static IndependentFormulaDistributionsBelief<dBelief> getMostRecentBeliefAncestor(
			ManagedComponent _component,
			IndependentFormulaDistributionsBelief<? extends dBelief> _gb)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		CASTBeliefHistory hist = (CASTBeliefHistory) _gb.get().hist;

		IndependentFormulaDistributionsBelief<dBelief> perceptBel = IndependentFormulaDistributionsBelief
				.create(dBelief.class, _component.getMemoryEntry(
						hist.ancestors.get(hist.ancestors.size() - 1).address,
						dBelief.class));
		return perceptBel;

	}

	public static WorkingMemoryPointer getMostRecentAncestorPointer(
			IndependentFormulaDistributionsBelief<?> _pb) {
		CASTBeliefHistory hist = (CASTBeliefHistory) _pb.get().hist;
		return hist.ancestors.get(hist.ancestors.size() - 1);
	}

	public static <PerceptType extends Ice.Object> PerceptType getMostRecentPerceptAncestor(
			ManagedComponent _component,
			IndependentFormulaDistributionsBelief<? extends dBelief> _pb,
			Class<PerceptType> _pCls) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		WorkingMemoryPointer ancestorPtr = getMostRecentAncestorPointer(_pb);
		assert ancestorPtr.type.equals(CASTUtils.typeName(_pCls)) : "type mismatch for requested percept ancestor";
		return _component.getMemoryEntry(ancestorPtr.address, _pCls);
	}

}
