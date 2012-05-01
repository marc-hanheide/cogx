/**
 * 
 */
package eu.cogx.beliefs.utils;

import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

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

	public static <BeliefClass extends dBelief> WorkingMemoryPointer getMostRecentAncestorPointer(
			ManagedComponent _component, WorkingMemoryAddress _beliefAddress,
			Class<BeliefClass> _clz) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {

		BeliefClass belief = _component.getMemoryEntry(_beliefAddress, _clz);
		CASTIndependentFormulaDistributionsBelief<BeliefClass> pb = CASTIndependentFormulaDistributionsBelief
				.create(_clz, belief);
		CASTBeliefHistory hist = (CASTBeliefHistory) pb.get().hist;
		return hist.ancestors.get(hist.ancestors.size() - 1);
	}

	/**
	 * Goes through all ancestors of a belief until it finds one with a matching
	 * type.
	 * 
	 * @param _component
	 * @param _beliefAddress
	 * @param _type
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	public static WorkingMemoryPointer recurseAncestorsForType(
			ManagedComponent _component, WorkingMemoryAddress _beliefAddress,
			String _type) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {

		// TODO need to catch something for incorrect class here I think

		// starting belief
		dBelief belief = _component.getMemoryEntry(_beliefAddress,
				dBelief.class);
		CASTIndependentFormulaDistributionsBelief<dBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(dBelief.class, belief);

		// if it's not an instance of this then it has not been set yet
		if (pb.get().hist instanceof CASTBeliefHistory) {

			CASTBeliefHistory hist = (CASTBeliefHistory) pb.get().hist;

			for (WorkingMemoryPointer ancestor : hist.ancestors) {
				if (ancestor.type.equals(_type)) {
					return ancestor;
				} else {
					WorkingMemoryPointer recurseResult = recurseAncestorsForType(
							_component, ancestor.address, _type);
					if (recurseResult != null) {
						return recurseResult;
					}
				}
			}
		}

		return null;
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

	/**
	 * Add or update a feature-value pair in a belief. Fetches the belief from
	 * WM and writes the value back.
	 * 
	 * @param _component
	 * @param _beliefAddress
	 * @param _clz
	 * @param _feature
	 * @param _value
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	public static <BeliefClass extends dBelief> void addFeature(
			ManagedComponent _component, WorkingMemoryAddress _beliefAddress,
			Class<BeliefClass> _clz, String _feature, String _value)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {

		BeliefClass belief = _component.getMemoryEntry(_beliefAddress, _clz);
		CASTIndependentFormulaDistributionsBelief<BeliefClass> pb = CASTIndependentFormulaDistributionsBelief
				.create(_clz, belief);
		addFeature(pb, _feature, _value);
		_component.overwriteWorkingMemory(_beliefAddress, pb.get());
	}

	public static <BeliefClass extends dBelief> void addFeature(
			ManagedComponent _component, WorkingMemoryAddress _beliefAddress,
			Class<BeliefClass> _clz, String _feature, boolean _value)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {

		BeliefClass belief = _component.getMemoryEntry(_beliefAddress, _clz);
		CASTIndependentFormulaDistributionsBelief<BeliefClass> pb = CASTIndependentFormulaDistributionsBelief
				.create(_clz, belief);
		addFeature(pb, _feature, _value);
		_component.overwriteWorkingMemory(_beliefAddress, pb.get());
	}

	public static <BeliefClass extends dBelief> void addFeature(
			CASTIndependentFormulaDistributionsBelief<BeliefClass> _belief,
			String _feature, String _value) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, 1);
		_belief.getContent().put(_feature, fd);
	}

	public static <BeliefClass extends dBelief> void addFeature(
			CASTIndependentFormulaDistributionsBelief<BeliefClass> _belief,
			String _feature, dFormula _value) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, 1);
		_belief.getContent().put(_feature, fd);
	}

	public static <BeliefClass extends dBelief> void addFeature(
			CASTIndependentFormulaDistributionsBelief<BeliefClass> _belief,
			String _feature, boolean _value) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_value, 1);
		_belief.getContent().put(_feature, fd);
	}
}
