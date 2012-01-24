/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.apache.log4j.Logger;

import VisionData.ViewConeMoveCost;
import VisionData.ViewConeMoveCostList;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPointer;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentListDiscreteTransferFunction;

/**
 * @author nah
 * 
 */
public class ViewConeMoveCostsTransferFunction
		extends
		DependentListDiscreteTransferFunction<ViewConeMoveCostList, GroundedBelief> {

	static Logger logger = Logger
			.getLogger(ViewConeMoveCostsTransferFunction.class);

	public ViewConeMoveCostsTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, logger, GroundedBelief.class);
	}

	@Override
	protected List<IndependentFormulaDistributions> getInnerDistributions(
			WorkingMemoryChange _wmc, ViewConeMoveCostList _from)
			throws InterruptedException {
		ArrayList<IndependentFormulaDistributions> inner = null;
		if (_from.costs.length > 0) {
			inner = new ArrayList<IndependentFormulaDistributions>();

			for (ViewConeMoveCost mc : _from.costs) {
				inner.add(createCostEntry("move-to-viewcone", (float) mc.cost,
						getReferredBeliefWithAncestor(mc.from),
						getReferredBeliefWithAncestor(mc.to)));
			}

		}
		return inner;
	}

	private void addActionName(IndependentFormulaDistributions _inner,
			String _action) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_action, 1);
		_inner.put("action", fd);
	}

	private IndependentFormulaDistributions createCostEntry(String _action,
			float _cost, WorkingMemoryPointer... _vals) {
		IndependentFormulaDistributions inner = IndependentFormulaDistributions
				.create(new CondIndependentDistribs(
						new HashMap<String, ProbDistribution>()));

		addActionName(inner, _action);

		for (int i = 0; i < _vals.length; i++) {
			addActionArg(inner, _vals[i], i);
		}

		addActionCost(inner, _action, _cost);

		return inner;
	}

	private void addActionArg(IndependentFormulaDistributions _inner,
			WorkingMemoryPointer _beliefPointer, int _i) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(WMPointer.create(_beliefPointer).get(), 1);
		_inner.put("val" + _i, fd);
	}

	private void addActionCost(IndependentFormulaDistributions _inner,
			String _action, float _cost) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(_cost, 1);
		_inner.put(_action + "-cost", fd);
	}
}
