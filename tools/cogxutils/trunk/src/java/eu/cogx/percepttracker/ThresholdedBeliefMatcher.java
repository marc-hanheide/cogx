package eu.cogx.percepttracker;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.PointerMap;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

public class ThresholdedBeliefMatcher<From extends dBelief, To extends dBelief>
		extends FormulaMatcher<From, To> {

	/**
	 * The belief from which this belief is derived
	 */
	public final String SOURCE_BELIEF_ID = "source-belief";

	public ThresholdedBeliefMatcher(List<String> _types, PointerMap<?> _map,
			Class<From> _fromCls, Class<To> _toCls) {
		super(_types, _map, _fromCls, _toCls);
	}

	public ThresholdedBeliefMatcher(List<String> _types, PointerMap<?> _map,
			Collection<String> _ignoreKeys, Class<From> _fromCls,
			Class<To> _toCls) {
		super(_types, _map, _ignoreKeys, _fromCls, _toCls);
	}

	protected EpistemicStatus getEpistemicStatus() {
		SharedEpistemicStatus status = new SharedEpistemicStatus();
		status.cgagents = new ArrayList<String>(2);
		status.cgagents.add("robot");
		status.cgagents.add("your mum");
		return status;
	}

	@Override
	public To create(WorkingMemoryAddress idToCreate, WorkingMemoryChange wmc,
			From from) throws IncompatibleAssignmentException {
		To create = super.create(idToCreate, wmc, from);
		create.estatus = getEpistemicStatus();
		return create;
	}

	@Override
	public void update(WorkingMemoryChange wmc, From from, To to)
			throws IncompatibleAssignmentException {

		// add in specific things
		CASTIndependentFormulaDistributionsBelief<From> fromProxy = createFromBeliefProxy(from);

		// add pointer back to belief this was generated from

		//TODO put this back in and find out why it breaks things
		//		WMPointer formula = WMPointer.create(wmc.address);
		PropositionFormula formula = PropositionFormula.create(wmc.address.id + " " + wmc.address.subarchitecture);	
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(formula.get(), 1d);
		fromProxy.getContent().put(SOURCE_BELIEF_ID, fd);

		// update with rest
		super.update(wmc, from, to);

	}
}
