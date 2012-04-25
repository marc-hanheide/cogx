package eu.cogx.percepttracker;

import java.util.Collection;
import java.util.List;

import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.PointerMap;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * Matching function that never matches anything. Allows you to use WMTracker
 * in a way that always creates a new To belief for a new observation of a From belief.
 * 
 * @author nah
 * 
 * @param <From>
 * @param <To>
 */
public class AlwaysFalseMatcher<From extends dBelief, To extends dBelief>
		extends FormulaMatcher<From, To> {

	public AlwaysFalseMatcher(List<String> types, PointerMap<?> map,
			Class<From> _fromCls, Class<To> _toCls) {
		super(types, map, _fromCls, _toCls);
	}

	public AlwaysFalseMatcher(List<String> types, PointerMap<?> map,
			Collection<String> ignoreKeys, Class<From> _fromCls,
			Class<To> _toCls) {
		super(types, map, ignoreKeys, _fromCls, _toCls);
	}

	@Override
	public double match(WorkingMemoryChange wmc, From from, To to) {
		return 0;
	}


}
