/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.beliefs;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ViewFilter;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class WMBeliefView {

	static class BeliefTypeFilter<T2 extends dBelief> implements ViewFilter<T2> {

		private final Set<String> types = new HashSet<String>();

		/**
		 * @param type
		 */
		public BeliefTypeFilter(String type) {
			this.types.add(type);
		}

		/**
		 * @param type
		 */
		public BeliefTypeFilter(String[] types) {
			for (String t : types)
				this.types.add(t);
		}

		@Override
		public boolean matches(T2 element) {
			return (types.contains(element.type));
		}
	}

	public static <T2 extends dBelief> WMView<T2> create(
			ManagedComponent component, Class<T2> encapsulatedType, String type) {
		return WMView.create(component, encapsulatedType,
				new BeliefTypeFilter<T2>(type));
	}

	public static <T2 extends dBelief> WMView<T2> create(
			ManagedComponent component, Class<T2> encapsulatedType,
			String subarchitectureId, String type) {
		return WMView.create(component, encapsulatedType, subarchitectureId,
				new BeliefTypeFilter<T2>(type));
	}

	public static <T2 extends dBelief> WMView<T2> create(
			ManagedComponent component, Class<T2> encapsulatedType,
			String[] type) {
		return WMView.create(component, encapsulatedType,
				new BeliefTypeFilter<T2>(type));
	}

	public static <T2 extends dBelief> WMView<T2> create(
			ManagedComponent component, Class<T2> encapsulatedType,
			String subarchitectureId, String[] type) {
		return WMView.create(component, encapsulatedType, subarchitectureId,
				new BeliefTypeFilter<T2>(type));
	}

}
