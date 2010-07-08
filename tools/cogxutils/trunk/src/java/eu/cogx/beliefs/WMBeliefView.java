/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.beliefs;

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

		private final String type;

		/**
		 * @param type
		 */
		public BeliefTypeFilter(String type) {
			this.type = type;
		}

		@Override
		public boolean matches(T2 element) {
			return (element.type.equals(type));
		}
	}

	public static <T2 extends dBelief> WMView<T2> create(
			ManagedComponent component, Class<T2> encapsulatedType, String type) {
		return WMView.create(component, encapsulatedType, new BeliefTypeFilter<T2>(type));
	}

	public static <T2 extends dBelief> WMView<T2> create(
			ManagedComponent component, Class<T2> encapsulatedType,
			String subarchitectureId, String type) {
		return WMView.create(component, encapsulatedType, subarchitectureId, new BeliefTypeFilter<T2>(type));
	}

}
