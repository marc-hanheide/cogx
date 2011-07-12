/**
 * 
 */
package eu.cogx.perceptmediator;

import java.util.EnumSet;
import java.util.Set;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntrySynchronizer;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * This implements a monitor for specific types of low-level percepts that are
 * propagated to the Binder. It implements Runnable so that several monitors can
 * be executed concurrently.
 * 
 * @author marc
 * 
 */
public class PerceptBindingMediator<T extends Ice.ObjectImpl, B extends dBelief>
		extends WMEntrySynchronizer<T, B> {

	public static <FromS extends Ice.ObjectImpl, B2 extends dBelief> PerceptBindingMediator<FromS, B2> create(
			ManagedComponent component, String toSA, Class<FromS> fromType,
			Class<B2> beliefType, TransferFunction<FromS, B2> tf) {
		Set<WorkingMemoryOperation> ops = EnumSet
				.allOf(WorkingMemoryOperation.class);

		return new PerceptBindingMediator<FromS, B2>(component, toSA, fromType,
				beliefType, tf, ops);
	}

	public static <FromS extends Ice.ObjectImpl, B2 extends dBelief> PerceptBindingMediator<FromS, B2> create(
			ManagedComponent component, Class<FromS> fromType,
			Class<B2> beliefType, TransferFunction<FromS, B2> tf) {
		Set<WorkingMemoryOperation> ops = EnumSet
				.allOf(WorkingMemoryOperation.class);

		return new PerceptBindingMediator<FromS, B2>(component, fromType,
				beliefType, tf, ops);
	}

	public static <FromS extends Ice.ObjectImpl, B2 extends dBelief> PerceptBindingMediator<FromS, B2> create(
			ManagedComponent component, Class<FromS> fromType,
			Class<B2> beliefType, TransferFunction<FromS, B2> tf,
			Set<WorkingMemoryOperation> wmOps) {

		return new PerceptBindingMediator<FromS, B2>(component, fromType,
				beliefType, tf, wmOps);
	}

	protected PerceptBindingMediator(
			ManagedComponent c,
			Class<T> fromType,
			Class<B> beliefType,
			castutils.castextensions.WMEntrySynchronizer.TransferFunction<T, B> transferFunction,
			Set<WorkingMemoryOperation> ops) {
		super(c, fromType, beliefType, transferFunction, ops);
	}

	protected PerceptBindingMediator(
			ManagedComponent c,
			String toSA,
			Class<T> fromType,
			Class<B> beliefType,
			castutils.castextensions.WMEntrySynchronizer.TransferFunction<T, B> transferFunction,
			Set<WorkingMemoryOperation> ops) {
		super(c, toSA, fromType, beliefType, transferFunction, ops);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return PerceptBindingMediator.class.getSimpleName() + "-"
				+ transferFunction.getClass().getSimpleName() + "-"
				+ this.fromType.getName() + "(" + super.toString() + ")";
	}

}
