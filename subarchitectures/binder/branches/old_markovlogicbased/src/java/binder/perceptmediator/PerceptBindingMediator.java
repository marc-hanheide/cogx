/**
 * 
 */
package binder.perceptmediator;

import java.util.EnumSet;
import java.util.Set;

import Ice.ObjectImpl;
import beliefmodels.autogen.beliefs.PerceptBelief;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEntrySynchronizer;

/**
 * This implements a monitor for specifc types of low-level percepts that are
 * propagated to the Binder. It implements Runnable so that several monitors can
 * be executed concurrently.
 * 
 * @author marc
 * 
 */
public class PerceptBindingMediator<T extends Ice.ObjectImpl> extends
		WMEntrySynchronizer<T, PerceptBelief> {

	protected PerceptBindingMediator(
			ManagedComponent c,
			Class<T> fromType,
			castutils.castextensions.WMEntrySynchronizer.TransferFunction<T, PerceptBelief> transferFunction,
			Set<WorkingMemoryOperation> ops) {
		super(c, fromType, PerceptBelief.class, transferFunction, ops);

	}

	public static <FromS extends ObjectImpl> PerceptBindingMediator<FromS> create(
			ManagedComponent component, Class<FromS> fromType,
			TransferFunction<FromS, PerceptBelief> tf,
			Set<WorkingMemoryOperation> wmOps) {

		return new PerceptBindingMediator<FromS>(component, fromType,
				tf, wmOps);
	}

	public static <FromS extends ObjectImpl> PerceptBindingMediator<FromS> create(
			ManagedComponent component, Class<FromS> fromType,
			TransferFunction<FromS, PerceptBelief> tf) {
		Set<WorkingMemoryOperation> ops = EnumSet
				.allOf(WorkingMemoryOperation.class);

		return new PerceptBindingMediator<FromS>(component, fromType,
				tf, ops);
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
