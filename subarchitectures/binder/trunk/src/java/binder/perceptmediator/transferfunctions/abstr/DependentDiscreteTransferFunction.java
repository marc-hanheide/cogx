package binder.perceptmediator.transferfunctions.abstr;

import java.util.Map.Entry;

import org.apache.log4j.Logger;

import beliefmodels.autogen.beliefs.PerceptBelief;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

public abstract class DependentDiscreteTransferFunction<From extends Ice.ObjectImpl>
		extends SimpleDiscreteTransferFunction<From> {

	public interface ReferenceMatchingFunction<RT> {
		boolean matches(RT r);
	}

	WMView<PerceptBelief> allBeliefs;
	WMContentWaiter<PerceptBelief> waitingBeliefReader;

	/**
	 * @param places
	 */
	public DependentDiscreteTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs, Logger logger) {
		super(component, logger);
		this.allBeliefs = allBeliefs;
		waitingBeliefReader = new WMContentWaiter<PerceptBelief>(this.allBeliefs);
	}

	/**
	 * this method tries to dereference a referred belief. It waits for the
	 * referred object to appear and blocks until then.
	 * 
	 * @param rmf
	 * @return the working memory address that corresponds
	 * @throws InterruptedException
	 */
	protected WorkingMemoryAddress getReferredBelief(
			ContentMatchingFunction<PerceptBelief> rmf) throws InterruptedException {
		logger.debug("trying to find referred belief");
		Entry<WorkingMemoryAddress, PerceptBelief> entry = waitingBeliefReader.read(rmf);
		logger.debug("got it: " + entry.getKey().id);
		return entry.getKey();
	}

}
