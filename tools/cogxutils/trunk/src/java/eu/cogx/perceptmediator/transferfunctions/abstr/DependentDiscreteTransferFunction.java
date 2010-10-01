package eu.cogx.perceptmediator.transferfunctions.abstr;

import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;
import eu.cogx.beliefs.slice.PerceptBelief;

/**
 * a more complex abstract {@link TransferFunction} required to realize
 * relations and processes where one belief links to another and it has to be
 * made sure that the linked belief is already on the binder. Uses
 * {@link WMContentWaiter} to guarantee this.
 * 
 * @author marc
 * 
 * @param <From>
 *            the type to transfer from
 */
public abstract class DependentDiscreteTransferFunction<From extends Ice.ObjectImpl>
		extends SimpleDiscreteTransferFunction<From, PerceptBelief> {

	protected WMView<PerceptBelief> allBeliefs;
	WMContentWaiter<PerceptBelief> waitingBeliefReader;

	/**
	 * constructor. Requires a {@link WMView} of all {@link PerceptBelief} to
	 * look for the referred Beliefs.
	 * 
	 * @see SimpleDiscreteTransferFunction
	 * @param component
	 * @param allBeliefs
	 * @param logger
	 */
	public DependentDiscreteTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs, Logger logger) {
		super(component, logger, PerceptBelief.class);
		this.allBeliefs = allBeliefs;
		waitingBeliefReader = new WMContentWaiter<PerceptBelief>(
				this.allBeliefs);
	}

	/**
	 * this method tries to dereference a referred belief. It waits for the
	 * referred object to appear and blocks until then.
	 * 
	 * @param contentMatchingFunction the matching function to be used
	 * @see WMContentWaiter
	 * @return the working memory address that corresponds
	 * @throws InterruptedException
	 */
	protected WorkingMemoryAddress getReferredBelief(
			ContentMatchingFunction<PerceptBelief> contentMatchingFunction)
			throws InterruptedException {
		logger.debug("trying to find referred belief");
		Entry<WorkingMemoryAddress, PerceptBelief> entry = waitingBeliefReader
				.read(contentMatchingFunction);
		logger.debug("got it: " + entry.getKey().id);
		return entry.getKey();
	}

}
