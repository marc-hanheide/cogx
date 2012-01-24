package eu.cogx.perceptmediator.transferfunctions.abstr;

import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import castutils.castextensions.BeliefWaiter;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.helpers.BeliefAncestorMatchingFunction;

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
public abstract class DependentListDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends dBelief>
		extends ListDiscreteTransferFunction<From, To> {

	protected WMView<To> allBeliefs;
	BeliefWaiter<To> waitingBeliefReader;

	/**
	 * constructor. Requires a {@link WMView} of all {@link PerceptBelief} to
	 * look for the referred Beliefs.
	 * 
	 * @see ListDiscreteTransferFunction
	 * @param component
	 * @param allBeliefs
	 * @param logger
	 */
	public DependentListDiscreteTransferFunction(ManagedComponent component,
			WMView<To> allBeliefs, Logger logger, Class<To> beliefClass) {
		super(component, logger, beliefClass);
		this.allBeliefs = allBeliefs;
		waitingBeliefReader = new BeliefWaiter<To>(this.allBeliefs);
	}

	/**
	 * this method tries to dereference a referred belief. It waits for the
	 * referred object to appear and blocks until then.
	 * 
	 * @param contentMatchingFunction
	 *            the matching function to be used
	 * @see WMContentWaiter
	 * @return the working memory address that corresponds
	 * @throws InterruptedException
	 */
	protected WorkingMemoryPointer getReferredBelief(
			ContentMatchingFunction<? super To> contentMatchingFunction)
			throws InterruptedException {
		getLogger().debug("trying to find referred belief");
		Entry<WorkingMemoryAddress, To> entry = waitingBeliefReader
				.read(contentMatchingFunction);
		getLogger().debug("got it: " + entry.getKey().id);
		return new WorkingMemoryPointer(entry.getKey(),CASTUtils.typeName(entry.getValue()));
	}

	/**
	 * this method tries to dereference a referred belief using a pointer to its
	 * ancestor, using the the {@link BeliefAncestorMatchingFunction}. It waits
	 * for the referred object to appear and blocks until then.
	 * 
	 * @param contentMatchingFunction
	 *            the matching function to be used
	 * @see WMContentWaiter
	 * @return the working memory address that corresponds
	 * @throws InterruptedException
	 */
	protected WorkingMemoryPointer getReferredBeliefWithAncestor(
			WorkingMemoryPointer _ancestorPointer) throws InterruptedException {
		return getReferredBelief(new BeliefAncestorMatchingFunction(
				_ancestorPointer));
	}

}
