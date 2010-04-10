package binder.components.perceptmediator.transferfunctions;

import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import beliefmodels.autogen.beliefs.PerceptBelief;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;

public abstract class DependentDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends PerceptBelief>
		extends SimpleDiscreteTransferFunction<From, To> {

	public interface ReferenceMatchingFunction<RT> {
		boolean matches(RT r);
	}

	WMView<To> allBeliefs;

	/**
	 * @param places
	 */
	public DependentDiscreteTransferFunction(WMView<To> relatedObjects) {
		super();
		// this.relatedObjects = relatedObjects;

		relatedObjects.registerHandler(new ChangeHandler<To>() {

			@Override
			public void entryChanged(Map<WorkingMemoryAddress, To> map,
					WorkingMemoryChange wmc, To newEntry, To oldEntry)
					throws CASTException {
				notifyWaitingThreads();

			}

		});

	}

	private synchronized void notifyWaitingThreads() {
		notifyAll();
	}

	/**
	 * this method tries to dereference a referred belief. It waits for the
	 * referred object to appear and blocks until then.
	 * 
	 * @param rmf
	 * @return the working memory address that corresponds
	 * @throws InterruptedException
	 */
	protected synchronized WorkingMemoryAddress getReferredBelief(
			ReferenceMatchingFunction<To> rmf) throws InterruptedException {

		while (true) {
			for (Entry<WorkingMemoryAddress, To> belief : allBeliefs
					.entrySet()) {
				if (rmf.matches(belief.getValue())) {
					return belief.getKey();
				}
				return belief.getKey();

			}
			Logger
					.getLogger(DependentDiscreteTransferFunction.class)
					.info(
							"didn't find the referred entity yet, have to wait until something new appears on the binder");

			wait();
		}
	}

}
