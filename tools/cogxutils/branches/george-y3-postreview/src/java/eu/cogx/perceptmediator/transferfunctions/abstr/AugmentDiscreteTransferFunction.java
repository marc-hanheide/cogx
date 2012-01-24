/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.abstr;

import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;

/**
 * This is an abstract class for the most simple {@link TransferFunction} to
 * establish a mapping between input percepts (of generic type From) and
 * {@link PerceptBelief}. all it does is creating discrete FeatureDistribution
 * of an {@link CondIndependentDistribs} in a belief. This abstract
 * implementation requires the specific mapping between percepts and belief to
 * be implemented in a subclass by implementing getFeatureValueMapping.
 * 
 * @author marc
 * 
 * @param <From>
 *            type we generate beliefs from
 */
public abstract class AugmentDiscreteTransferFunction<From extends Ice.ObjectImpl>
		extends CASTHelper implements TransferFunction<From, PerceptBelief> {

	protected WMView<PerceptBelief> allBeliefs;
	WMContentWaiter<PerceptBelief> waitingBeliefReader;

	/**
	 * constructor
	 * 
	 * @param component
	 */
	public AugmentDiscreteTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs, Logger logger) {
		super(component);
		this.allBeliefs = allBeliefs;
		waitingBeliefReader = new WMContentWaiter<PerceptBelief>(
				this.allBeliefs);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#createBelief(java.
	 * lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public PerceptBelief create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, From from) {
		return null;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * castutils.castextensions.WMEntrySynchronizer.TransferFunction#transform
	 * (cast.cdl.WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	@Override
	public boolean transform(WorkingMemoryChange wmc, From from,
			PerceptBelief perceptBelief) {
		assert (perceptBelief != null);
		assert (perceptBelief.content != null);
		assert (perceptBelief.content instanceof CondIndependentDistribs);

		CASTIndependentFormulaDistributionsBelief<PerceptBelief> p = CASTIndependentFormulaDistributionsBelief
				.create(PerceptBelief.class, perceptBelief);

		// update the end time to now!
		CASTTime starttime = p.getStartTime();
		p.setTime(starttime, SimpleDiscreteTransferFunction.now());

		IndependentFormulaDistributions features = p.getContent();
		Map<String, Formula> mapping;
		try {
			mapping = getFeatureValueMapping(wmc, from);
			for (Entry<String, Formula> fvm : mapping.entrySet()) {
				FormulaDistribution fd = FormulaDistribution.create();
				fd.add(fvm.getValue().get(), 1.0);
				features.put(fvm.getKey(), fd);
			}
		} catch (InterruptedException e) {
			component.logException(e);
		} catch (BeliefException e) {
			component.logException(e);
		}
		return true;
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
		getLogger().debug("trying to find referred belief");
		Entry<WorkingMemoryAddress, PerceptBelief> entry = waitingBeliefReader
				.read(contentMatchingFunction);
        if (entry == null) {
            getLogger().debug("timed out");
            return null;
        }
		getLogger().debug("got it: " + entry.getKey().id);
		return entry.getKey();
	}

	/**
	 * the abstract method
	 * 
	 * @param wmc
	 *            the {@link WorkingMemoryChange} that caused the update
	 * @param from
	 *            the data entry that caused the update
	 * @return a {@link Map} of feature names and {@link FeatureValue} that will
	 *         be written to the belief.
	 * @throws InterruptedException
	 * @throws BeliefException
	 */
	protected abstract Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, From from) throws InterruptedException,
			BeliefException;

}
