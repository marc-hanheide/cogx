/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.abstr;

import java.util.List;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;
import de.dfki.lt.tr.beliefs.data.CASTIndependentNestedFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentNestedFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribList;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;

/**
 * This is an abstract class for the most simple {@link TransferFunction} to
 * establish a mapping between input percepts (of generic type From) and
 * {@link dBelief}. all it does is creating discrete FeatureDistribution of an
 * {@link CondIndependentDistribList} in a belief. This abstract implementation
 * requires the specific mapping between percepts and belief to be implemented
 * in a subclass by implementing getFeatureValueMapping.
 * 
 * @author nah (copied from {@link SimpleDiscreteTransferFunction}
 * 
 * @param <From>
 *            type we generate beliefs from
 */
public abstract class ListDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends dBelief>
		extends AbstractDiscreteTransferFunction<From, To> {

	/**
	 * constructor
	 * 
	 * @param component
	 * @param beliefType
	 *            TODO
	 */
	public ListDiscreteTransferFunction(ManagedComponent component,
			Logger logger, Class<To> beliefType) {
		super(component, logger, beliefType);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#createBelief(java.
	 * lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public To create(WorkingMemoryAddress idToCreate, WorkingMemoryChange wmc,
			From from) {

		try {
			CASTIndependentNestedFormulaDistributionsBelief<To> pb = CASTIndependentNestedFormulaDistributionsBelief
					.create(beliefClass);
			pb.setId(idToCreate.id);
			pb.setType(getBeliefTypeFromCastType(wmc.type));
			pb.setPrivate("self");
			return pb.get();
		} catch (BeliefException e) {
			component.logException(e);
			return null;
		}

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
			To perceptBelief) {
		assert (perceptBelief != null);
		assert (perceptBelief.content != null);
		assert (perceptBelief.content instanceof CondIndependentDistribList);

		CASTIndependentNestedFormulaDistributionsBelief<To> p = CASTIndependentNestedFormulaDistributionsBelief
				.create(beliefClass, perceptBelief);

		// update the end time to now!
		CASTTime starttime = p.getStartTime();
		p.setTime(starttime, now());

		addAncestors(wmc, from, p);

		IndependentNestedFormulaDistributions nested = p.getContent();
		try {
			List<IndependentFormulaDistributions> innerDistributions = getInnerDistributions(
					wmc, from);
			if (innerDistributions == null) {
				return false;
			}
			for (IndependentFormulaDistributions ifd : innerDistributions) {
				nested.add(ifd);
			}
			fillBelief(p, wmc, from);
		} catch (BeliefException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}

		return true;
	}

	/**
	 * the abstract method
	 * 
	 * @param wmc
	 *            the {@link WorkingMemoryChange} that caused the update
	 * @param from
	 *            the data entry that caused the update
	 * @return a {@link List} of inner {@link IndependentFormulaDistributions}
	 *         that will be written to the belief.
	 * @throws InterruptedException
	 * @throws BeliefException
	 */
	protected abstract List<IndependentFormulaDistributions> getInnerDistributions(
			WorkingMemoryChange wmc, From from) throws BeliefException,
			InterruptedException;

	/**
	 * is being called after getInnerDistributions() and can be used in a
	 * subclass to populate the belief with more fine-controlled values that
	 * cannot be achieved by the simple mapping.
	 * 
	 * @param belief
	 *            the belief to be modified
	 */
	protected void fillBelief(
			CASTIndependentNestedFormulaDistributionsBelief<To> belief,
			WorkingMemoryChange wmc, From from) {

	}

}
