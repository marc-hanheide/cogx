/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.abstr;

import java.util.ArrayList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.StringTokenizer;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.interfaces.TimeServerPrx;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
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
public abstract class SimpleDiscreteTransferFunction<From extends Ice.ObjectImpl, To extends dBelief>
		extends CASTHelper implements TransferFunction<From, To> {



	public static String getBeliefTypeFromCastType(
			Class<? extends Ice.Object> class1) {
		return getBeliefTypeFromCastType(CASTUtils.typeName(class1));
	}

	public static String getBeliefTypeFromCastType(String casttype) {
		StringTokenizer st = new StringTokenizer(casttype, ":");
		String type = casttype.toLowerCase();
		while (st.hasMoreTokens())
			type = st.nextToken();
		return type;
	}

	private static TimeServerPrx timeServer = null;
	final protected Class<To> beliefClass;

	/**
	 * constructor
	 * 
	 * @param component
	 * @param beliefType TODO
	 */
	public SimpleDiscreteTransferFunction(ManagedComponent component,
			Logger logger, Class<To> beliefType) {
		super(component);
		this.beliefClass=beliefType;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.components.perceptmediator.TransferFunction#createBelief(java.
	 * lang.String, cast.cdl.CASTTime)
	 */
	@Override
	public To create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, From from) {

		try {
			// // create a simple history with just the link to the percept
			// CASTBeliefHistory hist = PerceptBuilder
			// .createNewPerceptHistory(wmc.address);
			//
			// // always create a CondIndependentDistribs
			// CondIndependentDistribs features = BeliefContentBuilder
			// .createNewCondIndependentDistribs();

			CASTIndependentFormulaDistributionsBelief<To> pb = CASTIndependentFormulaDistributionsBelief
					.create(beliefClass);
			pb.setId(idToCreate.id);
			pb.setType(getBeliefTypeFromCastType(wmc.type));
			pb.setPrivate("robot");
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
		assert (perceptBelief.content instanceof CondIndependentDistribs);

		CASTIndependentFormulaDistributionsBelief<To> p = CASTIndependentFormulaDistributionsBelief
				.create(beliefClass, perceptBelief);

		// update the end time to now!
		CASTTime starttime = p.getStartTime();
		p.setTime(starttime, now());

		addAncestors(wmc, from, p);
		
		IndependentFormulaDistributions features = p.getContent();
		Map<String, Formula> mapping;
		try {
			mapping = getFeatureValueMapping(wmc, from);
			

			for (Entry<String, Formula> fvm : mapping.entrySet()) {
				FormulaDistribution fd = FormulaDistribution.create();
				fd.add(fvm.getValue().get(), 1.0);
				features.put(fvm.getKey(), fd);
			}
			fillBelief(p, wmc, from);
		} catch (InterruptedException e) {
			component.logException(e);
		} catch (BeliefException e) {
			component.logException(e);
		}
		return true;
	}

	private void addAncestors(WorkingMemoryChange _wmc, From _from,
			CASTIndependentFormulaDistributionsBelief<To> _p) {
		CASTBeliefHistory hist = new CASTBeliefHistory(new ArrayList<WorkingMemoryPointer>(1), new ArrayList<WorkingMemoryPointer>(0));
		hist.ancestors.add(new WorkingMemoryPointer(_wmc.address, _wmc.type));
		_p.get().hist = hist;
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

	/**
	 * is being called after getFeatureValueMapping() and can be used in a
	 * subclass to populate the belief with more fine-controlled values that
	 * cannot be achieved by the simple mapping.
	 * 
	 * @param belief the belief to be modified
	 */
	protected void fillBelief(
			CASTIndependentFormulaDistributionsBelief<To> belief, WorkingMemoryChange wmc, From from) {

	}

	public static CASTTime now() {
		if (timeServer == null)
			timeServer = CASTUtils.getTimeServer();
		return timeServer.getCASTTime();
	}
}
