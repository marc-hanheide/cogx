/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.percepttracker;

import java.util.List;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentBasicDistributions;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.util.ProbFormula;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;

/**
 * this is a generic matcher that matches any
 * {@link CASTIndependentFormulaDistributionsBelief} of type
 * {@link PerceptBelief} to {@link GroundedBelief}. It compares all contained
 * Formulas and if the most likely ones are matching in the
 * {@link PerceptBelief} and the {@link GroundedBelief} they are assumed to be
 * matching. To be used with a {@link WMTracker}.
 * 
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class FormulaMatcher implements
		WMTracker.MatcherFunction<PerceptBelief, GroundedBelief> {

	private Logger logger;

	/**
	 * create a new FormulaMatcher
	 * 
	 * @param types
	 *            all the belief types this matcher accepts. These have to be
	 *            {@link CASTIndependentFormulaDistributionsBelief}, otherwise
	 *            they will never match and an update will cause an
	 *            {@link IncompatibleAssignmentException}
	 */
	public FormulaMatcher(List<String> types, PointerMap<?> map) {
		super();
		this.beliefTypes = types;
		wm2wmMap = map;
		logger = Logger.getLogger(FormulaMatcher.class);
	}

	protected final List<String> beliefTypes;

	protected final PointerMap<?> wm2wmMap;

	/*
	 * (non-Javadoc)
	 * 
	 * @seeeu.cogx.percepttracker.WMTracker.MatcherFunction#create(cast.cdl.
	 * WorkingMemoryAddress, cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public GroundedBelief create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, PerceptBelief from)
			throws IncompatibleAssignmentException {
		logger.debug("create new belief for PerceptBelief " + wmc.address.id);
		if (beliefTypes != null)
			if (!beliefTypes.contains(from.type)) {
				return null;
			}
		try {
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class);
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb = CASTIndependentFormulaDistributionsBelief
					.create(PerceptBelief.class, from);

			gb.setType(pb.getType());
			gb.setTime(pb.getStartTime(), pb.getEndTime());
			gb.setId(idToCreate.id);
			return gb.get();
		} catch (ClassCastException e) {
			throw (new IncompatibleAssignmentException(
					"cannot create new GroundedBelief due to incompatible data types"));
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see eu.cogx.percepttracker.WMTracker.MatcherFunction#match(cast.cdl.
	 * WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	/*
	 * (non-Javadoc)
	 * 
	 * @seeeu.cogx.percepttracker.WMTracker.MatcherFunction#match(cast.cdl.
	 * WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	@Override
	public double match(WorkingMemoryChange wmc, PerceptBelief from,
			GroundedBelief to) {
		logger.debug("match new belief for PerceptBelief " + from.id
				+ " with existing GroundedBelief " + to.id);
		if (beliefTypes != null)
			if (!beliefTypes.contains(from.type))
				return 0.0;
		try {
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gb = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, to);
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb = CASTIndependentFormulaDistributionsBelief
					.create(PerceptBelief.class, from);
			// if the content matches we can assume this is matching
			if (matches(pb.getContent(), gb.getContent())) {
				logger.debug("  => 1.0");
				return 1.0;
			} else {
				logger.debug("  => 0.0");
				return 0.0;
			}
		} catch (ClassCastException e) {
			logger.debug("incompatible classes => 0.0");
			return 0.0;
		}
	}

	/**
	 * matches all distributions of this {@link IndependentFormulaDistributions}
	 * to the argument. If all FormulaDistributions in this have a matching
	 * correspondent in the one given as argument to this method true is
	 * returned. This is very similar to equals() but it is not bijective.
	 * Instead it allows the 'other' distribution to contain
	 * {@link FormulaDistribution} that are not present in this one.
	 * 
	 * @param other
	 *            the {@link IndependentBasicDistributions} to match against
	 * @return true if all {@link FormulaDistribution} in this have a matching
	 *         correspondent in other
	 */
	public boolean matches(IndependentFormulaDistributions compareThis,
			IndependentFormulaDistributions compareTo) {
		for (Entry<String, FormulaDistribution> entry : compareThis.entrySet()) {
			FormulaDistribution otherEntry = compareTo.get(entry.getKey());
			if (otherEntry == null)
				return false;
			logger.debug("match " + entry.getKey());
			Formula ft = entry.getValue().getDistribution().getMostLikely();
			Formula fo = otherEntry.getDistribution().getMostLikely();
			logger.debug("ft=" + ft.toString() + ", fo=" + fo.toString());

			// try {
			if (!ft.get().getClass().isInstance(fo.get()))
				return false;
			if (ft.get() instanceof PointerFormula) {
				PointerFormula wmPointer = (PointerFormula) ft.get();
				// look up the corresponding value of the groundedbelief for
				// this pointer
				WorkingMemoryAddress lookUpGroundedBelief;
				lookUpGroundedBelief = wm2wmMap.get(wmPointer.pointer);
				assert (lookUpGroundedBelief != null);
				// it should be always valid here!
				if (!lookUpGroundedBelief
						.equals(((PointerFormula) fo.get()).pointer))
					return false;
			} else if (ft.get() instanceof IntegerFormula) {
				if (ft.getInteger() != fo.getInteger())
					return false;
			} else if (ft.get() instanceof FloatFormula) {
				if (ft.getDouble() != fo.getDouble())
					return false;
			} else if (ft.get() instanceof BooleanFormula) {
				if (ft.getBoolean() != fo.getBoolean())
					return false;
			} else if (ft.get() instanceof ElementaryFormula) {
				if (ft.getProposition() != fo.getProposition())
					return false;
			}
			// }
		}
		return true;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @seeeu.cogx.percepttracker.WMTracker.MatcherFunction#update(cast.cdl.
	 * WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	@Override
	public void update(WorkingMemoryChange wmc, PerceptBelief from,
			GroundedBelief to) throws IncompatibleAssignmentException {
		if (beliefTypes != null) {
			if (!beliefTypes.contains(from.type))
				throw (new IncompatibleAssignmentException(
						"cannot update with from type " + from.type));
			if (!beliefTypes.contains(to.type))
				throw (new IncompatibleAssignmentException(
						"cannot update with to type " + to.type));
		}
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> groundedBelief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, to);
		CASTIndependentFormulaDistributionsBelief<PerceptBelief> perceptBelief = CASTIndependentFormulaDistributionsBelief
				.create(PerceptBelief.class, from);
		groundedBelief.setTime(groundedBelief.getStartTime(), perceptBelief.getEndTime());
		groundedBelief.getContent().putAll(perceptBelief.getContent());
		try {
			for (Entry<String, FormulaDistribution> entry : groundedBelief.getContent()
					.entrySet()) {
				for (ProbFormula f : entry.getValue().getDistribution()) {
					// propagate any PointerFormulas to PerceptBeliefs to GroundedBeliefs
					if (f.getFormula().get() instanceof PointerFormula) {
						WMPointer wmp = WMPointer.create((Ice.Object) f
								.getFormula().get());
						WorkingMemoryAddress lookUpGroundedBelief;

						lookUpGroundedBelief = wm2wmMap.waitFor(wmp.getVal());
						logger.info("update " + entry.getKey() + " formula " + f.getFormula().toString()+" to " + CASTUtils.toString(lookUpGroundedBelief));
						wmp.setVal(lookUpGroundedBelief);
						logger.info("new val: " + entry.getKey() + " formula " + f.getFormula().toString());
					}
				}
			}
		} catch (InterruptedException e) {
			logger.error("interrupted", e);
		}
	}

	@Override
	public boolean canHandle(PerceptBelief from) {
		if (beliefTypes == null)
			return true;
		else
			return beliefTypes.contains(from.type);
	}

}
