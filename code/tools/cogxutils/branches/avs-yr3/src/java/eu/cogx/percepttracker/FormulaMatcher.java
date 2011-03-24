/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.percepttracker;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.PointerMap;
import castutils.castextensions.ValueNotAvailableException;
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
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.ProbFormula;

/**
 * this is a generic matcher that matches any
 * {@link CASTIndependentFormulaDistributionsBelief} of type {@link dBelief} to
 * {@link dBelief} (e.g. PerceptBelief to GroundedBelief). It compares all
 * contained Formulas and if the most likely ones are matching in the from
 * belief and the to belief they are assumed to be matching. There must be a
 * match for all formulas in the PerceptBelief, but it's ok if the
 * GroundedBelief has some more than are not corresponding. To be used with a
 * {@link WMTracker}.
 * 
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class FormulaMatcher<From extends dBelief, To extends dBelief>
		implements WMTracker.MatcherFunction<From, To> {

	/** the very small epsilon to test doubles for equality */
	private static final double EPSILON_EQUALITY = 1e-10;

	protected Set<String> ignoredKeys = new HashSet<String>();

	private Logger logger;

	private final Class<From> m_fromCls;

	private final Class<To> m_toCls;

	/**
	 * create a new FormulaMatcher
	 * 
	 * @param types
	 *            all the belief types this matcher accepts. These have to be
	 *            {@link CASTIndependentFormulaDistributionsBelief}, otherwise
	 *            they will never match and an update will cause an
	 *            {@link IncompatibleAssignmentException}
	 */
	public FormulaMatcher(List<String> types, PointerMap<?> map,
			Class<From> _fromCls, Class<To> _toCls) {
		super();
		this.beliefTypes = types;
		wm2wmMap = map;
		logger = Logger.getLogger(FormulaMatcher.class);
		m_fromCls = _fromCls;
		m_toCls = _toCls;

	}

	public FormulaMatcher(List<String> types, PointerMap<?> map,
			Collection<String> ignoreKeys, Class<From> _fromCls,
			Class<To> _toCls) {
		this(types, map, _fromCls, _toCls);
		this.ignoredKeys.addAll(ignoreKeys);
	}

	protected final List<String> beliefTypes;

	protected final PointerMap<?> wm2wmMap;

	protected final Set<String> ignoreForPointerPropagationSet = new HashSet<String>();

	/*
	 * (non-Javadoc)
	 * 
	 * @seeeu.cogx.percepttracker.WMTracker.MatcherFunction#create(cast.cdl.
	 * WorkingMemoryAddress, cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public To create(WorkingMemoryAddress idToCreate, WorkingMemoryChange wmc,
			From from) throws IncompatibleAssignmentException {
		logger.debug("create new belief for PerceptBelief " + wmc.address.id);
		if (beliefTypes != null)
			if (!beliefTypes.contains(from.type)) {
				return null;
			}
		try {
			CASTIndependentFormulaDistributionsBelief<To> gb = CASTIndependentFormulaDistributionsBelief
					.create(m_toCls);
			CASTIndependentFormulaDistributionsBelief<From> pb = createFromBeliefProxy(from);

			gb.setType(pb.getType());
			gb.setTime(pb.getStartTime(), pb.getEndTime());
			gb.setId(idToCreate.id);
			return gb.get();
		} catch (ClassCastException e) {
			throw (new IncompatibleAssignmentException("cannot create new "
					+ m_toCls + " due to incompatible data types"));
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
	public double match(WorkingMemoryChange wmc, From from, To to) {
		logger.debug("match new belief for PerceptBelief " + from.id
				+ " with existing GroundedBelief " + to.id);
		if (beliefTypes != null)
			if (!beliefTypes.contains(from.type))
				return 0.0;
		if (!to.type.equals(from.type))
			return 0.0;
		try {
			CASTIndependentFormulaDistributionsBelief<To> gb = createToBeliefProxy(to);
			CASTIndependentFormulaDistributionsBelief<From> pb = createFromBeliefProxy(from);
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
		} catch (InterruptedException e) {
			logger.warn("interrupted", e);
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
	 * @throws InterruptedException
	 */
	public boolean matches(IndependentFormulaDistributions compareThis,
			IndependentFormulaDistributions compareTo)
			throws InterruptedException {
		for (Entry<String, FormulaDistribution> entry : compareThis.entrySet()) {
			// if the current key is in the set of keys to ignore, we ignore it!
			if (ignoredKeys.contains(entry.getKey())) {
				logger.debug("ignore feature " + entry.getKey());
				continue;
			}
			FormulaDistribution otherEntry = compareTo.get(entry.getKey());
			if (otherEntry == null) {
				logger
						.debug("couldn't find corresponding formula, we continue to look for more");
				continue;
			}

			logger.debug("match " + entry.getKey());
			// get the most likely value from both; if they match we assume the
			// formulas do match!
			Formula ft = entry.getValue().getDistribution().getMostLikely();
			Formula fo = otherEntry.getDistribution().getMostLikely();

			logger.debug("ft=" + ft + ", fo=" + fo);
			// if any Formula didn't have a likely value, we take this as
			// acceptable and continue
			if (ft == null || fo == null)
				continue;

			// try {
			// if both Formulas are of different type, they never match
			if (!ft.get().getClass().isInstance(fo.get()))
				return false;
			// if we have a PointerFormula we have to find the corresponding ID
			// in the universe of the GroundedBeliefs to match
			if (ft.get() instanceof PointerFormula) {
				PointerFormula wmPointer = (PointerFormula) ft.get();
				// look up the corresponding value of the belief for
				// this pointer
				WorkingMemoryAddress lookUpGroundedBelief;
				try {
					if (wmPointer.type.equals(CASTUtils.typeName(m_fromCls))) {
						lookUpGroundedBelief = wm2wmMap
								.waitFor(wmPointer.pointer);
						assert (lookUpGroundedBelief != null);
						// it should be always valid here!
						if (!lookUpGroundedBelief.equals(((PointerFormula) fo
								.get()).pointer))
							return false;
					} else {
						if (!wmPointer.pointer.equals(((PointerFormula) fo
								.get()).pointer))
							return false;
					}
				} catch (ValueNotAvailableException e) {
					logger.warn("couldn't find "
							+ CASTUtils.toString(wmPointer.pointer), e);
				}
			} else if (ft.get() instanceof IntegerFormula) { // compare the
				// integer
				// values
				if (ft.getInteger() != fo.getInteger())
					return false;
			} else if (ft.get() instanceof FloatFormula) { // compare the float
				// values
				if (Math.abs(ft.getDouble() - fo.getDouble()) <= EPSILON_EQUALITY)
					return false;
			} else if (ft.get() instanceof BooleanFormula) { // compare the
				// boolean
				// values
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

	protected final CASTIndependentFormulaDistributionsBelief<To> createToBeliefProxy(
			To to) {
		return CASTIndependentFormulaDistributionsBelief.create(m_toCls, to);
	}

	protected final CASTIndependentFormulaDistributionsBelief<From> createFromBeliefProxy(
			From from) {
		return CASTIndependentFormulaDistributionsBelief
				.create(m_fromCls, from);
	}

	/*
	 * (non-Javadoc)
	 */
	@Override
	public void update(WorkingMemoryChange wmc, From from, To to)
			throws IncompatibleAssignmentException {
		if (beliefTypes != null) {
			if (!beliefTypes.contains(from.type))
				throw (new IncompatibleAssignmentException(
						"cannot update with from type " + from.type));
			if (!beliefTypes.contains(to.type))
				throw (new IncompatibleAssignmentException(
						"cannot update with to type " + to.type));
		}
		CASTIndependentFormulaDistributionsBelief<To> toBelief = createToBeliefProxy(to);
		CASTIndependentFormulaDistributionsBelief<From> fromBelief = createFromBeliefProxy(from);

		fillBeliefs(wmc, fromBelief, toBelief);
	}

	protected void fillBeliefs(WorkingMemoryChange wmc,
			CASTIndependentFormulaDistributionsBelief<From> fromBelief,
			CASTIndependentFormulaDistributionsBelief<To> toBelief) {

		try {
			toBelief.setTime(toBelief.getStartTime(), fromBelief.getEndTime());
			toBelief.getContent().putAll(fromBelief.getContent());
			propagatePointers(toBelief, ignoreForPointerPropagationSet);
		} catch (InterruptedException e) {
			logger.error("interrupted", e);
		}
	}

	protected void propagatePointers(
			CASTIndependentFormulaDistributionsBelief<To> toBelief,
			Set<String> ignoreProperties) throws InterruptedException {
		for (Entry<String, FormulaDistribution> entry : toBelief.getContent()
				.entrySet()) {
			// if we have properties to ignore, we do this now!
			if (ignoreProperties != null
					&& ignoreProperties.contains(entry.getKey()))
				continue;
			for (ProbFormula f : entry.getValue().getDistribution()) {
				// propagate any PointerFormulas to PerceptBeliefs to
				// GroundedBeliefs
				if (f.getFormula().get() instanceof PointerFormula) {

					WMPointer wmp = WMPointer.create((Ice.Object) f
							.getFormula().get());
					if (!wmp.getType().equals(CASTUtils.typeName(m_fromCls))) {
						logger
								.debug("ignore WMPointer as it is pointing out of the layer");
						continue;
					}
					WorkingMemoryAddress lookUpGroundedBelief;

					logger.info("checking the type of "
							+ CASTUtils.toString(wmp.getVal())
							+ " in the map to fill " + entry.getKey()
							+ " in type " + toBelief.getType() + ": "
							+ wmp.getType());

					logger.info("have to find the key "
							+ CASTUtils.toString(wmp.getVal())
							+ " in the map to fill " + entry.getKey()
							+ " in type " + toBelief.getType());
					try {
						lookUpGroundedBelief = wm2wmMap.waitFor(wmp.getVal());
						logger.info("update " + entry.getKey() + " formula "
								+ f.getFormula().toString() + " to "
								+ CASTUtils.toString(lookUpGroundedBelief));
						wmp.setVal(lookUpGroundedBelief);
						wmp.setType(CASTUtils.typeName(m_toCls));
						logger.info("new val: " + entry.getKey() + " formula "
								+ f.getFormula().toString());
					} catch (ValueNotAvailableException e) {
						logger.warn("couldn't find value for "
								+ CASTUtils.toString(wmp.getVal()));
					}
				}
			}
		}
	}

	@Override
	public boolean canHandle(From from) {
		if (beliefTypes == null)
			return true;
		else
			return beliefTypes.contains(from.type);
	}

}
