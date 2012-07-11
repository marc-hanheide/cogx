package motivation.components.generators;

import java.util.HashSet;
import java.util.Map.Entry;
import java.util.Set;
import java.util.Vector;

import motivation.factories.MotiveFactory;
import motivation.slice.HypothesisVerificationMotive;
import VisionData.VisualObject;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.WMBeliefView;
import eu.cogx.beliefs.slice.HypotheticalBelief;
import eu.cogx.perceptmediator.transferfunctions.ConnectivityTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class HypothesisVerificationGoalGenerator
		extends
		AbstractBeliefMotiveGenerator<HypothesisVerificationMotive, HypotheticalBelief> {

	@SuppressWarnings("serial")
	final static Set<String> VALID_ABOUT_FEATURES = new HashSet<String>() {
		{
			add("category");
			// add("label");
		}
	};
	@SuppressWarnings("serial")
	final static Set<String> VALID_RELATION_FEATURES = new HashSet<String>() {
		{
			add("dora__inroom");
			add("dora__inobject");
		}
	};

	WMView<dBelief> beliefs = WMBeliefView.create(
			this,
			dBelief.class,
			new String[] {
					SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(VisualObject.class),
					"room" });

	public HypothesisVerificationGoalGenerator() {
		super("", HypothesisVerificationMotive.class, HypotheticalBelief.class);
	}

	@Override
	protected void start() {
		super.start();
		try {
			beliefs.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	@Override
	protected HypothesisVerificationMotive checkForAddition(
			WorkingMemoryAddress addr, HypotheticalBelief newEntry) {
		HypothesisVerificationMotive hvm = MotiveFactory
				.createHypothesisVerificationMotive(addr);
		CASTIndependentFormulaDistributionsBelief<HypotheticalBelief> hyp = CASTIndependentFormulaDistributionsBelief
				.create(HypotheticalBelief.class, newEntry);
		StringBuilder sb = getGoal(hyp);
		if (sb == null)
			return null;
		// hvm.goal.goalString = "(exists (?o - visualobject) " + sb.toString()
		// + " )";
		hvm.goal.goalString = sb.toString();
		println("the goal string is: " + hvm.goal.goalString);
		return hvm;
	}

	private StringBuilder getGoal(
			CASTIndependentFormulaDistributionsBelief<HypotheticalBelief> hyp) {
		StringBuilder sb = new StringBuilder();
		// if this is a relation
		if (hyp.getType().equals(ConnectivityTransferFunction.LABEL_RELATION)) {
			println("it's a hypothesis about a relation");
			Vector<FormulaDistribution> values = new Vector<FormulaDistribution>();
			values.add(hyp.getContent().get(
					ConnectivityTransferFunction.LABEL_VAL0));
			values.add(hyp.getContent().get(
					ConnectivityTransferFunction.LABEL_VAL1));
			values.add(hyp.getContent().get("val2"));
			sb.append("(and ");
			for (Entry<String, FormulaDistribution> s : hyp.getContent()
					.entrySet()) {
				if (VALID_RELATION_FEATURES.contains(s.getKey())) {
					log("add kval goal for feature " + s.getKey());
					sb.append("(kval '");
					sb.append(this.getRobotBeliefAddr().id);
					sb.append("' (" + s.getKey());
					for (FormulaDistribution fd : values) {
						if (fd != null) {
							sb.append(" "
									+ fd.getDistribution().getMostLikely()
											.toString());
						}
					}
					sb.append("))");
				}
			}
			sb.append(")");
			log("relation goal is " + sb.toString());
		} else {
			FormulaDistribution about = hyp.getContent().get("about");
			if (about != null) {
				println("it's a hypothesis that is about something else");
				PointerFormula aboutPtr = (PointerFormula) about
						.getDistribution().getMostLikely().get();
				sb.append("(and ");
				for (Entry<String, FormulaDistribution> s : hyp.getContent()
						.entrySet()) {
					if (VALID_ABOUT_FEATURES.contains(s.getKey())) {
						log("add kval goal for feature " + s.getKey());
						sb.append("(kval '");
						sb.append(this.getRobotBeliefAddr().id);
						sb.append("' (" + s.getKey() + " '");
						sb.append(aboutPtr.pointer.id);
						sb.append("'))");
					}
				}
				sb.append(")");
			} else {
				println("it's a about something existing");
				sb.append("(kval '");
				sb.append(this.getRobotBeliefAddr().id);
				sb.append("' (entity-exists '");
				sb.append(hyp.getId());
				sb.append("'))");
			}
		}
		return sb;
	}

	@Override
	protected HypothesisVerificationMotive checkForUpdate(
			HypotheticalBelief newEntry, HypothesisVerificationMotive motive) {
		getLogger()
				.warn("we don expect this to be overwritten... so itś not implemented");
		return motive;
	}

}
