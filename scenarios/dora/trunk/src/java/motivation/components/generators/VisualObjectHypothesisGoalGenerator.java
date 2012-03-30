package motivation.components.generators;

import java.util.Map.Entry;

import motivation.factories.MotiveFactory;
import motivation.slice.HypothesisVerificationMotive;
import VisionData.VisualObject;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.WMBeliefView;
import eu.cogx.beliefs.slice.HypotheticalBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class VisualObjectHypothesisGoalGenerator
		extends
		AbstractBeliefMotiveGenerator<HypothesisVerificationMotive, HypotheticalBelief> {

	WMView<dBelief> beliefs = WMBeliefView.create(
			this,
			dBelief.class,
			new String[] {
					SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(VisualObject.class),
					SimpleDiscreteTransferFunction
							.getBeliefTypeFromCastType(ComaRoom.class) });

	public VisualObjectHypothesisGoalGenerator() {
		super(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class),
				HypothesisVerificationMotive.class, HypotheticalBelief.class);
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
		if (sb==null)
			return null;
		hvm.goal.goalString = "(exists (?o - visualobject) " + sb.toString()
				+ " )";
		println("the goal string is: " + hvm.goal.goalString);
		return hvm;
	}

	private StringBuilder getGoal(
			CASTIndependentFormulaDistributionsBelief<HypotheticalBelief> hyp) {
		StringBuilder sb = new StringBuilder();
		sb.append("(kval '");
		sb.append(this.getRobotBeliefAddr().id);
		sb.append("' (entity-exists '");
		sb.append(hyp.getId());
		sb.append("'))");
		return sb;
	}
	
//	private StringBuilder getGoal(
//			CASTIndependentFormulaDistributionsBelief<HypotheticalBelief> hyp) {
//		StringBuilder sb = new StringBuilder();
//		sb.append("(and ");
//		for (Entry<String, FormulaDistribution> entry : hyp.getContent()
//				.entrySet()) {
//			String conj = createConjunct(entry);
//			if (conj == null) {
//				getLogger().warn(
//						"cannot create complete goal for HypotheticalBelief as a referred (for '"
//								+ entry.getKey()
//								+ "') belief could not be resolved");
//				return null;
//			} else {
//				sb.append(conj + " ");
//				log("  added conjunct in hypothesis: " + conj);
//			}
//		}
//		sb.append(")");
//		return sb;
//	}

	

	private String createConjunct(Entry<String, FormulaDistribution> entry) {
		dFormula df = entry.getValue().getDistribution().getMostLikely().get();
		String c = "";
		if (df instanceof PointerFormula) {
			PointerFormula pf = (PointerFormula) df;
			if (beliefs.containsKey(pf.pointer)) {
				c = "(= (" + entry.getKey() + " ?o) " + "'" + pf.pointer.id + "')";
			} else {
				c = null;
			}
		} else if (df instanceof BooleanFormula) {
			BooleanFormula pf = (BooleanFormula) df;
			if (pf.val) {
				c = "(" + entry.getKey() + "  ?o)";
			} else {
				c = "(not (" + entry.getKey() + "  ?o))";
			}
		} else if (df instanceof ElementaryFormula) {
			ElementaryFormula pf = (ElementaryFormula) df;
			c = "(= (" + entry.getKey() + " ?o) " + pf.prop + ")";
		}
		return c;
	}

	@Override
	protected HypothesisVerificationMotive checkForUpdate(
			HypotheticalBelief newEntry, HypothesisVerificationMotive motive) {
		getLogger()
				.warn("we don expect this to be overwritten... so itś not implemented");
		return motive;
	}

}
