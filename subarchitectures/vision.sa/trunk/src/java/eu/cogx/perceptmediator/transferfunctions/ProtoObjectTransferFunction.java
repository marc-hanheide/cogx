/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.ProtoObject;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author mmarko (copied from VisualObjectTransferFunction and adapted)
 * 
 */
public class ProtoObjectTransferFunction extends
		SimpleDiscreteTransferFunction<ProtoObject, PerceptBelief> {

	public static final String PROTO_OBJECT_ID = "protoObjectID";
	static Logger logger = Logger.getLogger(ProtoObjectTransferFunction.class);

	public ProtoObjectTransferFunction(ManagedComponent component) {
		super(component, logger, PerceptBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, ProtoObject from)
			throws InterruptedException, BeliefException {

		Map<String, Formula> result = new HashMap<String, Formula>();

		result.put(PROTO_OBJECT_ID, PropositionFormula.create(wmc.address.id)
				.getAsFormula());

		// Add the desired view cone if necessary for fine-stereo processing.
		// When there is a desired view cone, a MoveToViewConeCommand must be
		// issued.

		// Unless there is a good reason to change this, the belief structure is
		// that ViewCones point to objects, but the reverse is not true as it is
		// currently unsupported in the translation to planning state

		// for (WorkingMemoryPointer ptr : from.desiredLocations) {
		// result.put("viewcone-ptr", WMPointer.create(ptr).getAsFormula());
		// }

		// add a flag: fine-soi-processed; if not, AnalyzeProtoObjectCommand
		// must be issued
		result.put("fine-soi-processed", DoubleFormula.create(0).getAsFormula());
		// A link to VisualObject ?

		return result;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @seeeu.cogx.perceptmediator.transferfunctions.abstr.
	 * SimpleDiscreteTransferFunction#fillBelief(de.dfki.lt.tr.beliefs.data.
	 * CASTIndependentFormulaDistributionsBelief, cast.cdl.WorkingMemoryChange,
	 * Ice.ObjectImpl)
	 */
	@Override
	protected void fillBelief(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> belief,
			WorkingMemoryChange wmc, ProtoObject from) {
		// TODO Auto-generated method stub
		super.fillBelief(belief, wmc, from);
		IndependentFormulaDistributions distr = belief.getContent();
		FormulaDistribution fd;

		// TODO: Fill the belief

		// fd = FormulaDistribution.create();
		// fd.add((float) from.salience, 1.0);
		// distr.put("salience", fd);

		// fd = FormulaDistribution.create();
		// fd.add(wmc.address.id, 1.0);
		// distr.put(PROTO_OBJECT_ID, fd);

		// fillConcept("color", distr, from.colorLabels, from.colorDistrib,
		// from.colorGains, from.colorGain, from.colorAmbiguity);
		// fillConcept("shape", distr, from.shapeLabels, from.shapeDistrib,
		// from.shapeGains, from.shapeGain, from.shapeAmbiguity);
		// fillConcept("ident", distr, from.identLabels, from.identDistrib,
		// null,
		// from.identGain, from.identAmbiguity);
	}

	// private void fillConcept(String concept,
	// IndependentFormulaDistributions distr, String[] labels,
	// double[] distrib, double[] gains, double gain, double ambiguity) {
	// FormulaDistribution fd = FormulaDistribution.create();
	// double maxGain = -1;
	// String gainStr = "";
	// for (int i = 0; i < labels.length; i++) {
	// fd.add(labels[i], distrib[i]);
	// if (gains != null && i<gains.length) {
	// if (maxGain < gains[i]) {
	// maxGain = gains[i];
	// gainStr = labels[i];
	// }
	// }
	// }
	// if (maxGain > 0) {
	// FormulaDistribution gainFD = FormulaDistribution.create();
	// gainFD.add(gainStr, 1.0);
	// distr.put("max-gain-label-" + concept, gainFD);
	// gainFD = FormulaDistribution.create();
	// gainFD.add((float) maxGain, 1.0);
	// distr.put("max-gain-value-" + concept, gainFD);
	// }
	// distr.put(concept, fd);

	// fd = FormulaDistribution.create();
	// fd.add((float) gain, 1.0);
	// distr.put("gain-" + concept, fd);

	// fd = FormulaDistribution.create();
	// fd.add((float) ambiguity, 1.0);
	// distr.put("ambiguity-" + concept, fd);

	// }
}
