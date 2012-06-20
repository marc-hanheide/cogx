/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
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
 * @author marc
 * 
 */
public class VisualObjectTransferFunction extends
		SimpleDiscreteTransferFunction<VisualObject, PerceptBelief> {

	public static final String VISUAL_OBJECT_ID = "protoObjectID";
	static Logger logger = Logger.getLogger(VisualObjectTransferFunction.class);

	public VisualObjectTransferFunction(ManagedComponent component) {
		super(component, logger, PerceptBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from)
			throws InterruptedException, BeliefException {
		Map<String, Formula> result = new HashMap<String, Formula>();

		result.put("salience", DoubleFormula.create(from.salience)
				.getAsFormula());
		result.put(VISUAL_OBJECT_ID, PropositionFormula.create(wmc.address.id)
				.getAsFormula());

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
			WorkingMemoryChange wmc, VisualObject from) {
		// TODO Auto-generated method stub
		super.fillBelief(belief, wmc, from);
		IndependentFormulaDistributions distr = belief.getContent();
		FormulaDistribution fd;

		// The status of the VO: unknow, visible, was_visible, removed
		String status;
		fd = FormulaDistribution.create();
		switch(from.presence) {
			case VopVISIBLE:
			 	status = "visible";
			 	break;
			case VopWasVISIBLE:
			 	status = "was-visible";
			 	break;
			case VopREMOVED:
			 	status = "removed";
			 	break;
			default:
			 	status = "unknown";
			 	break;
		}
		fd.add(status, 1.0);
		distr.put("presence", fd);

		fd = FormulaDistribution.create();
		fd.add((float) from.salience, 1.0);
		distr.put("salience", fd);

		fd = FormulaDistribution.create();
		fd.add(wmc.address.id, 1.0);
		distr.put(VISUAL_OBJECT_ID, fd);

		fillConcept("color", distr, from.colorLabels, from.colorDistrib,
				from.colorGains, from.colorGain, from.colorAmbiguity);
		fillConcept("shape", distr, from.shapeLabels, from.shapeDistrib,
				from.shapeGains, from.shapeGain, from.shapeAmbiguity);
		fillConcept("ident", distr, from.identLabels, from.identDistrib, null,
				from.identGain, from.identAmbiguity);
		// { // color
		// fd = FormulaDistribution.create();
		// for (int i = 0; i < from.colorLabels.length; i++) {
		// fd.add(from.colorLabels[i], from.colorDistrib[i]);
		// FormulaDistribution gainFD = FormulaDistribution.create();
		// gainFD.add((float) from.colorGains[i], 1.0);
		// distr.put("gain-color-" + from.colorLabels[i], gainFD);
		// }
		// distr.put("color", fd);
		//
		// fd = FormulaDistribution.create();
		// fd.add((float) from.colorGain, 1.0);
		// distr.put("colorGain", fd);
		//
		// fd = FormulaDistribution.create();
		// fd.add((float) from.colorAmbiguity, 1.0);
		// distr.put("colorAmbiguity", fd);
		//
		// }
		// { // shape
		// fd = FormulaDistribution.create();
		// for (int i = 0; i < from.shapeLabels.length; i++) {
		// fd.add(from.shapeLabels[i], from.shapeDistrib[i]);
		// }
		// distr.put("shape", fd);
		//
		// fd = FormulaDistribution.create();
		// fd.add((float) from.shapeGain, 1.0);
		// distr.put("shapeGain", fd);
		//
		// fd = FormulaDistribution.create();
		// fd.add((float) from.shapeAmbiguity, 1.0);
		// distr.put("shapeAmbiguity", fd);
		// }
		// { // ident
		// fd = FormulaDistribution.create();
		// for (int i = 0; i < from.identLabels.length; i++) {
		// fd.add(from.identLabels[i], from.identDistrib[i]);
		// }
		// distr.put("ident", fd);
		//
		// fd = FormulaDistribution.create();
		// fd.add((float) from.identGain, 1.0);
		// distr.put("identGain", fd);
		//
		// fd = FormulaDistribution.create();
		// fd.add((float) from.identAmbiguity, 1.0);
		// distr.put("identAmbiguity", fd);
		// }
	}

	private void fillConcept(String concept,
			IndependentFormulaDistributions distr, String[] labels,
			double[] distrib, double[] gains, double gain, double ambiguity) {
		FormulaDistribution fd = FormulaDistribution.create();
		double maxGain = -1;
		String gainStr = "";
		for (int i = 0; i < labels.length; i++) {
			fd.add(labels[i], distrib[i]);
			if (gains != null && i<gains.length) {
				if (maxGain < gains[i]) {
					maxGain = gains[i];
					gainStr = labels[i];
				}
			}
		}
		if (maxGain > 0) {
			FormulaDistribution gainFD = FormulaDistribution.create();
			gainFD.add(gainStr, 1.0);
			distr.put("max-gain-label-" + concept, gainFD);
			gainFD = FormulaDistribution.create();
			gainFD.add((float) maxGain, 1.0);
			distr.put("max-gain-value-" + concept, gainFD);
		}
		distr.put(concept, fd);

		fd = FormulaDistribution.create();
		fd.add((float) gain, 1.0);
		distr.put("gain-" + concept, fd);

		fd = FormulaDistribution.create();
		fd.add((float) ambiguity, 1.0);
		distr.put("ambiguity-" + concept, fd);

	}
}
