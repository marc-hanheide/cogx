/**
 * 
 */
package eu.cogx.perceptmediator;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;
import de.dfki.lt.tr.beliefs.data.Belief;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.factories.IndependentDistributionFactory;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author marc
 * 
 */
public class VisualObjectTransferFunction implements
		TransferFunction<VisualObject, PerceptBelief> {

	Logger logger = Logger.getLogger(VisualObjectTransferFunction.class);

	public VisualObjectTransferFunction(ManagedComponent component) {
	}

	@Override
	public PerceptBelief create(WorkingMemoryAddress idToCreate,
			WorkingMemoryChange wmc, VisualObject from) {
		CASTIndependentFormulaDistributionsBelief<PerceptBelief> pb = CASTIndependentFormulaDistributionsBelief
				.create(PerceptBelief.class);
		pb.setId(idToCreate.id);
		pb.setType(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(wmc.type));
		pb.setPrivate("robot");

		return pb.get();
	}

	@Override
	public boolean transform(WorkingMemoryChange wmc, VisualObject from,
			PerceptBelief to) {
		Belief<PerceptBelief> belief = Belief.create(PerceptBelief.class, to);
		IndependentDistribution distr = belief
				.getContent(IndependentDistributionFactory.get());
		FormulaDistribution fd;

		fd = FormulaDistribution.create();
		fd.add(from.label, from.labelConfidence);
		distr.put("label", fd.asDistribution());

		fd = FormulaDistribution.create();
		fd.add((float) from.salience, 1.0);
		distr.put("salience", fd.asDistribution());

		{ // color
			fd = FormulaDistribution.create();
			for (int i = 0; i < from.colorLabels.length; i++) {
				fd.add(from.colorLabels[i], from.colorDistrib[i]);
			}
			distr.put("color", fd.asDistribution());

			fd = FormulaDistribution.create();
			fd.add((float) from.colorGain, 1.0);
			distr.put("colorGain", fd.asDistribution());

			fd = FormulaDistribution.create();
			fd.add((float) from.colorAmbiguity, 1.0);
			distr.put("colorAmbiguity", fd.asDistribution());

		}
		{ // shape
			fd = FormulaDistribution.create();
			for (int i = 0; i < from.shapeLabels.length; i++) {
				fd.add(from.shapeLabels[i], from.shapeDistrib[i]);
			}
			distr.put("shape", fd.asDistribution());

			fd = FormulaDistribution.create();
			fd.add((float) from.shapeGain, 1.0);
			distr.put("shapeGain", fd.asDistribution());

			fd = FormulaDistribution.create();
			fd.add((float) from.shapeAmbiguity, 1.0);
			distr.put("shapeAmbiguity", fd.asDistribution());
		}
		return true;
	}

}
