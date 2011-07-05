/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.ProtoObject;
import VisionData.ViewCone;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributions;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

/**
 * @author mmarko (copied from VisualObjectTransferFunction and adapted)
 * 
 */
public class ViewConeTransferFunction extends
		SimpleDiscreteTransferFunction<ViewCone, PerceptBelief> {

	public static final String VIEW_CONE_ID = "viewConeID";
	static Logger logger = Logger.getLogger(ViewConeTransferFunction.class);

	public ViewConeTransferFunction(ManagedComponent component) {
		super(component, logger, PerceptBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, ViewCone from) throws InterruptedException,
			BeliefException {

		logger.info("Called with " + CASTUtils.toString(wmc));

		Map<String, Formula> result = new HashMap<String, Formula>();

		result.put(VIEW_CONE_ID, PropositionFormula.create(wmc.address.id)
				.getAsFormula());

		result.put("anchor-x", DoubleFormula.create(from.anchor.x)
				.getAsFormula());
		result.put("anchor-y", DoubleFormula.create(from.anchor.y)
				.getAsFormula());
		result.put("anchor-theta", DoubleFormula.create(from.anchor.z)
				.getAsFormula());
		result.put("dx", DoubleFormula.create(from.x).getAsFormula());
		result.put("dy", DoubleFormula.create(from.y).getAsFormula());
		result.put("dtheta", DoubleFormula.create(from.viewDirection)
				.getAsFormula());
		result.put("tilt", DoubleFormula.create(from.tilt)
				.getAsFormula());

		return result;
	}

}
