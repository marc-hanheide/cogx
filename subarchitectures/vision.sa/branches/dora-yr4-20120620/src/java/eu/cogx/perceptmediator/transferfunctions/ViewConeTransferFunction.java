/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.ViewCone;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.DoubleFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.BeliefAncestorMatchingFunction;

/**
 * @author mmarko and nah (copied from VisualObjectTransferFunction and adapted)
 * 
 */
public class ViewConeTransferFunction extends
		DependentDiscreteTransferFunction<ViewCone, GroundedBelief> {

	static Logger logger = Logger.getLogger(ViewConeTransferFunction.class);

//	private final WMView<ViewCone> m_allCones;

	public ViewConeTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, logger, GroundedBelief.class);
//		m_allCones = WMView.create(component, ViewCone.class);
//		try {
//			m_allCones.start();
//		} catch (UnknownSubarchitectureException e) {
//			logger.error("Can't start WMView", e);
//		}
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, ViewCone from)
			throws InterruptedException, BeliefException {

		Map<String, Formula> result = new HashMap<String, Formula>();

		if (from.target != null) {

			logger.debug("trying to resolve the belief produced from  "
					+ CASTUtils.toString(from.target.address));

			// resolve the address of the proto object this cone targets
			WorkingMemoryAddress poBelAddr = getReferredBelief(new BeliefAncestorMatchingFunction(
					from.target));

			// pointer to belief for proto object
			result.put(
					"target-object",
					WMPointer.create(poBelAddr,
							CASTUtils.typeName(GroundedBelief.class))
							.getAsFormula());
		}

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
		result.put("tilt", DoubleFormula.create(from.tilt).getAsFormula());

		return result;
	}

}
