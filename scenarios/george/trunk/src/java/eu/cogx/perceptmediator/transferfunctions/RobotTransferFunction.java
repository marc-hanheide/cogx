/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.BeliefAncestorMatchingFunction;
import execution.slice.Robot;

/**
 * @author nah
 * 
 */
public class RobotTransferFunction extends
		DependentDiscreteTransferFunction<Robot, GroundedBelief> {

	public static final String CURRENT_VIEWCONE_ID = "current-viewcone";
	static Logger logger = Logger.getLogger(RobotTransferFunction.class);

	public RobotTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, logger, GroundedBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, Robot from) throws InterruptedException,
			BeliefException {

		Map<String, Formula> result = new HashMap<String, Formula>();
		if (from.currentViewCone != null) {
			// resolve the address of the proto object this cone targets
			WorkingMemoryAddress vcBelAddr = getReferredBelief(new BeliefAncestorMatchingFunction(
					from.currentViewCone));
			// pointer to belief for proto object
			result.put(
					CURRENT_VIEWCONE_ID,
					WMPointer.create(vcBelAddr,
							CASTUtils.typeName(GroundedBelief.class))
							.getAsFormula());
		}
		return result;
	}

}
