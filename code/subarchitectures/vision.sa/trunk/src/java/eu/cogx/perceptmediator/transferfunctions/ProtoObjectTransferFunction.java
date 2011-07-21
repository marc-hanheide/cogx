/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.ProtoObject;
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

/**
 * @author mmarko (copied from VisualObjectTransferFunction and adapted)
 * 
 */
public class ProtoObjectTransferFunction extends
		DependentDiscreteTransferFunction<ProtoObject, GroundedBelief> {

	public static final String VISUAL_OBJECT_LINK = "po-is-associated-with";

	static Logger logger = Logger.getLogger(ProtoObjectTransferFunction.class);

	public ProtoObjectTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, logger, GroundedBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, ProtoObject from)
			throws InterruptedException, BeliefException {

		Map<String, Formula> result = new HashMap<String, Formula>();

		// result.put(PROTO_OBJECT_ID, PropositionFormula.create(wmc.address.id)
		// .getAsFormula());

		//only 0 or 1 for optional vo argument
		assert (from.visualObject.length <= 1);

		// if there is a VO associated with this PO
		if (from.visualObject.length == 1) {
			WorkingMemoryAddress voAddr = from.visualObject[0].address;

			logger.debug("trying to resolve the VisualObject belief produced from  "
					+ CASTUtils.toString(voAddr));

			// resolve the address of the proto object this cone targets
			WorkingMemoryAddress voBelAddr = getReferredBelief(new BeliefAncestorMatchingFunction(
					from.visualObject[0]));

			// pointer to belief for proto object
			result.put(
					VISUAL_OBJECT_LINK,
					WMPointer.create(voBelAddr,
							CASTUtils.typeName(GroundedBelief.class))
							.getAsFormula());

		}

		return result;
	}

}
