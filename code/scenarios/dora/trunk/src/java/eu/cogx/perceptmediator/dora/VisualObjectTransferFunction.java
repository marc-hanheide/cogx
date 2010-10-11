/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentLinkingDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.AgentMatchingFunction;

/**
 * @author marc
 * 
 */
public class VisualObjectTransferFunction
		extends
		DependentLinkingDiscreteTransferFunction<VisualObject, PerceptBelief, GroundedBelief> {

	public static final String LABEL_ID = "label";
	public static final String IS_IN = "is-in";

	public VisualObjectTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(VisualObjectTransferFunction.class),
				PerceptBelief.class);
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from) throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		// TODO: we should use a DoubleValue here!
		try {
			result.put(LABEL_ID, PropositionFormula.create(from.identLabels[0])
					.getAsFormula());
		} catch (BeliefException e) {
			component.logException(e);
		}

		return result;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see eu.cogx.perceptmediator.transferfunctions.abstr.
	 * SimpleDiscreteTransferFunction#fillBelief(de.dfki.lt.tr.beliefs.data.
	 * CASTIndependentFormulaDistributionsBelief, cast.cdl.WorkingMemoryChange,
	 * Ice.ObjectImpl)
	 */
	@Override
	protected void fillBelief(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> belief,
			WorkingMemoryChange wmc, VisualObject from) {
		FormulaDistribution fd = FormulaDistribution.create();
		Place currentPlace;
		try {
			WorkingMemoryAddress agentWMA = getReferredBelief(new AgentMatchingFunction(
					0));
			CASTIndependentFormulaDistributionsBelief<dBelief> agent = CASTIndependentFormulaDistributionsBelief
					.create(dBelief.class, allBeliefs.get(agentWMA));
			Formula place=agent.getContent().get(LocalizedAgentTransferFunction.IS_IN)
					.getDistribution().getMostLikely();
//			currentPlace = SpatialFacade.get(component).getPlace();
//			component.log("I am at place " + currentPlace.id
//					+ " when I found that object");
//			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
//					currentPlace.id));
//			WMPointer wmp = WMPointer.create(placeWMA, CASTUtils
//					.typeName(GroundedBelief.class));
			component.log("size of from.identDistrib: "
					+ from.identDistrib.length);
			fd.add(place.get(), from.identDistrib[0]);
			belief.getContent().put(IS_IN, fd);
//		} catch (CASTException e) {
//			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		} catch (BeliefException e) {
			component.logException(e);
		}
	}
}
