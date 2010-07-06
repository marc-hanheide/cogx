/**
 * 
 */
package eu.cogx.perceptmediator;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;

import VisionData.VisualObject;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import facades.SpatialFacade;

/**
 * @author marc
 * 
 */
public class LocalizedObjectTransferFunction extends
		DependentDiscreteTransferFunction<VisualObject> {

	public LocalizedObjectTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(LocalizedObjectTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from) throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		// TODO: we should use a DoubleValue here!
		try {
			Place currentPlace = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					currentPlace.id));
			result.put("ObjectId", PropositionFormula.create(wmc.address.id).getAsFormula());
			result.put("label", PropositionFormula.create(from.label).getAsFormula());
			result.put("is-in", WMPointer.create(placeWMA).getAsFormula());
		} catch (BeliefException e) {
			component.logException(e);
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}

		return result;
	}

}
