/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.abstr;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialData.Place;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import facades.SpatialFacade;

/**
 * a specialized {@link DependentDiscreteTransferFunction} that is used to
 * establish a relation between a perceived entity and the current location of
 * the robot.
 * 
 * @author marc
 * 
 * @param <From>
 *            the type to generate percepts from
 */
public abstract class LocalizedRelationTransferFunction<From extends Ice.ObjectImpl, To extends dBelief>
		extends DependentDiscreteTransferFunction<From, To> {

	/**
	 * constructor
	 * 
	 * @see DependentDiscreteTransferFunction
	 * @param component
	 * @param allBeliefs
	 * @param logger
	 */
	public LocalizedRelationTransferFunction(ManagedComponent component,
			WMView<To> allBeliefs, Logger logger, Class<To> beliefClass) {
		super(component, allBeliefs, logger, beliefClass);

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction
	 * #create(cast.cdl.WorkingMemoryAddress, cast.cdl.WorkingMemoryChange,
	 * Ice.ObjectImpl)
	 */
	@Override
	public To create(WorkingMemoryAddress newAddr,
			WorkingMemoryChange wmc, From from) {
		To bel = super.create(newAddr, wmc, from);
		bel.type = "relation";
		return bel;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction
	 * #getFeatureValueMapping(cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, From from) throws InterruptedException,
			BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		try {
			Place place;
			place = SpatialFacade.get(component).getPlace();
			WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
					place.id));
			WorkingMemoryAddress wmaFrom = getReferredBelief(getMatchingFunction(wmc.address.id));
			result
					.put("is-in", BoolFormula.create(true).getAsFormula());
			result.put("val0", WMPointer.create(wmaFrom, CASTUtils.typeName(this.beliefClass)).getAsFormula());
			result.put("val1", WMPointer.create(wmaPlace, CASTUtils.typeName(this.beliefClass)).getAsFormula());

		} catch (CASTException e) {
			component
					.logException(
							"CASTException when mapping features, not all features might have been assigned",
							e);
		}
		return result;
	}

	/**
	 * abstract method that returns the {@link ContentMatchingFunction} to be
	 * used
	 * 
	 * @param id
	 *            the WM id of the referred entry (to be waited for by
	 *            {@link WMContentWaiter})
	 * @return an instance of a {@link ContentMatchingFunction} that is used for
	 *         matching
	 */
	protected abstract ContentMatchingFunction<? super To> getMatchingFunction(
			String id);

}
