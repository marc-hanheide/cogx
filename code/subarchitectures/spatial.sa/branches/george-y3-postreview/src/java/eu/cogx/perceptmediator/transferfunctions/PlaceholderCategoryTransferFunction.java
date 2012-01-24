package eu.cogx.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import SpatialProperties.RoomCategoryPlaceholderProperty;
import SpatialProperties.*;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.formulas.BoolFormula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

public class PlaceholderCategoryTransferFunction extends
		DependentDiscreteTransferFunction<RoomCategoryPlaceholderProperty, GroundedBelief> {

	static final String ATTR_POSSIBLE_CAT = "leads_to_room";

	/**
	 * @param GroundedBeliefs
	 */
	public PlaceholderCategoryTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> GroundedBeliefs) {
		super(component, GroundedBeliefs, Logger
				.getLogger(PlaceholderCategoryTransferFunction.class), GroundedBelief.class);

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @seebinder.components.perceptmediator.transferfunctions.abstr.
	 * SimpleDiscreteTransferFunction#create(cast.cdl.WorkingMemoryAddress,
	 * cast.cdl.WorkingMemoryChange, Ice.ObjectImpl)
	 */
	@Override
	public GroundedBelief create(WorkingMemoryAddress newAddr,
			WorkingMemoryChange wmc, RoomCategoryPlaceholderProperty from) {
		GroundedBelief bel = super.create(newAddr, wmc, from);
		bel.type = "relation";
		return bel;
	}

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, final RoomCategoryPlaceholderProperty from)
			throws InterruptedException, BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();

        log("getting belief for: " + from.placeId);
		WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
				from.placeId));

        if (wmaPlace != null) {
            log("got referred belief: " + wmaPlace.id);
            String category = from.category;
        
            result.put("val0", WMPointer.create(wmaPlace, CASTUtils.typeName(this.beliefClass)).getAsFormula());
            result.put("val1", PropositionFormula.create(category).getAsFormula());
        }

		return result;
	}

	@Override
    protected void fillBelief(CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief,  WorkingMemoryChange wmc,
                                  RoomCategoryPlaceholderProperty from)  {
        
        double prob = 0;

        DiscreteProbabilityDistribution dist = (DiscreteProbabilityDistribution) from.distribution;
        
        for (ValueProbabilityPair pair : dist.data) {
            if (!(pair.value instanceof IntegerValue)) {
                    continue;
                }
                if (((IntegerValue) pair.value).value == 1) {
                    prob = pair.probability;
                }
        }
		FormulaDistribution fd = FormulaDistribution.create();
        fd.add(BoolFormula.create(true).get(), prob);
        belief.getContent().put(ATTR_POSSIBLE_CAT, fd);

    }

}
