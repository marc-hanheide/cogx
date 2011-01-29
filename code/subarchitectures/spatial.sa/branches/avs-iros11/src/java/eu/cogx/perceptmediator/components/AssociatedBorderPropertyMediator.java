/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.perceptmediator.components;

import SpatialProperties.AssociatedBorderPlaceholderProperty;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.abstr.PlacePropertyMediator;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class AssociatedBorderPropertyMediator extends
		PlacePropertyMediator<AssociatedBorderPlaceholderProperty> {

	public final static String ASSOCIATEDBORDER_FEATURENAME="associated-border";
	
	public AssociatedBorderPropertyMediator() {
		super(AssociatedBorderPlaceholderProperty.class);

	}

	@Override
	protected boolean fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> create,
			AssociatedBorderPlaceholderProperty from) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add((float) getFirstPropertyValue(from), 1.0);
		create.getContent().put(ASSOCIATEDBORDER_FEATURENAME, fd);
		return true;
	}

}