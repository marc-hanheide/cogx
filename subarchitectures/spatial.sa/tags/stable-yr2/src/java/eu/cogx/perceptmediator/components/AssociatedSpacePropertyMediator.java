/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.perceptmediator.components;

import SpatialProperties.AssociatedSpacePlaceholderProperty;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.abstr.PlacePropertyMediator;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class AssociatedSpacePropertyMediator extends
		PlacePropertyMediator<AssociatedSpacePlaceholderProperty> {

	public final static String ASSOCIATEDSPACE_FEATURENAME="associated-space";

	public AssociatedSpacePropertyMediator() {
		super(AssociatedSpacePlaceholderProperty.class);

	}

	@Override
	protected boolean fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> create,
			AssociatedSpacePlaceholderProperty from) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add((float) getFirstPropertyValue(from), 1.0);
		create.getContent().put(ASSOCIATEDSPACE_FEATURENAME, fd);
		return true;
	}

}