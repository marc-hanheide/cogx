/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.perceptmediator.components;

import SpatialProperties.GatewayPlaceProperty;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.components.abstr.PlacePropertyMediator;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class GatewayPropertyMediator extends
		PlacePropertyMediator<GatewayPlaceProperty> {

	public GatewayPropertyMediator() {
		super(GatewayPlaceProperty.class);

	}

	@Override
	protected boolean fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> create,
			GatewayPlaceProperty from) {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(from.mapValueReliable, 1.0);
		create.getContent().put("gateway", fd);
		return true;
	}

}