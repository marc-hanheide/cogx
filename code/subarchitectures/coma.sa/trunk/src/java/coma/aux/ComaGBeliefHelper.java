package coma.aux;

import VisionData.VisualObject;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public final class ComaGBeliefHelper {

	
	/**
	 * Returns the most likely (according to its probability distribution) 
	 * category of the entity the grounded belief is about. 
	 * Currently only handles VisualObject and ComaRoom GBeliefs!
	 * 
	 * @param gb - the GroundedBelief
	 * @return most likely category_id (for rooms) or label_id (for visual objects) 
	 */
	public static String getGBeliefCategory(GroundedBelief gb) {
		String cat = "";
		
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = CASTIndependentFormulaDistributionsBelief
        .create(GroundedBelief.class, gb);
		
		if (gbProxy.getType().equals(
                SimpleDiscreteTransferFunction
                        .getBeliefTypeFromCastType(ComaRoom.class))) {
            cat = gbProxy.getContent().get(
                    ComaRoomTransferFunction.CATEGORY_ID)
                    .getDistribution().getMostLikely().getProposition();
        } else if (gbProxy.getType().equals(
                SimpleDiscreteTransferFunction
                        .getBeliefTypeFromCastType(VisualObject.class))) {
            cat = gbProxy.getContent().get("label")
                    //VisualObjectTransferFunction.LABEL_ID)
                    .getDistribution().getMostLikely().getProposition();
        }
		return cat;
	}
	
	/**
	 * Returns the local name (i.e., without namespace prefix) 
	 * for an individual the grounded belief is about.
	 * E.g., "room0" or "object123"
	 * Currently only handles VisualObject and ComaRoom GBeliefs!
	 * 
	 * @param gb - the GroundedBelief
	 * @return coma individual name (w/o namespace prefix!)
	 */
	public static String getGBeliefComaIndividualName(GroundedBelief gb) {
		// currently only handles VisualObject and ComaRoom GBeliefs!
		String insName = "";
		
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = CASTIndependentFormulaDistributionsBelief
        .create(GroundedBelief.class, gb);
		
		if (gbProxy.getType().equals(
                SimpleDiscreteTransferFunction
                        .getBeliefTypeFromCastType(ComaRoom.class))) {
            insName = "room" + gbProxy.getContent().get(
                    ComaRoomTransferFunction.ROOM_ID)
                    .getDistribution().getMostLikely().getProposition();
        } else if (gbProxy.getType().equals(
                SimpleDiscreteTransferFunction
                        .getBeliefTypeFromCastType(VisualObject.class))) {
            insName = "object" + gbProxy.getId();
        }
		return insName;		
	}
	
	
}
