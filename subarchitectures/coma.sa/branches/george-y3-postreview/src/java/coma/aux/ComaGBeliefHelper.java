package coma.aux;

import VisionData.VisualObject;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
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
	 * @return most likely category_id (for rooms) or label_id (for visual objects) or empty String if n/a
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
			if (cat==null || cat.equals("")) cat = "room";
		} else if (gbProxy.getType().equals(
				SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class))) {
			cat = gbProxy.getContent().get("label")
			//VisualObjectTransferFunction.LABEL_ID)
			.getDistribution().getMostLikely().getProposition();
			if (cat==null || cat.equals("")) cat = "object";
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
	 * @return coma individual name (w/o namespace prefix!) or empty String if n/a
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
					.getDistribution().getMostLikely().getInteger();
		} else if (gbProxy.getType().equals(
				SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class))) {
			insName = "object" + gbProxy.getId().replace(":","_");
		}
		return insName;		
	}

	/**
	 * Returns the name of the spatial relation that 
	 * holds between the VisualObject GroundedBelief
	 * and another GroundedBelief (can be about a 
	 * ComaRoom or another VisualObject).
	 * 
	 * Currently only handles VisualObject GBeliefs!
	 * 
	 * @param gb - the GroundedBelief
	 * @return label of the spatial relation or empty String if n/a
	 */
	public static String getGBeliefRelation(GroundedBelief gb) {
		// currently only handles VisualObject GBeliefs!
		String relName = "";

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = CASTIndependentFormulaDistributionsBelief
		.create(GroundedBelief.class, gb);

		if (gbProxy.getType().equals(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class))) {
			relName = gbProxy.getContent().get("relation")
				.getDistribution().getMostLikely().getProposition();
		}
		return relName;		
	}

	/**
	 * Returns the related-to GroundedBelief (as WMPointer)
	 * of a VisualObject GroundedBelief.
	 * The related-to (aka relatee) can be a GroundedBelief
	 * about a VisualObject (recursive), or about a ComaRoom.
	 * 
	 * Currently only handles VisualObject GBeliefs!
	 * 
	 * @param gb - the GroundedBelief
	 * @return WMpointer of the related-to GroundedBelief or null if n/a
	 */
	public static WMPointer getGBeliefRelatee(GroundedBelief gb) {
		// currently only handles VisualObject GBeliefs!
		WMPointer relateePtr = null;

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = CASTIndependentFormulaDistributionsBelief
		.create(GroundedBelief.class, gb);

		if (gbProxy.getType().equals(SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class))) {
			relateePtr = WMPointer.create(gbProxy.getContent().get(
				"related-to").getDistribution().getMostLikely().get());
		}
		return relateePtr;		
	}


}
