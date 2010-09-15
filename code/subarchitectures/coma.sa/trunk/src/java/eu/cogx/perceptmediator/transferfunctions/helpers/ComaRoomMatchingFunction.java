/**
 * 
 */
package eu.cogx.perceptmediator.transferfunctions.helpers;

import comadata.ComaRoom;

import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.IndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;

/**
 * @author marc
 *
 */
public class ComaRoomMatchingFunction implements ContentMatchingFunction<PerceptBelief> {
	/** the identifier of the RoomId attribute */ 
	public static final String ROOM_ID = ComaRoomTransferFunction.ROOM_ID;

	private long roomId;
	
	/**
	 * @param roomId
	 */
	public ComaRoomMatchingFunction(long roomId) {
		this.roomId = roomId;
	}


	@Override
	public boolean matches(PerceptBelief r) {
		if (r.type.equals(SimpleDiscreteTransferFunction.getBeliefTypeFromCastType(ComaRoom.class))) {
			assert (r.content instanceof CondIndependentDistribs);
			
			IndependentFormulaDistributionsBelief<PerceptBelief> b = IndependentFormulaDistributionsBelief.create(PerceptBelief.class, r);
			FormulaDistribution fv = b.getContent().get(ROOM_ID);
			Formula mostLikelyPlace=fv.getDistribution().getMostLikely();
			int idVal = mostLikelyPlace.getInteger();
			return idVal == roomId;
			
		}
		else {
			return false;
		}
	}
	
}
