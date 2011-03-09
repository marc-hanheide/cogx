/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.LocalizedAgentTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentLinkingDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.AgentMatchingFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import execution.slice.TriBool;
import execution.slice.actions.ProcessConeGroup;

/**
 * @author marc
 * 
 */
public class VisualObjectTransferFunction
		extends
		DependentLinkingDiscreteTransferFunction<VisualObject, PerceptBelief, GroundedBelief> {

	private static final String PLANNER_SA = "planner.sa";
	public static final String LABEL_ID = "label";
	public static final String IS_IN = "is-in";
	private static final double BLOODY_THRESHOLD_ACCORDING_TO_MICHI = 0.08;

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
		FormulaDistribution fd1 = FormulaDistribution.create();
		FormulaDistribution fd2 = FormulaDistribution.create();
		FormulaDistribution fd3 = FormulaDistribution.create();
		try {
			log("check if we have pending cone actions");
//			Formula place = null;
//			long placeID;
			List<ProcessConeGroup> coneActions = new ArrayList<ProcessConeGroup>();
			component.getMemoryEntries(ProcessConeGroup.class, coneActions,
					PLANNER_SA);
			for (ProcessConeGroup pca : coneActions) {
				if (pca.success == TriBool.TRIINDETERMINATE) { // this guy is
					// currently
					// executed
				  	WorkingMemoryAddress coneGroupAddress = pca.coneGroupBeliefID;

//					CASTIndependentFormulaDistributionsBelief<GroundedBelief> coneGroupBelief = getMemoryEntry(coneGroupAddress, "binder.sa");
					CASTIndependentFormulaDistributionsBelief<GroundedBelief> coneGroupBelief = CASTIndependentFormulaDistributionsBelief
					  .create(GroundedBelief.class, allBeliefs.get(coneGroupAddress));

//					WorkingMemoryAddress coneGroupBelief = getReferredBelief(new PlaceMatchingFunction(
//							pca.placeID));
					fd1 = coneGroupBelief.getContent().get("location");
					fd2 = coneGroupBelief.getContent().get("relation");
					fd3 = coneGroupBelief.getContent().get("target");

					log("we found the cone group that belongs to this cone: "
							+ CASTUtils.toString(coneGroupAddress));
//					placeID = pca.placeID;
//					place = WMPointer
//							.create(
//									placeBelief,
//									SimpleDiscreteTransferFunction
											//.getBeliefTypeFromCastType(GroundedBelief.class))
//							.getAsFormula();
//					log("we found the WMPointer that belongs to this cone: "
//							+ place.toString());
					break;
				}
			}

//			if (place == null) { // if we haven't found that object through a
//				// cone action, find it by looking where we are now
//				log("try to find out where the agent is...");
//				WorkingMemoryAddress agentWMA = getReferredBelief(new AgentMatchingFunction(
//						0));
//				CASTIndependentFormulaDistributionsBelief<dBelief> agent = CASTIndependentFormulaDistributionsBelief
//						.create(dBelief.class, allBeliefs.get(agentWMA));
//				place = agent.getContent().get(
//						LocalizedAgentTransferFunction.IS_IN).getDistribution()
//						.getMostLikely();
//				placeID = place.getInteger();
//				// currentPlace = SpatialFacade.get(component).getPlace();
//				// component.log("I am at place " + currentPlace.id
//				// + " when I found that object");
//				// WorkingMemoryAddress placeWMA = getReferredBelief(new
//				// PlaceMatchingFunction(
//				// currentPlace.id));
//				// WMPointer wmp = WMPointer.create(placeWMA, CASTUtils
//				// .typeName(GroundedBelief.class));
//			}
//			assert (place != null);
			component.log("size of from.identDistrib: "
					+ from.identDistrib.length);

//			// OK, let's try to find the room this place is in
//			vector<comadata.ComaRoomPtr> comarooms;
//
//			getMemoryEntries<comadata::ComaRoom> (comarooms, "coma");
//			comadata.ComaRoom roomContainingPlace;
//
//			for (comadata.ComaRoom room : comarooms) {
//			  for (long pid : room.containedPlaceIds) {
//			    if (pid == placeID) {
//			      roomContainingPlace = room;
//			      break;
//			    }
//			  }
//			}




//			fd
//					.add(
//							place.get(),
//							from.identDistrib[0] > BLOODY_THRESHOLD_ACCORDING_TO_MICHI ? 1.0
//									: 0.0);
//			belief.getContent().put(IS_IN, fd);
			belief.getContent().put(IS_IN, fd1);
			belief.getContent().put("relation", fd2);
			belief.getContent().put(LABEL_ID, fd3);
			// } catch (CASTException e) {
			// component.logException(e);
//		} catch (InterruptedException e) {
//			component.logException(e);
		} catch (BeliefException e) {
			component.logException(e);
		} catch (UnknownSubarchitectureException e) {
			component.logException(e);
		}
	}
}
