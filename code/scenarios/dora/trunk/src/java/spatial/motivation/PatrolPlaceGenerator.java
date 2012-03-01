package spatial.motivation;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.PatrolMotive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import autogen.Planner.Goal;
import cast.CASTException;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import castutils.CASTTimeUtil;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import facades.SpatialFacade;

public class PatrolPlaceGenerator extends
		AbstractBeliefMotiveGenerator<PatrolMotive, GroundedBelief> {
	private static final long EXP_NORM = 30000;
	private static final String PLACETYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(Place.class));
	private static final int MAX_EXECUTION_TIME = 60 * 5;
	private static final int MAX_PLANNING_TIME = 30;

	/**
	 * the maximum costs to drop we assign if information gain is really high
	 * (~1) in seconds
	 */
	private static final double MAX_COSTS_TO_DROP = 50;

	public PatrolPlaceGenerator() {
		super(PLACETYPE, PatrolMotive.class, GroundedBelief.class);
	}

	@Override
	protected PatrolMotive checkForAddition(WorkingMemoryAddress adr,
			GroundedBelief newEntry) {
		assert (newEntry.type.equals(PLACETYPE));
		log("checkForAddition(): check belief " + newEntry.id + " for addition");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		// get the most likely status
		boolean isExplored = belief.getContent().get("placestatus")
				.getDistribution().getMostLikely().getProposition()
				.equalsIgnoreCase(PlaceStatus.TRUEPLACE.name());
		log("checkForAddition(): placestatus="
				+ belief.getContent().get("placestatus").getDistribution()
						.getMostLikely().getProposition());
		// if that is a place holder
		if (isExplored) {
			log("place is explored, so it is a goal to patrol");
			PatrolMotive result = new PatrolMotive();
			result.created = getCASTTime();
			result.maxExecutionTime = MAX_EXECUTION_TIME;
			result.maxPlanningTime = MAX_PLANNING_TIME;
			result.priority = MotivePriority.UNSURFACE;
			result.referenceEntry = adr;
			result.status = MotiveStatus.UNSURFACED;
			fillValues(belief, result);
			return result;
		}
		return null;
	}

	@Override
	protected PatrolMotive checkForUpdate(GroundedBelief newEntry,
			PatrolMotive motive) {
		assert (newEntry.type.equals(PLACETYPE));
		log("check goal " + CASTUtils.toString(motive.thisEntry)
				+ " for update");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		boolean isExplored = belief.getContent().get("placestatus")
				.getDistribution().getMostLikely().getProposition()
				.equalsIgnoreCase(PlaceStatus.TRUEPLACE.name());
		// if that is a place holder
		if (isExplored) {
			log("place is explored, so it is a goal to patrol");
			fillValues(belief, motive);
			return motive;
		} else {
			return null;
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.components.generators.AbstractBeliefMotiveGenerator#start()
	 */
	@Override
	protected void start() {
		super.start();
		try {
			SpatialFacade.get(this).registerPlaceChangedCallback(
					new SpatialFacade.PlaceChangedHandler() {
						@Override
						public synchronized void update(Place p) {
							log("explicitly scheduling all motives to be checked due to place change. new place is "
									+ p.id);
							recheckAllMotives();
						}
					});
		} catch (CASTException e1) {
			println("exception when registering placeChangedCallbacks");
			e1.printStackTrace();
		}

	}

	private void fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief,
			PatrolMotive motive) {
		motive.updated = getCASTTime();

		FormulaDistribution pIdFD = belief.getContent().get(
				PlaceTransferFunction.PLACE_ID_ID);
		motive.placeID = pIdFD.getDistribution().getMostLikely().getInteger();
		Place currentPlace;
		try {
			currentPlace = SpatialFacade.get(this).getPlace();

			if (currentPlace != null) {
				log("we are currently at place " + currentPlace.id);
				// if we currently are at the place of this motive we update
				// the last visited time stamp
				CASTTime now = this.getCASTTime();
				if (((PatrolMotive) motive).lastVisisted == null)
					((PatrolMotive) motive).lastVisisted = now;
				else {
					if (currentPlace.id == ((PatrolMotive) motive).placeID) {
						((PatrolMotive) motive).lastVisisted = now;
						// reset the number of tries to zero, as we succeeded to
						// patrol here
						motive.tries = 0;
					}
				}
				long timediff = CASTTimeUtil.diff(now,
						((PatrolMotive) motive).lastVisisted);
				motive.informationGain = 1 - (1. / Math
						.exp((timediff / EXP_NORM) * Math.log(2)));
				double costs = SpatialFacade.get(this).queryCosts(
						currentPlace.id, ((PatrolMotive) motive).placeID);
				if (costs < Double.MAX_VALUE) {
					motive.costs = (float) costs;
				}

			}

			motive.goal = new Goal(computeImportance(motive), -1, "(= (is-in "
					+ this.getRobotBeliefAddr().id + ") " + belief.getId()
					+ ")", false);
			log("goal is " + motive.goal.goalString + " with inf-gain "
					+ motive.informationGain);
		} catch (CASTException e) {
			logException(e);
		} catch (InterruptedException e) {
			logException(e);
		}

	}

	/**
	 * assigns costs to the motive
	 * 
	 * @param motive
	 * 
	 */
	@Override
	protected void assignCosts(Motive motive) {
		try {
			Place currentPlace;
			currentPlace = SpatialFacade.get(this).getPlace();
			log("compute cost from current place " + currentPlace.id
					+ " to place " + ((PatrolMotive) motive).placeID);
			if (currentPlace != null) {
				double costs = SpatialFacade.get(this).queryCosts(
						currentPlace.id, ((PatrolMotive) motive).placeID);
				if (costs < Double.MAX_VALUE) {
					motive.costs = (float) costs;
				}
			}
		} catch (CASTException e) {
			logException(e);
		} catch (InterruptedException e) {
			logException(e);
		}
	}

	float computeImportance(PatrolMotive m) {
		if (m.informationGain < 0)
			return -1.0f;
		else
			return (float) (m.informationGain * MAX_COSTS_TO_DROP);
	}

}
