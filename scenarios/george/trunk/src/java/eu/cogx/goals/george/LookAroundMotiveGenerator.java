package eu.cogx.goals.george;

import java.util.HashSet;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import motivation.components.generators.AbstractBeliefMotiveGenerator;
import motivation.slice.LookAtViewConeMotive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.PlanProxy;
import VisionData.LookAroundCommand;
import VisionData.ViewCone;
import VisionData.VisionCommandStatus;
import autogen.Planner.Goal;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class LookAroundMotiveGenerator extends
		AbstractBeliefMotiveGenerator<LookAtViewConeMotive, GroundedBelief> {

	// TODO hardcoded vision.sa

	private static final String VISION_SA = "vision.sa";

	// number of millis to wait with nothing going on before looking around.

	public static final long BOREDOM_THRESHOLD = 20000;

	public class GetCones implements Runnable, WorkingMemoryChangeReceiver {

		@Override
		public void run() {

			if (System.currentTimeMillis() - m_lastSystemActivity < BOREDOM_THRESHOLD) {
				println("not bored yet");
				return;
			}

			try {

				println("ok, bored now, asking for some cones to look at");

				// TODO make some kind of cone cleanup work
				// // cleaning up old cones
				// for (WorkingMemoryAddress coneAddr : m_myCones) {
				// deleteFromWorkingMemory(coneAddr);
				// }

				LookAroundCommand cmd = new LookAroundCommand(null,
						VisionCommandStatus.VCREQUESTED);

				WorkingMemoryAddress cmdAddr = new WorkingMemoryAddress(
						newDataID(), VISION_SA);
				addChangeFilter(ChangeFilterFactory.createAddressFilter(
						cmdAddr, WorkingMemoryOperation.OVERWRITE), this);
				addToWorkingMemory(cmdAddr, cmd);
			} catch (CASTException e) {
				logException(e);
			}

		}

		@Override
		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {
			println("got some cones back");
			LookAroundCommand cmd = getMemoryEntry(_wmc.address,
					LookAroundCommand.class);
			for (ViewCone vc : cmd.viewCones) {
				WorkingMemoryAddress wma = new WorkingMemoryAddress(
						newDataID(), VISION_SA);
				m_myCones.add(wma);
				try {
					addToWorkingMemory(wma, vc);
				} catch (CASTException e) {
					logException(e);
					m_myCones.remove(wma);
				}
			}
			deleteFromWorkingMemory(_wmc.address);
			removeChangeFilter(this);
		}

	}

	private static final String VC_TYPE = SimpleDiscreteTransferFunction
			.getBeliefTypeFromCastType(CASTUtils.typeName(ViewCone.class));

	private static final int MAX_EXECUTION_TIME = 30;

	private static final int MAX_PLANNING_TIME = 30;

	private static final Class<PlanProxy> ACTIVITY_CLASS = PlanProxy.class;

	private final ScheduledThreadPoolExecutor m_executor = new ScheduledThreadPoolExecutor(
			1);

	private final HashSet<WorkingMemoryAddress> m_myCones = new HashSet<WorkingMemoryAddress>(
			3);

	private long m_lastSystemActivity;

	public LookAroundMotiveGenerator() {
		super(VC_TYPE, LookAtViewConeMotive.class, GroundedBelief.class);
	}

	@Override
	protected void start() {
		super.start();

		// listen for plan proxy additions for signals of system activity
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ACTIVITY_CLASS, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange arg0)
							throws CASTException {
						activityNoted();
					}
				});

		m_lastSystemActivity = System.currentTimeMillis();

		// check for something to do every 20 seconds
		// TODO make initial delay and repeat configurable
		m_executor.scheduleWithFixedDelay(new GetCones(), 20, 20,
				TimeUnit.SECONDS);

	}

	private void activityNoted() {
		m_lastSystemActivity = System.currentTimeMillis();
	}

	@Override
	protected LookAtViewConeMotive checkForAddition(WorkingMemoryAddress _wma,
			GroundedBelief _newEntry) {

		assert (_newEntry.type.equals(VC_TYPE));

		log("checkForAddition(): check belief " + _newEntry.id
				+ " for addition");

		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, _newEntry);

		CASTBeliefHistory hist = (CASTBeliefHistory) _newEntry.hist;
		WorkingMemoryAddress coneAddr = hist.ancestors.get(0).address;

		LookAtViewConeMotive result = null;

		if (m_myCones.contains(coneAddr)
				&& !belief.getContent().containsKey("looked-at")) {

			log("ViewCone  has not been looked at, so generating motive.");

			result = new LookAtViewConeMotive();

			result.coneAddr = coneAddr;

			result.created = getCASTTime();
			result.updated = result.created;
			result.correspondingUnion = "";
			result.maxExecutionTime = MAX_EXECUTION_TIME;
			result.maxPlanningTime = MAX_PLANNING_TIME;
			result.priority = MotivePriority.UNSURFACE;
			result.referenceEntry = _wma;
			result.status = MotiveStatus.UNSURFACED;

			result.goal = new Goal(100f,
					"(looked-at '" + belief.getId() + "')", false);

			log("goal is " + result.goal.goalString + " with inf-gain "
					+ result.informationGain);

		}
		return result;
	}

	@Override
	protected LookAtViewConeMotive checkForUpdate(GroundedBelief _newEntry,
			LookAtViewConeMotive _existingMotive) {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, _newEntry);

		if (!belief.getContent().containsKey("looked-at")) {
			log("ViewCone belief is still not looked at, so leaving motive unchanged.");
			return _existingMotive;
		} else {
			// TODO make some kind of cone cleanup work
			// try {
			// log("ViewCone belief is now looked at, so removing motive and viewcone.");
			// deleteFromWorkingMemory(_existingMotive.coneAddr);
			// } catch (Exception e) {
			// logException(e);
			// }
			return null;
		}
	}

}
