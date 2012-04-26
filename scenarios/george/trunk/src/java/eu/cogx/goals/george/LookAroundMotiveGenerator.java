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
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class LookAroundMotiveGenerator extends
		AbstractBeliefMotiveGenerator<LookAtViewConeMotive, MergedBelief> {

	// TODO hardcoded vision.sa

	private static final String VISION_SA = "vision.sa";

	// number of millis to wait with nothing going on before looking around.

	public static final long BOREDOM_THRESHOLD = 40000;

	public class GetCones implements Runnable, WorkingMemoryChangeReceiver {

		@Override
		public void run() {

			lockComponent();

			// only generate cones if none are waiting to be looked at
			if (m_myCones.isEmpty()) {

				if (System.currentTimeMillis() - m_lastSystemActivity < BOREDOM_THRESHOLD) {
					println("not bored yet");
				} else {
					try {

						println("ok, bored now, asking for some cones to look at");

						LookAroundCommand cmd = new LookAroundCommand(null,
								VisionCommandStatus.VCREQUESTED);

						WorkingMemoryAddress cmdAddr = new WorkingMemoryAddress(
								newDataID(), VISION_SA);
						addChangeFilter(
								ChangeFilterFactory.createAddressFilter(
										cmdAddr,
										WorkingMemoryOperation.OVERWRITE), this);
						addToWorkingMemory(cmdAddr, cmd);
					} catch (CASTException e) {
						logException(e);
					}
				}
			}

			unlockComponent();

		}

		@Override
		public void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {
			log("got some cones back");

			// Don't clean up, because it ruins the "current_viewcone" belief of
			// the robot and thus makes it impossible to find further plans.
			//
			// // create the reciever to clean up this mess
			// addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
			// LookAtViewConeMotive.class, WorkingMemoryOperation.DELETE),
			// new CleanupReceiver());

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

	private boolean m_fired;

	public LookAroundMotiveGenerator() {
		super(VC_TYPE, LookAtViewConeMotive.class, MergedBelief.class);
		m_fired = false;
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
			MergedBelief _newEntry) {

		assert (_newEntry.type.equals(VC_TYPE));

		log("checkForAddition(): check belief " + _newEntry.id
				+ " for addition");

		CASTIndependentFormulaDistributionsBelief<MergedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, _newEntry);

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
			result.maxExecutionTime = MAX_EXECUTION_TIME;
			result.maxPlanningTime = MAX_PLANNING_TIME;
			result.priority = MotivePriority.UNSURFACE;
			result.referenceEntry = _wma;
			result.status = MotiveStatus.UNSURFACED;

			result.goal = new Goal(100f, -1,
					"(looked-at '" + belief.getId() + "')", false);

			log("goal is " + result.goal.goalString + " with inf-gain "
					+ result.informationGain);

		}
		return result;
	}

	@Override
	protected LookAtViewConeMotive checkForUpdate(MergedBelief _newEntry,
			LookAtViewConeMotive _existingMotive) {
		CASTIndependentFormulaDistributionsBelief<MergedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(MergedBelief.class, _newEntry);

		if (!belief.getContent().containsKey("looked-at")) {
			log("ViewCone belief is still not looked at, so leaving motive unchanged.");
			return _existingMotive;
		} else {
			log("ViewCone belief has been looked at, so removing motive.");
			m_myCones.remove(_existingMotive.coneAddr);
			return null;
		}
	}

	// /**
	// * Cleans up this component's view cones when no look around motives are
	// * left.
	// *
	// * TODO parameterise based on newly added VCs.
	// *
	// * @author nah
	// *
	// */
	// private class CleanupReceiver implements WorkingMemoryChangeReceiver {
	// @Override
	// public void workingMemoryChanged(WorkingMemoryChange arg0)
	// throws CASTException {
	//
	// // see if there are any look around motives left
	//
	// if (!hasAvailableMotives()) {
	//
	// Iterator<WorkingMemoryAddress> i = m_myCones.iterator();
	//
	// while (i.hasNext()) {
	// log("cleaning up cone");
	// deleteFromWorkingMemory(i.next());
	// i.remove();
	// }
	//
	// removeChangeFilter(this);
	//
	// } else {
	// println("some motives still available");
	// }
	// }
	//
	// }

}
