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
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.MergedBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import execution.slice.Robot;

public class LookAroundMotiveGenerator extends
		AbstractBeliefMotiveGenerator<LookAtViewConeMotive, MergedBelief> {

	// TODO hardcoded vision.sa

	public static final String LOOKED_AT = "looked-at";

	private static final String VISION_SA = "vision.sa";

	// number of millis to wait with nothing going on before looking around.

	public static final long BOREDOM_THRESHOLD = 20000;

	public class DeleteViewCone implements Runnable {

		private final WorkingMemoryAddress m_toDelete;

		public DeleteViewCone(WorkingMemoryAddress m_toDelete) {
			this.m_toDelete = m_toDelete;
		}

		@Override
		public void run() {
			try {
				lockComponent();
				// get robot struct
				assert (m_robotAddress != null);
				Robot rbt = getMemoryEntry(m_robotAddress, Robot.class);
				if (rbt.currentViewCone.address.equals(m_toDelete)) {
					log("cone in use, rescheduling deletion");
					scheduleForDeletion(m_toDelete);
				} else {
					deleteFromWorkingMemory(m_toDelete);
				}
			} catch (SubarchitectureComponentException e) {
				logException(
						"Problem when performing scheduled deletion. You can ignore this",
						e);
			} finally {
				log("unlocking");
				unlockComponent();
			}
		}

	}

	public class GetCones implements Runnable, WorkingMemoryChangeReceiver {

		@Override
		public void run() {

			lockComponent();

			// only generate cones if none are waiting to be looked at
			if (m_myCones.isEmpty()) {

				if (System.currentTimeMillis() - m_lastSystemActivity < BOREDOM_THRESHOLD) {
					log("not bored yet");
				} else {
					try {

						log("ok, bored now, asking for some cones to look at");

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
			} else {
				log("my cones not empty");
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

	protected WorkingMemoryAddress m_robotAddress;

	public LookAroundMotiveGenerator() {
		super(VC_TYPE, LookAtViewConeMotive.class, MergedBelief.class);
		monitorMotivesForDeletion(true);
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

		// get robot address in a one-time reciever
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(Robot.class),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange arg0)
							throws CASTException {
						m_robotAddress = arg0.address;
						removeChangeFilter(this);
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

		LookAtViewConeMotive result = null;

		try {

			CASTIndependentFormulaDistributionsBelief<MergedBelief> coneBelief = CASTIndependentFormulaDistributionsBelief
					.create(MergedBelief.class, _newEntry);

			WorkingMemoryPointer vcPtr;

			vcPtr = BeliefUtils.recurseAncestorsForType(this, _wma,
					CASTUtils.typeName(ViewCone.class));

			if (vcPtr != null) {

				WorkingMemoryAddress coneAddr = vcPtr.address;

				if (m_myCones.contains(coneAddr)
						&& !coneHasBeenLookedAt(coneBelief)) {

					log("ViewCone has not been looked at, so generating motive.");

					// HACK planner doesn't recognise objects with no attribute
					// set
					BeliefUtils.addFeature(coneBelief,
							LookAroundMotiveGenerator.LOOKED_AT, false);
					overwriteWorkingMemory(coneBelief.getId(), "binder",
							coneBelief.get());
					// END HACK

					result = new LookAtViewConeMotive();

					result.coneAddr = coneAddr;

					result.created = getCASTTime();
					result.updated = result.created;
					result.maxExecutionTime = MAX_EXECUTION_TIME;
					result.maxPlanningTime = MAX_PLANNING_TIME;
					result.priority = MotivePriority.UNSURFACE;
					result.referenceEntry = _wma;
					result.status = MotiveStatus.UNSURFACED;

					result.goal = new Goal(100f, -1, "(" + LOOKED_AT + " '"
							+ coneBelief.getId() + "')", false);

					log("goal is " + result.goal.goalString + " with inf-gain "
							+ result.informationGain);

				}
			} else {
				getLogger().warn("Unable to find ViewCone belief ancsestor",
						getLogAdditions());
			}
		} catch (DoesNotExistOnWMException e) {
			logException(e);
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		} catch (PermissionException e) {
			logException(e);
		} catch (ConsistencyException e) {
			logException(e);
		}

		return result;
	}

	private boolean coneHasBeenLookedAt(
			CASTIndependentFormulaDistributionsBelief<MergedBelief> coneBelief) {
		FormulaDistribution fd = coneBelief.getContent().get(LOOKED_AT);
		boolean lookedAt = false;
		if (fd != null) {
			lookedAt = fd.getDistribution().getMostLikely().getBoolean();
		}
		return lookedAt;

	}

	@Override
	protected LookAtViewConeMotive checkForUpdate(MergedBelief _newEntry,
			LookAtViewConeMotive _existingMotive) {
		// CASTIndependentFormulaDistributionsBelief<MergedBelief> belief =
		// CASTIndependentFormulaDistributionsBelief
		// .create(MergedBelief.class, _newEntry);
		//
		// if (!coneHasBeenLookedAt(belief)) {
		// log("ViewCone belief is still not looked at, so leaving motive unchanged.");
		// return _existingMotive;
		// } else {
		// log("ViewCone belief has been looked at, so removing motive.");
		// m_myCones.remove(_existingMotive.coneAddr);
		// scheduleForDeletion(_existingMotive.coneAddr);
		// return null;
		// }

		// let the planner determine completion
		return _existingMotive;
	}

	@Override
	protected void motiveWasCompleted(LookAtViewConeMotive _motive)
			throws SubarchitectureComponentException {
		scheduleForDeletion(_motive.coneAddr);
	}

	private void scheduleForDeletion(WorkingMemoryAddress coneAddr) {
		log("scheduling cone for deletion: " + CASTUtils.toString(coneAddr));
		m_myCones.remove(coneAddr);
		scheduleForDeletion(new DeleteViewCone(coneAddr));
	}

	private void scheduleForDeletion(DeleteViewCone _deleter) {
		m_executor.schedule(_deleter, 20, TimeUnit.SECONDS);
	}
}
