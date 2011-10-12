package spatial.execution;

import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.dora.PersonTransferFunction;
import execution.slice.ActionStatus;
import execution.slice.TriBool;
import execution.slice.actions.EngageWithHuman;
import execution.slice.actions.TurnToHuman;
import execution.util.NonBlockingActionExecutor;

public class TurnToPersonExecutor extends
		NonBlockingActionExecutor<TurnToHuman> implements
		WorkingMemoryChangeReceiver {

	public static final String UNRESPONSIVE = "unresponsive";
	public static final String ENGAGED = "engaged";

	public TurnToPersonExecutor(ManagedComponent component) {
		super(component, TurnToHuman.class);
	}

	private boolean m_navIsComplete = false;
	private WorkingMemoryAddress m_navCmdAddr;
	private CASTIndependentFormulaDistributionsBelief<GroundedBelief> m_humanBelief;
	private WorkingMemoryAddress m_humanBeliefAddr;
	private boolean m_dlgIsOngoing = false;
	private WorkingMemoryAddress m_dlgActionAddr;

	// public boolean accept(Action _action) {
	// m_humanBeliefAddr = ((EngageWithHuman) _action).beliefAddress;
	// try {
	// m_humanBelief = CASTIndependentFormulaDistributionsBelief.create(
	// GroundedBelief.class, getMemoryEntry(m_humanBeliefAddr,
	// GroundedBelief.class));
	// } catch (CASTException e) {
	// logException(e);
	// return false;
	// }
	// return true;
	// }

	/*
	 * (non-Javadoc)
	 * 
	 * @see execution.util.ActionExecutor#stopExecution()
	 */
	@Override
	public void stopExecution() {
		if (m_dlgIsOngoing) {
			try {
				// stop the dialog action as well
				getComponent().deleteFromWorkingMemory(m_dlgActionAddr);
			} catch (CASTException e) {
				logException(e);
			}
		}

		// remove overwrite receiver
		if (!m_navIsComplete) {
			try {
				log("aborting execution");
				getComponent().removeChangeFilter(this);
				// reread
				getComponent().lockEntry(m_navCmdAddr,
						WorkingMemoryPermissions.LOCKEDODR);
				NavCommand navCmd = getComponent().getMemoryEntry(m_navCmdAddr,
						NavCommand.class);
				navCmd.comp = Completion.COMMANDABORTED;
				getComponent().overwriteWorkingMemory(m_navCmdAddr, navCmd);
				getComponent().unlockEntry(m_navCmdAddr);

			} catch (DoesNotExistOnWMException e) {
				//
				// e.printStackTrace();
			} catch (SubarchitectureComponentException e) {
				e.printStackTrace();
			}
		}
	}

	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
		// read in the nav cmd
		getComponent().lockEntry(_wmc.address,
				WorkingMemoryPermissions.LOCKEDODR);
		NavCommand cmd = getComponent().getMemoryEntry(_wmc.address,
				NavCommand.class);
		if (cmd.comp == Completion.COMMANDFAILED) {
			log("command failed by the looks of this: " + cmd.comp);
			m_navIsComplete = true;
			getComponent().deleteFromWorkingMemory(_wmc.address);
			getComponent().removeChangeFilter(this);
			executionComplete(TriBool.TRIFALSE);
		} else if (cmd.comp == Completion.COMMANDSUCCEEDED) {
			log("command completed by the looks of this: " + cmd.comp);
			getComponent().deleteFromWorkingMemory(_wmc.address);
			getComponent().removeChangeFilter(this);
			m_navIsComplete = true;
			initiateDialogAction();
		} else {
			log("command in progress: " + cmd.comp);
			getComponent().unlockEntry(_wmc.address);
		}
	}

	private void initiateDialogAction() throws CASTException {

		EngageWithHuman ewh = new EngageWithHuman(ActionStatus.PENDING,
				TriBool.TRIINDETERMINATE, m_humanBeliefAddr, false);
		m_dlgActionAddr = new WorkingMemoryAddress(getComponent().newDataID(),
				getComponent().getSubarchitectureID());
		getComponent().addChangeFilter(
				ChangeFilterFactory.createAddressFilter(m_dlgActionAddr,
						WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {

					@Override
					public void workingMemoryChanged(WorkingMemoryChange wmc)
							throws CASTException {
						if (wmc.operation == WorkingMemoryOperation.DELETE) {
							println("dialog action removed... indicates abortion");
							getComponent().removeChangeFilter(this);
							m_dlgIsOngoing = false;

						}

						getComponent().lockEntry(wmc.address,
								WorkingMemoryPermissions.LOCKEDODR);

						EngageWithHuman action = getComponent().getMemoryEntry(
								wmc.address, EngageWithHuman.class);
						switch (action.status) {
						case COMPLETE:
							// flag engagement depending on action outcome
							flagEngagementInBelief(action.success == TriBool.TRITRUE);
							log("dialog action completed and engagement flagged: "
									+ action.success.toString());
							getComponent().removeChangeFilter(this);
							getComponent().deleteFromWorkingMemory(wmc.address);
							executionComplete(action.success);
							m_dlgIsOngoing = false;
							break;
						default:
							log("dialog action ongoing: "
									+ action.status.toString());
							getComponent().unlockEntry(wmc.address);
							break;

						}

					}
				});
		getComponent().addToWorkingMemory(m_dlgActionAddr, ewh);
		m_dlgIsOngoing = true;

	}

	@Override
	public void executeAction() {
		try {

			m_humanBeliefAddr = getAction().beliefAddress;
			try {
				m_humanBelief = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, getComponent()
								.getMemoryEntry(m_humanBeliefAddr,
										GroundedBelief.class));
			} catch (CASTException e) {
				logException(e);
				executionComplete(TriBool.TRIFALSE);
				return;
			}
			double posX = m_humanBelief.getContent().get(
					PersonTransferFunction.ATTR_POS_X).getDistribution()
					.getMostLikely().getDouble();
			double posY = m_humanBelief.getContent().get(
					PersonTransferFunction.ATTR_POS_Y).getDistribution()
					.getMostLikely().getDouble();
			double posTheta = m_humanBelief.getContent().get(
					PersonTransferFunction.ATTR_POS_THETA).getDistribution()
					.getMostLikely().getDouble();
			// else create the command and send it off, ignoring path
			// transition
			// probs for now
			NavCommand cmd = SpatialActionInterface.newNavCommand();
			cmd.cmd = CommandType.GOTOPOSITION;
			cmd.destId = new long[] {};
			cmd.angle = new double[] {};
			cmd.distance = new double[] {};
			cmd.pose = new double[] { posX, posY, posTheta };
			cmd.tolerance = new double[] { 0.1, 0.1, Math.PI * 10.0 / 180.0 };

			// going to add locally for the time being, but using wma to
			// allow
			// this to be changed later
			m_navCmdAddr = new WorkingMemoryAddress(getComponent().newDataID(),
					getComponent().getSubarchitectureID());
			getComponent().addChangeFilter(
					ChangeFilterFactory.createAddressFilter(m_navCmdAddr,
							WorkingMemoryOperation.OVERWRITE), this);

			getComponent().addToWorkingMemory(m_navCmdAddr, cmd);

		} catch (NullPointerException e) {
			logException(e);
			executionComplete(TriBool.TRIFALSE);
			return;
		} catch (BeliefException e) {
			logException(e);
			executionComplete(TriBool.TRIFALSE);
			return;
		} catch (CASTException e) {
			logException(e);
			executionComplete(TriBool.TRIFALSE);
		}

	}

	/**
	 * @param b
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws PermissionException
	 * @throws UnknownSubarchitectureException
	 */
	void flagEngagementInBelief(boolean b) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {
		FormulaDistribution fd = FormulaDistribution.create();
		fd.add(b, 1.0);
		m_humanBelief.getContent().put(ENGAGED, fd);
		getComponent().overwriteWorkingMemory(m_humanBeliefAddr,
				m_humanBelief.get());
		fd=FormulaDistribution.create();
		fd.add(!b, 1.0);
		m_humanBelief.getContent().put(UNRESPONSIVE, fd);
		getComponent().overwriteWorkingMemory(m_humanBeliefAddr,
				m_humanBelief.get());
	}
}
