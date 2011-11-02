package manipulation.runner.cogx;

import java.util.Map;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.cogx.CogXManipulatorStore;
import manipulation.core.cogx.armConnector.CogXKatanaArmConnector;
import manipulation.core.cogx.baseConnector.CogXDoraBaseConnector;
import manipulation.core.cogx.camConnector.CogXBlortConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.Manipulator.ManipulatorName;
import manipulation.core.share.ManipulatorStore;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.armConnector.ArmConnector.ArmName;
import manipulation.core.share.types.Configuration;
import manipulation.itemMemory.ItemMemory;
import manipulation.runner.share.Runner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.FineArmMovementCommand;
import manipulation.slice.GetCurrentArmPose;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationExternalCommand;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.MoveArmToPose;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateFarArmMovementCommand;
import manipulation.slice.SimulateMoveToPose;
import manipulation.slice.StopCommand;
import manipulation.strategies.CommandExecution;
import manipulation.strategies.Strategy;
import manipulation.strategies.Strategy.Name;
import manipulation.visualisation.CogXTestGUI;

import org.apache.log4j.Logger;

import NavData.RobotPose2d;
import VisionData.VisualObject;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * concrete start-up / runner for the Birmingham / CogX environment
 * 
 * @author Torben Toeniges
 * 
 */
public class CogXRunner extends ManagedComponent implements Runner {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	private CommandWatcher watcher;

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void configure(Map<String, String> config) {
		Configuration configuration = null;
		if (config.containsKey("--configPath")
				&& config.containsKey("--armName")) {

			ArmConnector.ArmName armName = null;

			if (config.get("--armName").equals("katana450")) {
				armName = ArmName.KATANA450;
			} else if (config.get("--armName").equals("katana300")) {
				armName = ArmName.KATANA300;
			} else if (config.get("--armName").equals("simulation")) {
				armName = ArmName.SIMULATION;
			} else {
				logger.error("Cannot parse arm name - using simulation environment");
				armName = ArmName.SIMULATION;
			}

			configuration = new Configuration(config.get("--configPath"),
					armName, config.containsKey("--testGUI"),
          config.containsKey("--enableTableObstacleHack"));
		} else {
			logger.error("Cannot read arguments - exit");
			System.exit(-1);
		}

		ManipulatorStore cogxManStore = new CogXManipulatorStore();
		ItemMemory itemMemory = new ItemMemory();

		watcher = new CommandWatcher();

		manipulator = cogxManStore.orderManipulator(
				ManipulatorName.COGX_MOBILE_MANIPULATOR, this, itemMemory,
				watcher, configuration);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void start() {
		logger.debug("Adding Listener");
		addBaseMovementListener();
		addVisionListener();
		addManipulationListener();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void runComponent() {
		if (manipulator.getConfiguration().isGui()) {
			new CogXTestGUI(manipulator);
		}

		startStrategy(Name.COMMAND_EXECUTION);
	}

	/**
	 * add a robot base position listener to the CAST working memory
	 */
	public void addBaseMovementListener() {

		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXDoraBaseConnector) manipulator.getBaseConnector())
								.robotPositionChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXDoraBaseConnector) manipulator.getBaseConnector())
								.robotPositionChanged(_wmc);
					}
				});
	}

	/**
	 * add the vision listener to the CAST working memory
	 */
	public void addVisionListener() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				VisualObject.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXBlortConnector) manipulator.getCamConnector())
								.visualObjectChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXBlortConnector) manipulator.getCamConnector())
								.visualObjectChanged(_wmc);
					}
				});
		
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				VisualObject.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXBlortConnector) manipulator.getCamConnector())
								.visualObjectDelete(_wmc);
					}
				});
	}

	/**
	 * add the manipulation listener to the CAST working memory
	 */
	public void addManipulationListener() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				ManipulationExternalCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.info("getting manipulation command (add)");
						try {
							ManipulationExternalCommand command = getMemoryEntry(
									_wmc.address,
									ManipulationExternalCommand.class);
							watcher.setCurrentCommandAddress(_wmc.address);
							watcher.newCommand(command);
						} catch (DoesNotExistOnWMException e) {
							logger.error(e);
						} catch (UnknownSubarchitectureException e) {
							logger.error(e);
						}
					}
				});

		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				ManipulationExternalCommand.class,
				WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							logger.info("getting manipulation command (overwrite)");

							ManipulationExternalCommand command = getMemoryEntry(
									_wmc.address,
									ManipulationExternalCommand.class);
							if ((command.status == ManipulationCommandStatus.NEW)
									|| (command.status == ManipulationCommandStatus.CHANGED)) {
								watcher.setCurrentCommandAddress(_wmc.address);
								watcher.newCommand(command);
							} else {
								logger.debug("ignoring overwrite action");
							}

							if ((command.status == ManipulationCommandStatus.FINISHED)) {
								if (command instanceof FarArmMovementCommand) {
									FarArmMovementCommand fc = (FarArmMovementCommand) command;
									logger.debug("FAR ARM MOVEMENT");
									logger.debug("COMP: " + fc.comp);
									logger.debug("STAT: " + fc.status);
									logger.debug("POS: " + fc.reachedPose.pos.x
											+ " | " + fc.reachedPose.pos.y
											+ " | " + fc.reachedPose.pos.z);
									logger.debug("ROT: "
											+ fc.reachedPose.rot.m00 + " "
											+ fc.reachedPose.rot.m01 + " "
											+ fc.reachedPose.rot.m02);
									logger.debug("ROT: "
											+ fc.reachedPose.rot.m10 + " "
											+ fc.reachedPose.rot.m11 + " "
											+ fc.reachedPose.rot.m12);
									logger.debug("ROT: "
											+ fc.reachedPose.rot.m20 + " "
											+ fc.reachedPose.rot.m21 + " "
											+ fc.reachedPose.rot.m22);
									logger.debug("ADR: " + fc.targetObjectAddr);
								}
								if (command instanceof PutDownCommand) {
									PutDownCommand pc = (PutDownCommand) command;
									logger.debug("PUT DOWN");
									logger.debug("COMP: " + pc.comp);
									logger.debug("STAT: " + pc.status);
									logger.debug("BASED OBJ ARD: "
											+ pc.basedObjectAddr);

								}

								if (command instanceof FineArmMovementCommand) {
									FineArmMovementCommand lc = (FineArmMovementCommand) command;
									logger.debug("LIN APP");
									logger.debug("COMP: " + lc.comp);
									logger.debug("STAT: " + lc.status);
									logger.debug("GRASP STAT: "
											+ lc.graspStatus);
									logger.debug("TARGET OBJ ADR"
											+ lc.targetObjectAddr);
								}

								if (command instanceof SimulateFarArmMovementCommand) {
									SimulateFarArmMovementCommand sc = (SimulateFarArmMovementCommand) command;
									logger.debug("SIM APP");
									logger.debug("COMP: " + sc.comp);
									logger.debug("STAT: " + sc.status);
									logger.debug("POS: "
											+ sc.simulatedReachablePose.pos.x
											+ " | "
											+ sc.simulatedReachablePose.pos.y
											+ " | "
											+ sc.simulatedReachablePose.pos.z);
									logger.debug("ROT: "
											+ sc.simulatedReachablePose.rot.m00
											+ " "
											+ sc.simulatedReachablePose.rot.m01
											+ " "
											+ sc.simulatedReachablePose.rot.m02);
									logger.debug("ROT: "
											+ sc.simulatedReachablePose.rot.m10
											+ " "
											+ sc.simulatedReachablePose.rot.m11
											+ " "
											+ sc.simulatedReachablePose.rot.m12);
									logger.debug("ROT: "
											+ sc.simulatedReachablePose.rot.m20
											+ " "
											+ sc.simulatedReachablePose.rot.m21
											+ " "
											+ sc.simulatedReachablePose.rot.m22);
								}

								if (command instanceof StopCommand) {
									StopCommand sc = (StopCommand) command;
									logger.debug("STOP APP");
									logger.debug("COMP: " + sc.comp);
									logger.debug("STAT: " + sc.status);
								}

								if (command instanceof MoveArmToHomePositionCommand) {
									MoveArmToHomePositionCommand mc = (MoveArmToHomePositionCommand) command;
									logger.debug("STOP APP");
									logger.debug("COMP: " + mc.comp);
									logger.debug("STAT: " + mc.status);
								}

								if (command instanceof OpenGripperCommand) {
									OpenGripperCommand mc = (OpenGripperCommand) command;
									logger.debug("OPEN APP");
									logger.debug("COMP: " + mc.comp);
									logger.debug("STAT: " + mc.status);
								}

								if (command instanceof CloseGripperCommand) {
									CloseGripperCommand cc = (CloseGripperCommand) command;
									logger.debug("CLOSE APP");
									logger.debug("COMP: " + cc.comp);
									logger.debug("STAT: " + cc.status);
									logger.debug("GRASP STAT: "
											+ cc.graspStatus);
								}

								if (command instanceof MoveArmToPose) {
									MoveArmToPose mc = (MoveArmToPose) command;
									logger.debug("MOVE TO POSE APP");
									logger.debug("COMP: " + mc.comp);
									logger.debug("STAT: " + mc.status);
									logger.debug("TARGETPOS: "
											+ mc.targetPose.pos.x + " | "
											+ mc.targetPose.pos.y + " | "
											+ mc.targetPose.pos.z);
									logger.debug("ROT: "
											+ mc.targetPose.rot.m00 + " "
											+ mc.targetPose.rot.m01 + " "
											+ mc.targetPose.rot.m02);
									logger.debug("ROT: "
											+ mc.targetPose.rot.m10 + " "
											+ mc.targetPose.rot.m11 + " "
											+ mc.targetPose.rot.m12);
									logger.debug("ROT: "
											+ mc.targetPose.rot.m20 + " "
											+ mc.targetPose.rot.m21 + " "
											+ mc.targetPose.rot.m22);

									logger.debug("REACHEDPOS: "
											+ mc.reachedPose.pos.x + " | "
											+ mc.reachedPose.pos.y + " | "
											+ mc.reachedPose.pos.z);
									logger.debug("ROT: "
											+ mc.reachedPose.rot.m00 + " "
											+ mc.reachedPose.rot.m01 + " "
											+ mc.reachedPose.rot.m02);
									logger.debug("ROT: "
											+ mc.reachedPose.rot.m10 + " "
											+ mc.reachedPose.rot.m11 + " "
											+ mc.reachedPose.rot.m12);
									logger.debug("ROT: "
											+ mc.reachedPose.rot.m20 + " "
											+ mc.reachedPose.rot.m21 + " "
											+ mc.reachedPose.rot.m22);
								}

								if (command instanceof GetCurrentArmPose) {
									GetCurrentArmPose gc = (GetCurrentArmPose) command;
									logger.debug("GET ARM POSE");
									logger.debug("COMP: " + gc.comp);
									logger.debug("STAT: " + gc.status);

									logger.debug("CURRENT POS: "
											+ gc.currentPose.pos.x + " | "
											+ gc.currentPose.pos.y + " | "
											+ gc.currentPose.pos.z);
									logger.debug("ROT: "
											+ gc.currentPose.rot.m00 + " "
											+ gc.currentPose.rot.m01 + " "
											+ gc.currentPose.rot.m02);
									logger.debug("ROT: "
											+ gc.currentPose.rot.m10 + " "
											+ gc.currentPose.rot.m11 + " "
											+ gc.currentPose.rot.m12);
									logger.debug("ROT: "
											+ gc.currentPose.rot.m20 + " "
											+ gc.currentPose.rot.m21 + " "
											+ gc.currentPose.rot.m22);
								}

								if (command instanceof SimulateMoveToPose) {
									SimulateMoveToPose sc = (SimulateMoveToPose) command;
									logger.debug("SIMULATE ARM POS");
									logger.debug("COMP: " + sc.comp);
									logger.debug("STAT: " + sc.status);

									logger.debug("CURRENT POS: "
											+ sc.simulatedReachablePose.pos.x
											+ " | "
											+ sc.simulatedReachablePose.pos.y
											+ " | "
											+ sc.simulatedReachablePose.pos.z);
									logger.debug("ROT: "
											+ sc.simulatedReachablePose.rot.m00
											+ " "
											+ sc.simulatedReachablePose.rot.m01
											+ " "
											+ sc.simulatedReachablePose.rot.m02);
									logger.debug("ROT: "
											+ sc.simulatedReachablePose.rot.m10
											+ " "
											+ sc.simulatedReachablePose.rot.m11
											+ " "
											+ sc.simulatedReachablePose.rot.m12);
									logger.debug("ROT: "
											+ sc.simulatedReachablePose.rot.m20
											+ " "
											+ sc.simulatedReachablePose.rot.m21
											+ " "
											+ sc.simulatedReachablePose.rot.m22);
								}
							}
						} catch (DoesNotExistOnWMException e) {
							logger.error(e);
						} catch (UnknownSubarchitectureException e) {
							logger.error(e);
						}
					}
				});
	}

	public void updateWorkingMemoryCommand(WorkingMemoryAddress adress,
			ManipulationExternalCommand cmd) {
		try {
                        getMemoryEntry(adress, ManipulationExternalCommand.class);
			overwriteWorkingMemory(adress, cmd);
		} catch (DoesNotExistOnWMException e) {
			logger.error(e);
		} catch (ConsistencyException e) {
			logger.error(e);
		} catch (PermissionException e) {
			logger.error(e);
		} catch (UnknownSubarchitectureException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void startStrategy(Strategy.Name strategyName) {
		((CogXKatanaArmConnector) manipulator.getArmConnector()).initSimMove();

		if (strategyName == Name.COMMAND_EXECUTION) {
			Strategy strategy = new CommandExecution(manipulator);
			strategy.startExecution();
		} else {
			logger.error("Does not know the strategy!");
		}

	}
}
