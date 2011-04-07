package manipulation.runner.cogx;

import java.util.Map;

import manipulation.core.cogx.CogXManipulatorStore;
import manipulation.core.cogx.baseConnector.CogXDoraBaseConnector;
import manipulation.core.cogx.camConnector.CogXBlortConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.Manipulator.ManipulatorName;
import manipulation.core.share.ManipulatorStore;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.armConnector.ArmConnector.ArmName;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.Configuration;
import manipulation.core.share.types.Vector3D;
import manipulation.itemMemory.ItemMemory;
import manipulation.runner.share.Runner;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.LinearBaseMovementApproachCommand;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.strategies.Strategy;
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
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cogx.Math.Vector3;

/**
 * concrete start-up / runner for the Birmingham / CogX environment
 * 
 * @author ttoenige
 * 
 */
public class CogXRunner extends ManagedComponent implements Runner {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void configure(Map<String, String> config) {
		Configuration configuration = null;
		if (config.containsKey("--reachingDistance")
				&& config.containsKey("--distanceInFrontDesk")
				&& config.containsKey("--configPath")
				&& config.containsKey("--maxDistanceVP")
				&& config.containsKey("--nearPoints")
				&& config.containsKey("--fixedTilt")
				&& config.containsKey("--armName")) {

			ArmConnector.ArmName armName = null;

			if (config.get("--armName").equals("katana450")) {
				armName = ArmName.KATANA450;
			} else if (config.get("--armName").equals("katana300")) {
				armName = ArmName.KATANA300;
			} else if (config.get("--armName").equals("simluation")) {
				armName = ArmName.SIMULATION;
			} else {
				logger.error("Cannot parse arm name - using simulation environment");
				armName = ArmName.SIMULATION;
			}

			configuration = new Configuration(Double.parseDouble(config
					.get("--reachingDistance")), Double.parseDouble(config
					.get("--distanceInFrontDesk")), config.get("--configPath"),
					Double.parseDouble(config.get("--maxDistanceVP")),
					Double.parseDouble(config.get("--nearPoints")), armName,
					Double.parseDouble(config.get("--fixedTilt")));
		} else {
			logger.error("Cannot read arguments - exit");
			System.exit(-1);
		}

		ManipulatorStore cogxManStore = new CogXManipulatorStore();
		ItemMemory itemMemory = new ItemMemory();

		manipulator = cogxManStore.orderManipulator(
				ManipulatorName.INTELLIGENT_GRASPING, this, itemMemory,
				configuration);
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
		new CogXTestGUI(manipulator);
	}

	/**
	 * add a robot base position listener to the CAST working memory
	 */
	public void addBaseMovementListener() {

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXDoraBaseConnector) manipulator.getBaseConnector())
								.robotPositionChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
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
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXBlortConnector) manipulator.getCamConnector())
								.visualObjectChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((CogXBlortConnector) manipulator.getCamConnector())
								.visualObjectChanged(_wmc);
					}
				});
	}

	/**
	 * add the manipulation listener to the CAST working memory
	 */
	public void addManipulationListener() {
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				PutDownCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting putDownCommand from add action in WM");

						try {
							PutDownCommand command = getMemoryEntry(
									_wmc.address, PutDownCommand.class);
						} catch (DoesNotExistOnWMException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				PutDownCommand.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting putDownCommand from overwrite action in WM");
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				FarArmMovementCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting FarArmMovementCommand from add action in WM");

						try {
							FarArmMovementCommand command = getMemoryEntry(
									_wmc.address, FarArmMovementCommand.class);

							Vector3 targetPosition = command.targetObject.pose.pos;
							ArmError error = manipulator.getArmConnector()
									.approachObject(
											new Vector3D(targetPosition.x,
													targetPosition.y,
													targetPosition.z));

							command.xError = error.getPoseError().getX();
							command.yError = error.getPoseError().getY();
							command.zError = error.getPoseError().getZ();

							if (error.isSuccess()) {
								command.comp = ManipulationCompletion.SUCCEEDED;
							} else {
								command.comp = ManipulationCompletion.FAILED;
							}

							try {
								overwriteWorkingMemory(_wmc.address, command);
							} catch (ConsistencyException e) {
								logger.error(e);
							} catch (PermissionException e) {
								logger.error(e);
							}
						} catch (DoesNotExistOnWMException e) {
							logger.error(e);
						} catch (UnknownSubarchitectureException e) {
							logger.error(e);
						}

					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				FarArmMovementCommand.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting FarArmMovementCommand from overwrite action in WM");
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				LinearGraspApproachCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting LinearGraspApproachCommand from add action in WM");
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				LinearGraspApproachCommand.class,
				WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting LinearGraspApproachCommand from overwrite action in WM");
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				SimulateGraspCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting SimulateGraspCommand from add action in WM");
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				SimulateGraspCommand.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting SimulateGraspCommand from overwrite action in WM");
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				LinearBaseMovementApproachCommand.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				logger.error("Getting LinearBaseMovementApproachCommand from add action in WM");
			}
		});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				LinearBaseMovementApproachCommand.class,
				WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						logger.error("Getting LinearBaseMovementApproachCommand from overwrite action in WM");
					}
				});
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void startStrategy(Strategy.Name strategyName) {
		logger.info("No strategies available to start");
	}
}
