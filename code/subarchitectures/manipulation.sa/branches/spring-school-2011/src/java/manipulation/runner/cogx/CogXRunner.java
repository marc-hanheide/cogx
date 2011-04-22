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
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationExternalCommand;
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
					armName);
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
		new CogXTestGUI(manipulator);

		startStrategy(Name.COMMAND_EXECUTION);
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

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
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
		((CogXKatanaArmConnector) manipulator.getArmConnector())
				.initSimMove();

		if (strategyName == Name.COMMAND_EXECUTION) {
			Strategy strategy = new CommandExecution(manipulator);
			strategy.startExecution();
		} else {
			logger.error("Does not know the strategy!");
		}

	}
}
