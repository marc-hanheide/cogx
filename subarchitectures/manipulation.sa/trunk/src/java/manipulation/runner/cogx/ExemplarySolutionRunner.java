package manipulation.runner.cogx;

import java.util.Map;

import manipulation.core.cogx.CogXManipulatorStore;
import manipulation.core.cogx.baseConnector.ExemplarySolutionDoraBaseConnector;
import manipulation.core.cogx.camConnector.ExemplarySolutionBlortConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.Manipulator.ManipulatorName;
import manipulation.core.share.ManipulatorStore;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.armConnector.ArmConnector.ArmName;
import manipulation.core.share.types.Configuration;
import manipulation.itemMemory.ItemMemory;
import manipulation.runner.share.Runner;
import manipulation.strategies.CalibrationStrategy;
import manipulation.strategies.MobileManipulation;
import manipulation.strategies.Strategy;
import manipulation.visualisation.ExecutionGUI;

import org.apache.log4j.Logger;

import NavData.RobotPose2d;
import VisionData.VisualObject;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * concrete start-up / runner for the Birmingham / CogX environment
 * 
 * @author ttoenige
 * 
 */
public class ExemplarySolutionRunner extends ManagedComponent implements Runner {

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
				ManipulatorName.EXEMPLARY_SOLUTION, this, itemMemory, null,
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
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void runComponent() {
		new ExecutionGUI(manipulator);
	}

	/**
	 * add a robot base position listener to the CAST working memory
	 */
	public void addBaseMovementListener() {

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((ExemplarySolutionDoraBaseConnector) manipulator
								.getBaseConnector()).robotPositionChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((ExemplarySolutionDoraBaseConnector) manipulator
								.getBaseConnector()).robotPositionChanged(_wmc);
					}
				});
	}

	/**
	 * add the vision listener to the CAST working memorys
	 */
	public void addVisionListener() {
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((ExemplarySolutionBlortConnector) manipulator
								.getCamConnector()).visualObjectChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((ExemplarySolutionBlortConnector) manipulator
								.getCamConnector()).visualObjectChanged(_wmc);
					}
				});
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void startStrategy(Strategy.Name strategyName) {
		Strategy strategy = null;
		switch (strategyName) {
		case MOBILE_MANIPULATION:
			strategy = new MobileManipulation(manipulator);
			break;
		case CALIBRATION:
			strategy = new CalibrationStrategy(manipulator);
			break;
		default:
			logger.error("Does not know the strategy name!");
			break;
		}
		strategy.startExecution();
	}
}
