package manipulation.muster.runner.cogx;

import java.util.Map;

import manipulation.muster.core.cogx.CogXManipulatorStore;
import manipulation.muster.core.cogx.baseConnector.MasterThesisDoraBaseConnector;
import manipulation.muster.core.cogx.camConnector.MasterThesisBlortConnector;
import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.ManipulatorStore;
import manipulation.muster.core.share.Manipulator.ManipulatorName;
import manipulation.muster.core.share.armConnector.ArmConnector;
import manipulation.muster.core.share.armConnector.ArmConnector.ArmName;
import manipulation.muster.core.share.types.Configuration;
import manipulation.muster.itemMemory.ItemMemory;
import manipulation.muster.runner.share.Runner;
import manipulation.muster.strategies.CalibrationStrategy;
import manipulation.muster.strategies.MobileManipulation;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.visualisation.ExecutionGUI;

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
public class MasterThesisRunner extends ManagedComponent implements Runner {

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
				ManipulatorName.MASTER_THESIS, this, itemMemory, null,
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
						((MasterThesisDoraBaseConnector) manipulator
								.getBaseConnector()).robotPositionChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				RobotPose2d.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((MasterThesisDoraBaseConnector) manipulator
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
						((MasterThesisBlortConnector) manipulator
								.getCamConnector()).visualObjectChanged(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						((MasterThesisBlortConnector) manipulator
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
