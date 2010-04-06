package celmarchitecture.subarchitectures.simulator;

import java.io.IOException;
import java.util.Map;

import NavData.RobotPose2d;
import cast.SubarchitectureComponentException;
import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;
import celm.autogen.CELMEventSpecificFeaturesEntry;
import celm.autogen.CELMEventTime;
import celm.autogen.CELMEventTimestamp;
import celm.autogen.CELMEventToStore;
import celm.autogen.CELMPartialEventToStore;
import celm.conversion.EventConverter;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;
import elm.event.Event;
import elm.event.EventIDException;
import elm.tests.EventSimulator;

/**
 * Simulator can generate fake events of different kinds: full events, partial
 * events and RobotPoses. Useful for testing and development, not necessary in
 * an integrated system.
 * 
 * @author Dennis Stachowicz
 */
public class Simulator extends ManagedComponent {

	public static final boolean storeEventsLocally = GlobalSettings.singleSA;

	public static final boolean createFullEvents = true;
	public static final boolean createPartialEvents = true;
	public static final boolean createRobotPoses = true;

	public static final int robotPosesCnt = 10;

	private boolean localVerbose = false;
	private boolean verbose = GlobalSettings.verbose || localVerbose;
	private SANames saNames = new SANames();

	private EventSimulator privSimulator = new EventSimulator();
	private EventConverter converter = new EventConverter();

	public Simulator() {
		super();
	}

	protected void configure(Map<String, String> config) {
		saNames.configure(config);
	}

	private CELMEventToStore generateEventToStore() {

		CELMEventToStore ce = null;

		try {

			Event e = privSimulator.moveAndGetAtomicEvent();

			if (verbose) {
				println("generated EventToStore\n");
				CASTTime bt = getCASTTime();
				println("BALTTime: " + bt.s + ", " + bt.us
						+ "\nmatches Java Date: "
						+ celm.conversion.CASTTimeConverter.toJavaDate(bt));

			}

			ce = converter.getEventToStore(e);

		} catch (EventIDException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
		return ce;
	}

	private CELMPartialEventToStore generatePartialEventToStore() {

		CELMEventTime etime = new CELMEventTime(new CELMEventTimestamp(System
				.currentTimeMillis() - 2000), new CELMEventTimestamp(System
				.currentTimeMillis()));

		CELMPartialEventToStore ce = new CELMPartialEventToStore(
				"partial test type 1", etime, new String[0],
				new CELMEventSpecificFeaturesEntry[0], new byte[0]);

		return ce;
	}

	private RobotPose2d generateRobotPose() {

		RobotPose2d rp = new RobotPose2d();

		rp.time = getCASTTime();

		privSimulator.move();
		double[] pos = privSimulator.getPosition();

		rp.x = pos[0];
		rp.y = pos[1];
		rp.theta = 0;

		return rp;
	}

	@Override
	public void runComponent() {
		try {
			while (isRunning()) {
				// do nothing for a while
				Thread.sleep(1000);
				// Thread.sleep(5);

				// must check we're still running after sleep!
				if (isRunning()) {

					// lock from external access // NEEDED???
					lockComponent();

					if (verbose)
						println("simulating...");

					if (createFullEvents)
						if (storeEventsLocally)
							addToWorkingMemory(newDataID(),
									generateEventToStore());
						else
							addToWorkingMemory(newDataID(), saNames.writerSA,
									generateEventToStore());

					if (createRobotPoses) {
						for (int i = 0; i < robotPosesCnt; i++)
							if (storeEventsLocally)
								addToWorkingMemory(newDataID(),
										generateRobotPose());
							else
								addToWorkingMemory(newDataID(),
										saNames.writerSA, generateRobotPose());
					}

					if (createPartialEvents)
						if (storeEventsLocally)
							addToWorkingMemory(newDataID(),
									generatePartialEventToStore());
						else
							addToWorkingMemory(newDataID(), saNames.writerSA,
									generatePartialEventToStore());

					unlockComponent();
				}
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}

		// FINALLY CLAUSE FOR unlockComponent???

	}

	@Override
	protected void taskAdopted(String _taskID) {
	}

	@Override
	protected void taskRejected(String _taskID) {
	}

}
