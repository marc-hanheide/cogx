package celmarchitecture.subarchitectures.elmwriter;

import java.util.Map;
import java.util.concurrent.PriorityBlockingQueue;

import NavData.RobotPose2d;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import celm.autogen.CELMEventLocation;
import celm.autogen.CELMEventToStore;
import celm.autogen.CELMPartialEventToStore;
import celm.conversion.EventConverter;
import celm.util.CastELMPartialEventComparator;
import celm.util.PositionNotFoundException;
import celm.util.TimedPosition;
import celm.util.TimedPositionBuffer;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;
import elm.event.EventLocation;
import elm.event.EventLocationFactory;
import elm.event.EventSpecificFeatures;

/**
 * LocationMonitor keeps track of the robot's position via RobotPose2d updates and
 * stores this information to fuse it with partial event information. <br>
 * "RobotPose2d + CELMPartialEventToStore = CELMEventToStore"
 * 
 * @author Dennis Stachowicz
 */
public class LocationMonitor extends ManagedComponent {

	public static final double bufferDistance = GlobalSettings.defaultAtomicBuffer;
	public static final int posBufferSize = 1024 * 1024;
	public static final int pbqCapacity = 1024;

	public static final String tooEarlyKey = "LocationMonitorNOTE";
	public static final String tooEarlyValue = "TOO_EARLY_NO_POSITION";

	public static final double[] loc0Coords = new double[] { 0, 0 };
	public static final double loc0Buffer = 0.01;

	private TimedPositionBuffer posBuffer = new TimedPositionBuffer(
			posBufferSize);

	private PriorityBlockingQueue<CELMPartialEventToStore> pEventQueue = new PriorityBlockingQueue<CELMPartialEventToStore>(
			pbqCapacity, new CastELMPartialEventComparator());

	private boolean localVerbose = false; // true;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	private boolean strictMatch = false;

	private boolean inELMWriterSA = false || GlobalSettings.singleSA;
	private SANames saNames = new SANames();

	// might need adjustment later on...
	private EventLocationFactory elf = new EventLocationFactory();

	private EventConverter converter = new EventConverter();

	public LocationMonitor() {
		super();
	}

	protected void configure(Map<String, String> config) {
		saNames.configure(config);
	}

	@Override
	public void start() {

			WorkingMemoryChangeReceiver wmcrRobotPose2d = new WorkingMemoryChangeReceiver() {

				public void workingMemoryChanged(WorkingMemoryChange _wmc) {

					// log(CASTUtils.toString(_wmc));
					addRobotPose2d(_wmc);
				}
			};

			WorkingMemoryChangeReceiver wmcrPartialEvent = new WorkingMemoryChangeReceiver() {

				public void workingMemoryChanged(WorkingMemoryChange _wmc) {

					// log(CASTUtils.toString(_wmc));
					addPartialEvent(_wmc);
				}
			};

			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					RobotPose2d.class, WorkingMemoryOperation.ADD), wmcrRobotPose2d);
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					RobotPose2d.class, WorkingMemoryOperation.OVERWRITE),
					wmcrRobotPose2d);

			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					CELMPartialEventToStore.class, WorkingMemoryOperation.ADD),
					wmcrPartialEvent);


	}

	private void addRobotPose2d(WorkingMemoryChange _ceventChange) {

		try {
			if (verbose)
				println("found RobotPose2d");

			CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.address);

			RobotPose2d pose = (RobotPose2d) wme.getData();

			posBuffer.addPosition(new TimedPosition(pose));

			if (verbose)
				println("stored RobotPose2d");
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	private void addPartialEvent(WorkingMemoryChange _ceventChange) {
		// get the action from wm
		try {

			CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.address);

			CELMPartialEventToStore partialCEvent = (CELMPartialEventToStore) wme
					.getData();

			if (verbose)
				println("found partial event of type \""
						+ partialCEvent.eventType + "\" adding to queue...");

			pEventQueue.add(partialCEvent);

			deleteFromWorkingMemory(_ceventChange.address);

			if (verbose)
				println("deleted partial event from WM");

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	@Override
	public void runComponent() {

		try {
			// prepare a default location for the case we need to record
			// "early events"
			EventLocation loc0 = elf.fromPoint(loc0Coords, loc0Buffer);
			String loc0String = loc0.getWKTString();

			while (posBuffer.empty())
				Thread.sleep(10);

			while (isRunning()) {
				// do nothing for a while
				// Thread.sleep(1000);
				// Thread.sleep(5);

				// must check we're still running after sleep!
				if (isRunning()) {

					CELMPartialEventToStore pets = pEventQueue.take();

					if (verbose)
						println("got:" + "\nbegin:    "
								+ pets.eventTime.begin.milliseconds
								+ "\nEARLIEST: "
								+ posBuffer.getEarliestTimeValue()
								+ "\nend:      "
								+ pets.eventTime.end.milliseconds
								+ "\nLATEST:   "
								+ posBuffer.getLatestTimeValue());

					if (pets.eventTime.begin.milliseconds < posBuffer
							.getEarliestTimeValue()) {
						// harsh reaction for the moment,
						// can be adjusted later
						if (posBuffer.full()) {
							println("POSITION BUFFER SIZE TOO SMALL!!! HAVE TO DROP PARTIAL EVENT...");

							if (GlobalSettings.exitOnException)
								System
										.exit(GlobalSettings.exitValueOnException);
						} else {
							if (strictMatch)
								println("PARTIAL EVENT TOO EARLY, no position "
										+ "information available. Sorry, have to DROP IT!");
							else {
								// use a fixed default location, ...
								CELMEventToStore ets = converter
										.getEventToStore(pets,
												new CELMEventLocation(
														loc0String));

								if (verbose)
									println("got an early event of type "
											+ ets.event.eventType
											+ ", saving it with a default location and a note");

								// ... add a note about it...
								EventSpecificFeatures esf = converter
										.toEventSpecificFeatures(ets.event.eventSpecificFeatures);
								esf.addKeyValuePair(tooEarlyKey, tooEarlyValue);
								ets.event.eventSpecificFeatures = converter
										.toCEventSpecificFeatures(esf);

								// ... and finally save it
								if (inELMWriterSA)
									addToWorkingMemory(newDataID(), ets);
								else
									addToWorkingMemory(newDataID(),
											saNames.writerSA, ets);
							}
						}
					} else if (pets.eventTime.end.milliseconds > posBuffer
							.getLatestTimeValue()) {
						// we do not know yet about positions at that time,
						// but we will later (hopefully)...
						if (verbose)
							println("too early, waiting for more position information to come...");
						pEventQueue.add(pets);
						Thread.sleep(500);
					} else {
						// lock from external access // NEEDED???
						lockComponent();

						double[][] coordinatesArray = posBuffer
								.getPositionArray(
										pets.eventTime.begin.milliseconds,
										pets.eventTime.end.milliseconds);

						EventLocation loc = elf.fromLinestring(
								coordinatesArray, bufferDistance);

						addToWorkingMemory(newDataID(), converter
								.getEventToStore(pets, new CELMEventLocation(
										loc.getWKTString())));

						// let other stuff happen if necessary // NEEDED?
						unlockComponent();

						if (verbose)
							println("added full event");
					}

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
		/*
		 * catch (ConversionException e) { e.printStackTrace(); }
		 */
		catch (PositionNotFoundException e) {
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
