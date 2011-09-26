/**
 * @author hanheidm
 */
package vision.components;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import kinect.slice.KinectPerson;
import kinect.slice.PersonDetectorInterface;
import kinect.slice.PersonDetectorInterfacePrx;
import VisionData.PeopleDetectionCommand;
import VisionData.Person;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author hanheidm
 * 
 */
public class PersonDetectorRunner extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	private static final String GUI_KEY = "--gui";
	private static final String SERVER_KEY = "--server";
	private static final String CONTINUOUS_FREQ_KEY = "--continuous";
	private static final String INTEGRATE_DETECTIONS_KEY = "--integrate-detections";
	private static final long TIME_BETWEEN_DETECTIONS = 100;
	private String serverId = null;
	private PersonDetectorInterfacePrx server = null;
	private long continuousFreq = -1;
	private int integrateDetections = 1;
	private boolean useGui = false;

	@Override
	protected void configure(Map<String, String> config) {
		if (config.containsKey(SERVER_KEY)) {
			serverId = config.get(SERVER_KEY);
		}
		if (config.containsKey(CONTINUOUS_FREQ_KEY)) {
			continuousFreq = Long.parseLong(config.get(CONTINUOUS_FREQ_KEY));
		}
		if (config.containsKey(INTEGRATE_DETECTIONS_KEY)) {
			integrateDetections = Integer.parseInt(config
					.get(INTEGRATE_DETECTIONS_KEY));
		}
		if (config.containsKey(GUI_KEY)) {
			useGui = true;
		}
	}

	@Override
	protected void runComponent() {
		if (continuousFreq > 0) {
			println("running in continuous mode, with detection triggered every "
					+ continuousFreq + " ms.");
			while (isRunning()) {
				if (server == null)
					break;
				queryServer();
				sleepComponent(continuousFreq);
			}
		} else {
			println("running in trigger mode only");
		}
	}

	private synchronized Map<Integer, KinectPerson> detectPersons() {
		Map<Integer, KinectPerson> persons = queryServer();
		for (Entry<Integer, KinectPerson> p : persons.entrySet()) {
			debug("person found: id=" + p.getKey() + " size="
					+ p.getValue().size);
		}

		Person person = new Person();
		if (persons.size() > 0) {
			println("we found at least one person");
			person.existProb = 1.0;
		} else {
			println("no person found");
			person.existProb = 0.0;
		}
		try {
			addToWorkingMemory(newDataID(), person);
		} catch (CASTException e) {
			logException(e);
		}

		return persons;
	}

	private synchronized Map<Integer, KinectPerson> queryServer() {
		if (server == null)
			return new HashMap<Integer, KinectPerson>();
		Map<Integer, KinectPerson> persons = new HashMap<Integer, KinectPerson>();
		// integrate multiple detections
		for (int i = 0; i < integrateDetections; i++) {
			persons.putAll(server.getPersons());
			if (i < integrateDetections - 1) {
				try {
					Thread.sleep(TIME_BETWEEN_DETECTIONS);
				} catch (InterruptedException e) {
					logException(e);
					break;
				}
			}
		}
		return persons;
	}

	@Override
	protected void start() {
		try {
			if (serverId != null) {
				server = getIceServer(serverId, PersonDetectorInterface.class,
						PersonDetectorInterfacePrx.class);
				addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
						PeopleDetectionCommand.class,
						WorkingMemoryOperation.ADD), this);
			}
		} catch (CASTException e) {
			logException(e);
		}
		if (useGui) {
			new PersonDetectorGUI(this);

		}
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {
		// load command
		getMemoryEntry(_wmc.address, PeopleDetectionCommand.class);

		// do detection
		detectPersons();

		// executed the command, results (if any) are on working memory,
		// now delete command as not needed anymore
		deleteFromWorkingMemory(_wmc.address);

	}

}
