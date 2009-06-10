package celmarchitecture.subarchitectures.monitors;

import java.util.Date;
import java.util.Properties;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryOperation;
import celm.autogen.CELMEventTime;
import celm.autogen.CELMPartialEventToStore;
import celm.conversion.EventConverter;
import celm.conversion.EventTimeConverter;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;
import elm.event.EventSpecificFeatures;

/**
 * SimpleAbstractWMMonitor provides functionality to simplify writing monitor
 * processes reporting events to the C-ELM system (mainly addPartialEvent()).
 * 
 * To use it just derive your monitor process from this class.
 * 
 * @author Dennis Stachowicz
 */
public class SimpleAbstractWMMonitor extends ManagedComponent {

	public static final boolean storeEventsLocally = GlobalSettings.singleSA;

	private boolean localVerbose = false;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	private SANames saNames = new SANames();

	public SimpleAbstractWMMonitor(String _id) {
		super();
	}

	public void configure(Properties config) {
		saNames.configure(config);
	}

	protected <Type extends Ice.Object> void addGlobalAddOverwriteFilter(Class<Type> c,
			WorkingMemoryChangeReceiver wmcrProcessEvent) {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
				WorkingMemoryOperation.ADD), wmcrProcessEvent);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
				WorkingMemoryOperation.OVERWRITE), wmcrProcessEvent);
	}

	protected <Type extends Ice.Object> void addGlobalAddFilter(Class<Type> c,
			WorkingMemoryChangeReceiver wmcrProcessEvent)
			 {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
				WorkingMemoryOperation.ADD), wmcrProcessEvent);

	}

	protected <Type extends Ice.Object> void addGlobalOverwriteFilter(Class<Type> c,
			WorkingMemoryChangeReceiver wmcrProcessEvent)
 {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
				WorkingMemoryOperation.OVERWRITE), wmcrProcessEvent);
	}

	protected <Type extends Ice.Object> void addGlobalDeleteFilter(Class<Type> c,
			WorkingMemoryChangeReceiver wmcrProcessEvent) {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(c,
				WorkingMemoryOperation.DELETE), wmcrProcessEvent);

	}

	/**
	 * Report an event to the episodic-like memory system.
	 * <p>
	 * Minimum requirement: you need to know about its type. All other
	 * parameters can be null and are filled with default values in that case.
	 * Particularly in the case of time the current system time is taken as both
	 * the start and end time of the event. However if you know at which time
	 * something happened, please specify this!
	 */
	protected void addPartialEvent(String eventType,
			byte[] eventSpecificBinaryData, Date startTime, Date endTime,
			String[] physicalEntityIDs, EventSpecificFeatures data)
			throws SubarchitectureComponentException {

		CELMEventTime eventTime;
		if (startTime == null || endTime == null) {
			long curTime = System.currentTimeMillis();
			eventTime = EventTimeConverter.toCEventTime(curTime, curTime);
		} else
			eventTime = EventTimeConverter.toCEventTime(startTime, endTime);

		CELMPartialEventToStore partialCEvent = new CELMPartialEventToStore(
				eventType,
				eventTime,
				(physicalEntityIDs == null ? new String[0] : physicalEntityIDs),
				EventConverter.toCEventSpecificFeatures(data),
				(eventSpecificBinaryData == null ? new byte[0]
						: eventSpecificBinaryData));

		if (verbose)
			println("adding partial event (type \"" + eventType
					+ "\") to ELM writer WM...");

		if (storeEventsLocally)
			addToWorkingMemory(newDataID(), partialCEvent);
		else
			addToWorkingMemory(newDataID(), saNames.writerSA, partialCEvent);

	}

	/*
	 * @see
	 * cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted
	 * (java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {
	}

	/*
	 * @see
	 * cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected
	 * (java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
	}

}
