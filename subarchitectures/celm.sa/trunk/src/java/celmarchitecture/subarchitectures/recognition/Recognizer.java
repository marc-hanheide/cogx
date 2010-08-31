package celmarchitecture.subarchitectures.recognition;

import java.util.Map;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import celm.autogen.CELMStoredEvent;
import celm.conversion.EventConverter;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;
import elm.event.Event;
import elm.event.WKTParseException;
import elm.eventrecognition.RecognitionManager;
import elm.eventrecognition.SuperEventRecognizer;

/**
 * The Recognizer process is notified of events arrived in the ELM memory store
 * and passes them over to a RecognitionManager (where specific handlers can be
 * registered) for further processing. When one of the more specific handlers /
 * recognizers find that some events together form a complex event this is
 * reported back to the C-ELM system (to ElmWriter, in particular) starting a
 * new cycle of processing...
 * 
 * @see RecognitionManager
 * @author Dennis Stachowicz
 */
public class Recognizer extends ManagedComponent {

	private boolean localVerbose = false;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	private boolean singleSA = GlobalSettings.singleSA;
	private SANames saNames = new SANames();

	private EventConverter converter = new EventConverter();
	private RecognitionManager recManager = null;

	public Recognizer() {
		super();
		recManager = new RecognitionManager();
	}

	protected void registerRecognizer(SuperEventRecognizer _recognizer) {
		recManager.registerRecognizer(_recognizer);
	}

	protected void configure(Map<String, String> config) {
		saNames.configure(config);
	}

	protected void newEventDetected(Event event) {
		if (verbose)
			println("new event detected: " + event);

		// store or return immediately to elmwriter WM
		try {
			if (singleSA)
				addToWorkingMemory(newDataID(), converter
						.getEventToStore(event));
			else
				addToWorkingMemory(newDataID(), saNames.writerSA, converter
						.getEventToStore(event));
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	/*
	 * @see cast.architecture.abstr.WorkingMemoryReaderComponent#start()
	 */
	@Override
	public void start() {
		recManager.start();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				CELMStoredEvent.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						// log(CASTUtils.toString(_wmc));
						processEvent(_wmc);

					}
				});
	}

	private void processEvent(WorkingMemoryChange _ceventChange) {

		try {
			if (verbose)
				println("found event to process");

			CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.address);

			CELMStoredEvent cevent = (CELMStoredEvent) wme.getData();

			if (verbose)
				println("adding event to queue");
			recManager.newEvent(converter.getEvent(cevent));

			deleteFromWorkingMemory(_ceventChange.address);

			if (verbose)
				println("done");
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (WKTParseException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	/*
	 * @see cast.architecture.subarchitecture.ManagedComponent#taskAdopted
	 * (java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {
	}

	/*
	 * @see cast.architecture.subarchitecture.ManagedComponent#taskRejected
	 * (java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
	}

}
