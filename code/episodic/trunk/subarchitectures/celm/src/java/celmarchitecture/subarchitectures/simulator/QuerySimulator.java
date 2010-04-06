package celmarchitecture.subarchitectures.simulator;

import java.util.Map;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import celm.autogen.CELMEventQuery;
import celm.autogen.CELMStoredEvent;
import celm.conversion.ConversionException;
import celm.conversion.EventConverter;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;
import elm.event.EventTemplate;
import elm.event.EventTime;

/**
 * QuerySimulator is a process which can be used to quickly test whether
 * recollection for recent events works. It repeatedly asks which events
 * happened in the last 2 seconds.
 * 
 * @author Dennis Stachowicz
 */
public class QuerySimulator extends ManagedComponent {

	public static final int qLimit = 10;

	private boolean localVerbose = false; // true;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	private EventConverter converter = new EventConverter();

	private boolean singleSA = true; // false;
	private SANames saNames = new SANames();

	public QuerySimulator() {
		super();
	}

	protected void configure(Map<String, String> config) {
		saNames.configure(config);
	}

	/*
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();
	}

	private void processRecollectionResults(WorkingMemoryChange _ceventChange) {

		try {

			CELMEventQuery query = getMemoryEntry(_ceventChange.address,
					CELMEventQuery.class);

			// for (int i = 0; i < query.events.length; i++)
			// println(i + ": " + converter.getEvent(query.events[i]));

			deleteFromWorkingMemory(_ceventChange.address);

			if (verbose)
				println("deleted query (and results) from working memory");

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
		// catch (WKTParseException e) {
		// e.printStackTrace();
		// }

		// catch (ConversionException e) {
		// e.printStackTrace();
		// }

	}

	protected void generateQuery() throws ConversionException,
			SubarchitectureComponentException {

		if (verbose)
			println("generating a new query...");

		EventTemplate template = new EventTemplate();

		// what happened in (or at least overlapping) the last 2 seconds?
		long currentTime = System.currentTimeMillis();
		template.time = new EventTime(currentTime - 2000, currentTime);
		template.timeMatchMode = template.matchIntersectionNotEmpty;

		String dataID = newDataID();
		if (singleSA) {
			addChangeFilter(ChangeFilterFactory.createIDFilter(dataID,
					WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							processRecollectionResults(_wmc);
						}
					});

			addToWorkingMemory(dataID, new CELMEventQuery(qLimit, converter
					.getEventCue(template), new CELMStoredEvent[0]));
		} else {
			addChangeFilter(ChangeFilterFactory.createAddressFilter(dataID,
					saNames.recollectionSA, WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							processRecollectionResults(_wmc);
						}
					});

			addToWorkingMemory(dataID, saNames.recollectionSA,
					new CELMEventQuery(qLimit, converter.getEventCue(template),
							new CELMStoredEvent[0]));

		}

		if (verbose)
			println("new query (and change filter!) added.");

	}

	/*
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
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

					generateQuery();

					// let other stuff happen if necessary
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
		} catch (ConversionException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}

		// FINALLY CLAUSE FOR unlockProcess???

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
