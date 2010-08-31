package celmarchitecture.subarchitectures.elmwriter;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.util.Map;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import celm.autogen.CELMEventToStore;
import celm.conversion.EventConverter;
import celmarchitecture.global.DBConfig;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;
import elm.dbio.ELMDatabaseWriter;
import elm.event.AtomicEvent;
import elm.event.Event;
import elm.event.EventIDException;
import elm.event.EventLocationFactory;
import elm.event.EventTime;
import elm.event.EventType;
import elm.event.WKTParseException;

/**
 * ElmWriter provides an interface from CAST to the ELM memory store.
 * 
 * @author Dennis Stachowicz
 */
public class ElmWriter extends ManagedComponent {

	public static final String configKeyFlushDB = "--flush-db";

	private boolean flushDB = false;

	private boolean localVerbose = false;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	private boolean singleSA = GlobalSettings.singleSA;

	private DBConfig dbConfig = new DBConfig();
	private SANames saNames = new SANames();

	private EventConverter converter = new EventConverter();
	private ELMDatabaseWriter dbWriter = null;

	public ElmWriter() {
		super();
	}

	protected void configure(Map<String, String> config) {
		dbConfig.configure(config);
		saNames.configure(config);
		
		if (config.containsKey(configKeyFlushDB)) 
	    		flushDB = true;
	}

	private Connection connect(String url, String user, String password) {
		Connection connection = null;

		try {
			Class.forName("org.postgresql.Driver");
		} catch (ClassNotFoundException cnfe) {
			println("Could not find the driver!");
			cnfe.printStackTrace();
			System.exit(1);
		}

		if (verbose)
			println("Registered the driver.\nConnecting to the database... ");

		try {
			// The second and third arguments are the username and password,
			// respectively.They should be whatever is necessary to connect
			// to the database.
			connection = DriverManager.getConnection(url, user, password);
		} catch (SQLException se) {
			println("Could not connect.");
			se.printStackTrace();
			System.exit(1);
		}

		if (verbose)
			println("Connected.");

		return connection;
	}

	@Override
	public void start() {
		super.start();

		try {
			dbWriter = new ELMDatabaseWriter(connect("jdbc:postgresql://"
					+ dbConfig.server + "/" + dbConfig.name, dbConfig.user,
					dbConfig.passwd));

			if (flushDB) {
				if (verbose)
					println("flushing old ELM db entries...");
				dbWriter.flushMemoryStore();
			
			}			
			
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					CELMEventToStore.class, WorkingMemoryOperation.ADD),
			// addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(CELMEventToStore.class,
					// WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {

							storeEvent(_wmc);
						}
					});
			
			addWakeupEvent();
			if (verbose)
				println("ELMWriter waking up...");
		} catch (SQLException se) {
			println("Could not initialize ELMDatabaseWriter.");
			se.printStackTrace();

			System.exit(GlobalSettings.exitValueOnException);
		}
	}

	private void addWakeupEvent() {

		long time = System.currentTimeMillis();
		double[] pos0 = { 0.0, 0.0 };
		AtomicEvent event = new AtomicEvent(new EventType(
				"--ElmWriter wakes up--"), new EventTime(time, time),
				new EventLocationFactory().fromPoint(pos0, 0.0001),
				(byte[]) null, null, null);
		try {
			dbWriter.storeEvent(event);
		} catch (Exception e) {
			e.printStackTrace();

			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}

	}

	private void storeEvent(WorkingMemoryChange _ceventChange) {

		try {
			if (verbose)
				println("going to store event...");

			CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.address);
			CELMEventToStore cevent = (CELMEventToStore) wme.getData();

			Event event = converter.getEvent(cevent);
			dbWriter.storeEvent(event);

			if (verbose)
				println("stored an event");

			if (singleSA)
				addToWorkingMemory(newDataID(), converter.getStoredEvent(event));
			else
				addToWorkingMemory(newDataID(), saNames.recognitionSA,
						converter.getStoredEvent(event));

			deleteFromWorkingMemory(_ceventChange.address.id);

			if (verbose)
				println("deleted an event");

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (SQLException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (EventIDException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (WKTParseException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}

	}

	@Override
	protected void taskAdopted(String _taskID) {
	}

	@Override
	protected void taskRejected(String _taskID) {
	}

}
