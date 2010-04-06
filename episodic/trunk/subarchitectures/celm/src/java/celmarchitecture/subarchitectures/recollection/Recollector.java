package celmarchitecture.subarchitectures.recollection;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.util.Map;
import java.util.Vector;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import celm.autogen.CELMEventCue;
import celm.autogen.CELMEventQuery;
import celm.autogen.CELMStoredEvent;
import celm.conversion.ConversionException;
import celm.conversion.EventConverter;
import celmarchitecture.global.DBConfig;
import celmarchitecture.global.GlobalSettings;
import elm.dbio.ELMDatabaseReader;
import elm.dbio.ELMDatabaseReaderException;
import elm.event.Event;
import elm.event.EventLocationFactory;
import elm.event.EventTemplate;
import elm.event.WKTParseException;

/**
 * Recollector is a process which matches events from the ELM memory store with
 * a cue it finds on its working memory. <br>
 * Interaction with this process works as follows: Another process writes a
 * CELMEventQuery struct to Recollector's WM *after* registering a WM change
 * receiver on this particular item. Recollector overwrites the struct, filling
 * the events field. Now the calling process can read results and delete the
 * struct.
 * 
 * @see CELMEventQuery
 * @see CELMEventCue
 * @author Dennis Stachowicz
 */
public class Recollector extends ManagedComponent {

	public static final int defaultLimit = 50;

	private boolean localVerbose = false; // true;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	private EventConverter converter = new EventConverter();
	private ELMDatabaseReader dbReader = null;

	private DBConfig dbConfig = new DBConfig();

	private EventLocationFactory elFactory = new EventLocationFactory();

	public Recollector() {
		super();
	}

	protected void configure(Map<String, String> config) {
		dbConfig.configure(config);
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
			dbReader = new ELMDatabaseReader(connect("jdbc:postgresql://"
					+ dbConfig.server + "/" + dbConfig.name, dbConfig.user,
					dbConfig.passwd), elFactory);
			dbReader.setDebugMode(verbose);

			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					CELMEventQuery.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {

							answerQuery(_wmc);
						}
					});
		} catch (SQLException se) {
			println("Could not initialize ELMDatabaseReader.");
			se.printStackTrace();
			System.exit(GlobalSettings.exitValueOnException);
		}
	}

	private void answerQuery(WorkingMemoryChange wmChange) {

		try {
			if (verbose)
				println("found a new query...");

			CASTData<?> wme = getWorkingMemoryEntry(wmChange.address);
			CELMEventQuery query = (CELMEventQuery) wme.getData();

			EventTemplate template = converter.getEventTemplate(query.cue);

			int limit = query.limit < 0 ? defaultLimit : query.limit;

			Vector<Event> results = dbReader.getMatchingEvents(template, limit,
					0);

			CELMStoredEvent[] storedEvents = new CELMStoredEvent[results.size()];
			for (int i = 0; i < results.size(); i++)
				storedEvents[i] = converter.getStoredEvent(results.get(i));

			query.events = storedEvents;
			overwriteWorkingMemory(wmChange.address, query);

			if (verbose)
				println("results written to WM");

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (SQLException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (ELMDatabaseReaderException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (WKTParseException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (ConversionException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}

	}

	/*
	 * @see
	 * cast.architecture.subarchitecture.PrivilegedManagedComponent#taskAdopted
	 * (java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {
	}

	/*
	 * @see
	 * cast.architecture.subarchitecture.PrivilegedManagedComponent#taskRejected
	 * (java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
	}

}
