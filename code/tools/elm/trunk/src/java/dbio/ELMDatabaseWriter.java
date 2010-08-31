/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.dbio;

import elm.dbio.names.*;
import elm.event.*;
import elm.tests.EventCounter;

import java.sql.Connection;
import java.sql.SQLException;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.Statement;

import java.io.Serializable;
import java.io.IOException;

import java.util.Date;
import java.util.Vector;
import java.util.Set;
import java.util.Iterator;

public class ELMDatabaseWriter
    // EventCounter interface is only temporary for benchmarking
    implements EventCounter {

    protected Connection        connection              = null;

    protected PreparedStatement ps_new_id               = null;
    protected PreparedStatement ps_event_to_type    	= null;
    protected PreparedStatement ps_phys_entities 	= null;
    protected Statement         s_elm		        = null;
    protected PreparedStatement ps_binary_data		= null;
    protected PreparedStatement ps_assoc_string_data	= null;
    protected PreparedStatement ps_sub_events		= null;
    protected PreparedStatement ps_update_apex		= null;
    

    // -- only for temporary benchmarking -------------------------
    protected boolean dbBenchmark                       = false;
    // protected String benchmarkConfigName             = "undefined";
    protected PreparedStatement ps_benchmark            = null;
    protected PreparedStatement ps_event_stats          = null;
    
    protected long t1, t2, t_se;

    public long newEventTime = 0, // ettTime = 0, 
	objInvTime = 0, elmTime = 0, seTime = 0, apexTime = 0, 
	binaryEventDataTime = 0, esfTime = 0, 
	storedEvents = 0, inStoreEventTime = 0, commitTime = 0;  
    public long esfFeaturesSum = 0, lastEventID = 0;
    // ------------------------------------------------------------ 

    public ELMDatabaseWriter(Connection c) throws SQLException {

	connection = c;
                
	ps_new_id = 
	    connection.prepareStatement("SELECT nextval('" + 
					SequenceNames.elm_event_id_seq + "')");	   
	ps_event_to_type = connection.prepareStatement("INSERT INTO " + 
						       TableNames.event_to_type + 
						       " (" + ColumnNames.event_id + 
						       ", " + ColumnNames.event_type + 
						       ") " + "VALUES (?, ?)");
	   
	ps_phys_entities = connection.prepareStatement("INSERT INTO " + 
						       TableNames.phys_entities_involved + 
						       " (" + ColumnNames.phys_entity_id + 
						       ", " + ColumnNames.event_id + 
						       ") " + "VALUES (?, ?)");

	s_elm = connection.createStatement();
	    
	ps_binary_data = connection.prepareStatement("INSERT INTO " + 
						    TableNames.event_specific_binary_data  + 
						    " (" + ColumnNames.event_id + 
						    ", " + ColumnNames.binary_data + 
						    ") VALUES (?, ?)");

	ps_assoc_string_data = connection.prepareStatement("INSERT INTO " + 
							   TableNames.event_specific_features  + 
							   " (" + ColumnNames.event_id + 
							   ", " + ColumnNames.feature_name +
							   ", " + ColumnNames.feature_value + 
							   ") VALUES (?, ?, ?)");

	    
	ps_sub_events = connection.prepareStatement("INSERT INTO " + 
						    TableNames.sub_events + 
						    " (" + ColumnNames.event_id +
						    ", " + ColumnNames.sub_event_id + 
						    ") VALUES (?, ?)");

	ps_update_apex = connection.prepareStatement("UPDATE " +
						     TableNames.elm + 
						     " SET " + 
						     ColumnNames.apex_event + 
						     " = false WHERE " + 
						     ColumnNames.event_id +
						     " = ?");
	    
    }

    public synchronized void flushMemoryStore()	throws SQLException {
	
	Statement s = connection.createStatement();
	s.executeUpdate("DELETE FROM " + TableNames.elm + ";");
    }

    
    private synchronized EventID getNewEventID() throws SQLException, EventIDException {

	EventID id = null;
	ResultSet rs = ps_new_id.executeQuery();
        if (!rs.next())
	    throw new EventIDException("could not get a new event ID");
	id = new EventID(rs.getInt(1));
	rs.close();
	// ----------------tmp----------------------
	lastEventID = id.getLong();
	// -----------------------------------------
	return id;
    }


    // returns the event_id assigned
    public synchronized EventID storeEvent(Event event)
	throws SQLException, EventIDException {

	t_se = currentTime();
	
	EventID eventID = null;

	try {
	    	    
	    // --- BEGIN TRANSACTION ---
	    connection.setAutoCommit(false);

	    
	    // 0: get eventID
	    t1 = currentTime();
	    eventID = getNewEventID();
	    event.assignEventID(eventID);
	    t2 = currentTime();
	    newEventTime += t2 - t1;
 
	    // 1: INSERT INTO elm (event_id, apex, degree, type, start_time, end_time, location)
	    String dBegin = Double.toString(event.getTime().getBeginInSeconds());
	    String dEnd   = Double.toString(event.getTime().getEndInSeconds());
    
	    t1 = currentTime();
	    s_elm.executeUpdate("INSERT INTO " + TableNames.elm + 
				" (" + ColumnNames.event_id +
				", " + ColumnNames.apex_event + 
				", " + ColumnNames.event_degree +
				", " + ColumnNames.event_type +
				", " + ColumnNames.start_time + 
				", " + ColumnNames.end_time + 
				", " + ColumnNames.location +
				") VALUES (" + eventID.getLong() + ", " +
				event.isApexEvent() + ", " +
				event.getDegree() + ", " + 
				"'" + event.getEventType().getName() + "'" +
				", to_timestamp(" + dBegin + "), " + 
				" to_timestamp(" + dEnd + "), " +
				event.getLocation().getPostGISString() + ")");
	    t2 = currentTime();
	    elmTime += t2 - t1;
		
	    // 2: INSERT INTO event_specific_binary_data (event_id, binary_data) 
	    if (event.hasEventSpecificBinaryData()) {
		t1 = currentTime();
		ps_binary_data.setLong(1, eventID.getLong());
		ps_binary_data.setBytes(2, event.getEventSpecificBinaryDataAsByteArray());
		ps_binary_data.executeUpdate();
		t2 = currentTime();
		binaryEventDataTime += t2 - t1;
	    }

	    // 3: INSERT INTO event_specific_features (event_id, feature_name, feature_value) 
	    if (event.getEventSpecificFeatures() != null) {
		t1 = currentTime();
		ps_assoc_string_data.setLong(1, eventID.getLong());

		Set<String> keySet = event.getEventSpecificFeatures().keySet();
		Iterator<String> keyIterator = keySet.iterator();

		if (dbBenchmark)
		    esfFeaturesSum = 0;
		while (keyIterator.hasNext()) {
		    String key = keyIterator.next();
		    ps_assoc_string_data.setString(2, key);
		    Set<String> values = event.getEventSpecificFeatures().get(key);
		    Iterator<String> valuesIterator = values.iterator();
		    while (valuesIterator.hasNext()) {
			ps_assoc_string_data.setString(3, valuesIterator.next());
			ps_assoc_string_data.executeUpdate();
			if (dbBenchmark)
			    esfFeaturesSum++;
		    }
		}

		
		t2 = currentTime();
		esfTime += t2 - t1;
	    }

	    // 4a: INSERT INTO sub_events (super_event_id, sub_event_id)
	    t1 = currentTime();
	    Vector<EventID> subEvents = event.getSubEventIDs();
	    if (subEvents != null) {
		ps_sub_events.setLong(1, eventID.getLong());
		for (int i = 0; i < subEvents.size(); i++) {
		    ps_sub_events.setLong(2, subEvents.get(i).getLong());
		    ps_sub_events.executeUpdate();
		}
	    }
	    t2 = currentTime();
	    seTime += t2 - t1;
	
	    // 4b: UPDATE sub_events (set apex_event := false)
	    t1 = currentTime();
	    if (subEvents != null) 
		for (int i = 0; i < subEvents.size(); i++) {
		    ps_update_apex.setLong(1, subEvents.get(i).getLong());
		    ps_update_apex.executeUpdate();
		}
	    
	    t2 = currentTime();
	    apexTime += t2 - t1;


	    // 5: INSERT INTO phys_entities_involved (phys_entity_id, event_id)		
	    t1 = currentTime();
	    Vector<PhysicalEntityID> entitiesInvolved = event.getPhysicalEntityIDs();
	    if (entitiesInvolved != null) {
		
		ps_phys_entities.setLong(2, eventID.getLong());		
		for (int i = 0; i < entitiesInvolved.size(); i++) {
		    ps_phys_entities.setString(1, entitiesInvolved.get(i).getString());
		    ps_phys_entities.executeUpdate();
		}
	    }
	    t2 = currentTime();
	    objInvTime += t2 - t1;
	}        
	finally {
	    t1 = currentTime();
	    connection.commit();
	    connection.setAutoCommit(true);
	    commitTime += currentTime() - t1;
	}
	// --- END TRANSACTION

	storedEvents++;  // temporary for testing and benchmarking
	inStoreEventTime += currentTime() - t_se;

	if (dbBenchmark) {
	    recordEventStatistics(event);
	    recordTimes(eventID);
	    resetTimers();	    
	}

	return eventID;
    }

    // --------------- below: benchmarking code -------------------------

    protected long currentTime() {

	// Caution: nanoTime is known to not work properly 
	// on MultiCore systems!!!
	return System.nanoTime() / 1000; // System.currentTimeMillis();
    }


    public void resetTimers() {

	newEventTime = 0; 
	// ettTime = 0; 
	objInvTime = 0; 
	elmTime = 0; 
	seTime = 0; 
	apexTime = 0; 
	binaryEventDataTime = 0;
	esfTime = 0; 
	inStoreEventTime = 0;
	commitTime = 0;  
    }
    
    public void setCurrentBMConfigName(String configName) throws SQLException {
	
	Statement s = connection.createStatement();
	s.executeUpdate("DELETE FROM bm_current; INSERT INTO bm_current(current_config) VALUES ('" + configName + "')");
    }
    
    public void turnOnDBBenchmark(String configName) throws SQLException {
	turnOnDBBenchmark(configName, -1);
    }
    
    public void turnOnDBBenchmark(String configName, int run_no) throws SQLException {

	resetEventCnt();
	
	dbBenchmark = true;
	// benchmarkConfigName = configName;
	// storedEvents = 0;
	ps_benchmark = connection.prepareStatement("INSERT INTO insertion_times (" + 
						   "event_id, " +
						   "new_event_id, " +
						   "obj_inv, " + 
						   "elm, " + 
						   "subevents, " +
						   "apex, " + 
						   "binary_data, " +
						   "esf, " +
						   "commit_time, " +
						   "in_store_event, " +
						   "event_cnt, " +
						   "run_no " +
						   ") VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, " +
							      run_no + ")");
	
	ps_event_stats = connection.prepareStatement("INSERT INTO event_statistics (" + 
						     "event_id, " +
 						     "config_name, " + 
						     "event_type_length, " +
						     "subevents_no, " + 
						     "esbd_size, " +
						     "esf_no, " +
						     "esf_sum " +   
						     ") VALUES (?, '" + configName + 
						     "', ?, ?, ?, ?, ?)"); 
    } 

    public void turnOffDBBenchmark() {
	dbBenchmark = false;
    }

    private void recordTimes(EventID eventID) throws SQLException {

	ps_benchmark.setLong(1, eventID.getLong());
	ps_benchmark.setLong(2, newEventTime);
	ps_benchmark.setLong(3, objInvTime);
	ps_benchmark.setLong(4, elmTime);
	ps_benchmark.setLong(5, seTime);
	ps_benchmark.setLong(6, apexTime);
	ps_benchmark.setLong(7, binaryEventDataTime);
	ps_benchmark.setLong(8, esfTime);
	ps_benchmark.setLong(9, commitTime);
	ps_benchmark.setLong(10, inStoreEventTime);
	ps_benchmark.setLong(11, storedEvents);

	ps_benchmark.executeUpdate();
    }

    private void recordEventStatistics(Event event) throws SQLException {

	long eventTypeLength, subeventsNo, esbdSize, esfNo;
	
	eventTypeLength = event.getEventType().getName().length();
	subeventsNo = (!event.isAtomic() ? event.getSubEventIDs().size() : 0);
	esbdSize = (event.hasEventSpecificBinaryData() ?
		    event.getEventSpecificBinaryDataAsByteArray().length : 0);
	esfNo = (event.getEventSpecificFeatures() != null ?
		 event.getEventSpecificFeatures().size() : 0);
/*	esfFeaturesSum = (event.getEventSpecificFeatures() != null ?
			  event.getEventSpecificFeatures().size() : 0);*/
	
	ps_event_stats.setLong(1, event.getEventID().getLong());
	ps_event_stats.setLong(2, eventTypeLength);
	ps_event_stats.setLong(3, subeventsNo);
	ps_event_stats.setLong(4, esbdSize);
	ps_event_stats.setLong(5, esfNo);
	ps_event_stats.setLong(6, esfFeaturesSum);

	ps_event_stats.executeUpdate();
    }

    public long getEventCnt() {
	return storedEvents;
    }
    public void resetEventCnt() {
	storedEvents = 0;
    }
    public long getLastEventID() {
	return lastEventID;
    }
    public void resetLastEventID() {
	lastEventID = 0;
    }
    
}
