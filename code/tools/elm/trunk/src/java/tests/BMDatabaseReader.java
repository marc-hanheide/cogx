/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import java.sql.Connection;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.PreparedStatement;
import java.util.Vector;

import elm.event.*;
import elm.dbio.ELMDatabaseReader;
import elm.dbio.ELMDatabaseReaderException;
// import elm.tests.EventCounter;

public class BMDatabaseReader extends ELMDatabaseReader {

    // ------ temporary benchmarking code ------
    private boolean dbBenchmark = false;
    private long t1, queryConstructionTime = 0, queryExecutionTime = 0;
    private PreparedStatement ps_statistics = null;
    private EventCounter eventCounter = null;
    // -----------------------------------------

    public BMDatabaseReader(Connection c,
			    EventLocationFactory elFactory) 
	throws SQLException {

	super(c, elFactory);
    }
    
    public synchronized Vector<Event> getMatchingEvents(BMQuery bmq,
						        int limit,
							int offset,
							int orderMode)

 	throws SQLException, WKTParseException, ELMDatabaseReaderException {

	try {
	    t1 = currentTime();
	    String query = composeQuery(bmq.t, limit, offset, orderMode);
	    queryConstructionTime = currentTime() - t1;
	    t1 = currentTime();
	    Vector<Event> events = getMatchingEvents(query);
	    queryExecutionTime = currentTime() - t1;
	    
	    if (dbBenchmark) {
		recordQueryStatistics(bmq, events.size(), limit);
		resetTimers();
	    }
	    return events;
	}
	catch (ELMDatabaseReaderException e) {
	    System.err.println("caught exception in the context of event " + bmq.src);
	    System.err.println("will throw it again...");
	    throw e;
	}
	catch (SQLException e) {
	    System.err.println("caught exception in the context of event " + bmq.src);
	    System.err.println("will throw it again...");
	    throw e;
	}	
	catch (WKTParseException e) {
	    System.err.println("caught exception in the context of event " + bmq.src);
	    System.err.println("will throw it again...");
	    throw e;
	}	
    }
    
    protected long currentTime() {

	// Caution: nanoTime is known to not work properly 
	// on MultiCore systems!!!
	return System.nanoTime() / 1000; // System.currentTimeMillis();
    }


    public void resetTimers() {

	queryConstructionTime = 0;
	queryExecutionTime = 0;
    }
    
    public void turnOnDBBenchmark(String configName, 
				  int run_no,
				  EventCounter ec) 
	throws SQLException {

	dbBenchmark = true;
	// benchmarkConfigName = configName;
	eventCounter = ec;
	ps_statistics = connection.prepareStatement("INSERT INTO query_statistics (" +
						   "config_name, " +
						   "sqnumber, " +	
						   "min_event_id, " +
						   "max_event_id, " +
						   "match_apex, " +
						   "apex, " +
						   "min_degree, " +
						   "max_degree, " +
						   "exact_type_match, " +
						   "event_type, " +
						   "time_match_mode, " +
						   "start_time, " +
						   "end_time, " +
						   "location_match_mode, " +
						   "esbd_size, " +
						   "esf_match_mode, " + 
						   "restrict_esf_to_keys, " +
						   "esf_no, " +
						   "esf_sum, " +
						   "subevents_match_mode, " +
						   "no_of_subevents, " +
						   "superevents_match_mode, " +
						   "no_of_superevents, " +
						   "result_size, " +
						   "result_limit, " +
						   "construction_time, " +
						   "execution_time, " +
						   "event_cnt, " +
						   "last_event_id, " + 
						   "src_id, " +
						   "qcomment, " +
						   "run_no " +
						   ") VALUES ('" + configName + 
						   "', ?, ?, ?, ?, ?, " +
		                                   "   ?, ?, ?, ?, ?, " +
		                                   "   ?, ?, ?, ?, ?, " +
						   "   ?, ?, ?, ?, ?, " + 
						   "   ?, ?, ?, ?, ?, " +
						   "   ?, ?, ?, ?, ?, " + run_no + ")");
    } 

    public void turnOffDBBenchmark() {
	dbBenchmark = false;
    }


    private void recordQueryStatistics(BMQuery bmq, int resultSize, int limit) throws SQLException {

	ps_statistics.setInt(1, sqNumber);
	
	if (bmq.t.minEventID == null)
	    ps_statistics.setNull(2, java.sql.Types.INTEGER);
	else
	    ps_statistics.setLong(2, bmq.t.minEventID.getLong());
	

	if (bmq.t.maxEventID == null)
	    ps_statistics.setNull(3, java.sql.Types.INTEGER);
	else
	    ps_statistics.setLong(3, bmq.t.maxEventID.getLong());
	
	ps_statistics.setBoolean(4, bmq.t.matchApexEvent);
	ps_statistics.setBoolean(5, bmq.t.apexEvent);
	
	ps_statistics.setLong(6, bmq.t.minDegree);
	ps_statistics.setLong(7, bmq.t.maxDegree);
		
	ps_statistics.setBoolean(8, bmq.t.exactTypeMatch);
	if (bmq.t.eventType == null)
	    // ps_statistics.setNull(9, java.sql.Types.VARCHAR);  // ???
	    ps_statistics.setString(9, ""); 
	else
	    ps_statistics.setString(9,  bmq.t.eventType.getName());
	
	ps_statistics.setInt(10, bmq.t.timeMatchMode);
	if (bmq.t.time == null) {
	    ps_statistics.setNull(11, java.sql.Types.TIMESTAMP);
	    ps_statistics.setNull(12, java.sql.Types.TIMESTAMP);
	}    
	else {
	    ps_statistics.setTimestamp(11, new java.sql.Timestamp(bmq.t.time.getMicroTimeBegin()));
	    ps_statistics.setTimestamp(12, new java.sql.Timestamp(bmq.t.time.getMicroTimeEnd()));
	}

	ps_statistics.setInt(13, bmq.t.locationMatchMode);
	
	ps_statistics.setInt(14, (bmq.t.binaryEventData == null ? 0 : bmq.t.binaryEventData.length));
	
	ps_statistics.setInt(15, bmq.t.esfMatchMode);
	ps_statistics.setBoolean(16, bmq.t.esfRestrictMatchToKeys);
	ps_statistics.setInt(17, bmq.t.esf == null ? 0 : bmq.t.esf.size());
	ps_statistics.setInt(18, bmq.t.esf == null ? 0 : bmq.t.esf.totalNumberOfPairs());
    
	ps_statistics.setInt(19, bmq.t.subEventMatchMode);
	ps_statistics.setInt(20, (bmq.t.subEventIDs == null ? 0 : bmq.t.subEventIDs.size()));
	
	ps_statistics.setInt(21, bmq.t.superEventMatchMode);
	ps_statistics.setInt(22, (bmq.t.superEventIDs == null ? 0 : bmq.t.superEventIDs.size()));
	
	ps_statistics.setLong(23, resultSize);
	ps_statistics.setLong(24, limit);
	ps_statistics.setLong(25, queryConstructionTime);
	ps_statistics.setLong(26, queryExecutionTime);
	
	if (eventCounter == null)
	    ps_statistics.setNull(27, java.sql.Types.BIGINT);
	else
	    ps_statistics.setLong(27, eventCounter.getEventCnt());	

	ps_statistics.setLong(28, eventCounter.getLastEventID());
	
	if (bmq.src == null)
	    ps_statistics.setNull(29, java.sql.Types.INTEGER);
	else
	    ps_statistics.setLong(29, bmq.src.getLong());	
	
	if (bmq.comment == null || bmq.comment.equals(""))
	    ps_statistics.setNull(30, java.sql.Types.VARCHAR);
	else
	    ps_statistics.setString(30, bmq.comment);	

	
	ps_statistics.executeUpdate();
    }


}