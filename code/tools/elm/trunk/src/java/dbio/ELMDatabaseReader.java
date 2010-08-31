/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.dbio;

import java.sql.Connection;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.ResultSetMetaData;

import java.util.Vector;

import elm.dbio.names.*;
import elm.event.*;


public class ELMDatabaseReader {

    public static final int orderModeNone           = 0;
    public static final int orderModeApexDegreeTime = 1;
    public static final int orderModeTimeOnly       = 2;
    public static final int orderModeDefault        = orderModeApexDegreeTime;

    
    protected Connection            connection                          = null;
    protected EventLocationFactory  elFactory                           = null;

    protected PreparedStatement     ps_phys_entities_involved 	        = null;
    protected PreparedStatement     ps_binary_data		        = null;
    protected PreparedStatement     ps_assoc_string_data	        = null;
    protected PreparedStatement     ps_sub_events_of		        = null;
    protected PreparedStatement     ps_super_events_of	                = null;

    protected PreparedStatement     ps_events_by_id_range_with_data     = null;
    protected PreparedStatement     ps_events_by_id_range_without_data	= null;

    // -----------------------------------------
    private boolean debugMode                                           = false;
    // -----------------------------------------
    // number of subqueries included in the last call of composeQuery()
    // needed for BMDatabaseReader
    protected int sqNumber                                              = 0; 
    // -----------------------------------------
    
    public ELMDatabaseReader(Connection c,
			     EventLocationFactory elFactory) 
	throws SQLException {

	connection = c;

	this.elFactory = elFactory;


	ps_phys_entities_involved = connection.prepareStatement("SELECT " + ColumnNames.event_id +
								", " + ColumnNames.phys_entity_id  +
								" FROM " + 
								TableNames.phys_entities_involved + 
								" WHERE " + ColumnNames.event_id + 
								" = ?");

	ps_binary_data = connection.prepareStatement("SELECT " + ColumnNames.event_id +
						    ", " + ColumnNames.binary_data +
						    " FROM " + 
						    TableNames.event_specific_binary_data + 
						    " WHERE " + ColumnNames.event_id + 
						    " = ?");

	ps_assoc_string_data = connection.prepareStatement("SELECT " + ColumnNames.event_id +
							   ", " + ColumnNames.feature_name +
							   ", " + ColumnNames.feature_value +
							   " FROM " + 
							   TableNames.event_specific_features + 
							   " WHERE " + ColumnNames.event_id + 
							   " = ?");

	ps_sub_events_of = connection.prepareStatement("SELECT " + ColumnNames.event_id +
						       ", " + ColumnNames.sub_event_id +
						       " FROM " + 
						       TableNames.sub_events + 
						       " WHERE " + ColumnNames.event_id + 
						       " = ?");

	ps_super_events_of = connection.prepareStatement("SELECT " + ColumnNames.event_id +
							  ", " + ColumnNames.sub_event_id +
							  " FROM " + 
							  TableNames.sub_events + 
							  " WHERE " + ColumnNames.sub_event_id + 
							  " = ?");

	ps_events_by_id_range_with_data = 
	    connection.prepareStatement("SELECT " + TableNames.elm + "." +
					ColumnNames.event_id + " AS " + 
					ColumnNames.event_id + ", " +
					ColumnNames.apex_event + ", " +
					ColumnNames.event_degree + ", " +
					" AsText(" + ColumnNames.location + 
					") AS " + ColumnNames.location + 
					", " + ColumnNames.start_time + 
					", " + ColumnNames.end_time + 
					", " + ColumnNames.event_type + 
					", " + ColumnNames.binary_data + 
					" FROM " + TableNames.elm + " LEFT OUTER JOIN " +
					TableNames.event_specific_binary_data + 
					" ON " + TableNames.elm + "." +
					ColumnNames.event_id + " = " + 
					TableNames.event_specific_binary_data + "." + 
					ColumnNames.event_id + " WHERE ? <= " + 
					TableNames.elm + "." + ColumnNames.event_id + 
					" AND " + TableNames.elm + "." + 
					ColumnNames.event_id + " <= ?");

	ps_events_by_id_range_without_data = 
	    connection.prepareStatement("SELECT " + TableNames.elm + "." +
					ColumnNames.event_id + " AS " + 
					ColumnNames.event_id + ", " +
					ColumnNames.apex_event + ", " +
					ColumnNames.event_degree + ", " +
					" AsText(" + ColumnNames.location + 
					") AS " + ColumnNames.location + 
					", " + ColumnNames.start_time + 
					", " + ColumnNames.end_time + 
					", " + ColumnNames.event_type + 
					" FROM " + TableNames.elm + 
					" WHERE ? <= " + 
					TableNames.elm + "." + ColumnNames.event_id + 
					" AND " + TableNames.elm + "." + 
					ColumnNames.event_id + " <= ?");

    }

    public void setDebugMode(boolean value) {
	debugMode = value;
    }


    protected synchronized Vector<Event> readEvents(ResultSet rs, boolean fetchComplete) 
	throws SQLException, WKTParseException {
	  
	Vector<Event> eventVector = new Vector<Event>();
	
	// is event data included in the ResultSet???
	// if yes, we will read it out below
	// if not, we will silently ignore it...
	boolean haveEventSpecificBinaryData = false;
	ResultSetMetaData rsmd = rs.getMetaData();
	int columnCount = rsmd.getColumnCount();
	for (int i = 1; i <= columnCount; i++) {
	    if (ColumnNames.binary_data.equalsIgnoreCase(rsmd.getColumnName(i)))
		haveEventSpecificBinaryData = true;
	}

	while (rs.next()) {

	    byte[] binaryEventData = null;	    
	    Event event = null;
	    Vector<EventID>          subEventIDs       = null;
	    Vector<PhysicalEntityID> physicalEntityIDs = null;
	    EventSpecificFeatures     esf    = null;
	    
	    EventID eventID = new EventID(rs.getLong(ColumnNames.event_id));
	    boolean apex = rs.getBoolean(ColumnNames.apex_event);
	    int eventDegree = rs.getInt(ColumnNames.event_degree);
	    EventType eventType = new EventType(rs.getString(ColumnNames.event_type));
	    
	    if (haveEventSpecificBinaryData)
		binaryEventData = rs.getBytes(ColumnNames.binary_data);

	    EventLocation location = elFactory.fromString(rs.getString(ColumnNames.location));
	    EventTime time = new EventTime(rs.getTimestamp(ColumnNames.start_time),
					   rs.getTimestamp(ColumnNames.end_time));
	    
	    if (fetchComplete) {
		subEventIDs = getSubEventIDs(eventID);
		physicalEntityIDs = getPhysicalEntityIDs(eventID);
		esf = getEventSpecificFeatures(eventID);
	    }
	    event = new DBEvent(eventID, apex, eventDegree, eventType, time, location, 
				binaryEventData, subEventIDs, physicalEntityIDs, esf);
	    
	    eventVector.add(event);
	}

	return eventVector;
    }


    public synchronized byte[] getEventSpecificBinaryData(EventID eventID)
	throws SQLException {

	byte[] binaryEventData = null;

	ps_binary_data.setLong(1, eventID.getLong());
	ResultSet rs = ps_binary_data.executeQuery();

	if (rs.next())
	    binaryEventData = rs.getBytes(ColumnNames.binary_data);

	return binaryEventData;
    }
   
    public synchronized Vector<EventID> getSubEventIDs(EventID eventID) throws SQLException {
	
	Vector<EventID> subEvents = new Vector<EventID>();

	ps_sub_events_of.setLong(1, eventID.getLong());
	ResultSet rs = ps_sub_events_of.executeQuery();

	while (rs.next()) 
	    subEvents.add(new EventID(rs.getLong(ColumnNames.sub_event_id)));
	
	return subEvents;
    }

    public synchronized Vector<EventID> getSuperEventIDs(EventID eventID) throws SQLException {
	
	Vector<EventID> superEvents = new Vector<EventID>();

	ps_super_events_of.setLong(1, eventID.getLong());
	ResultSet rs = ps_super_events_of.executeQuery();

	while (rs.next()) 
	    superEvents.add(new EventID(rs.getLong(ColumnNames.event_id)));
	
	return superEvents;
    }

    public synchronized Vector<PhysicalEntityID> getPhysicalEntityIDs(EventID eventID) throws SQLException {

	Vector<PhysicalEntityID> entities = new Vector<PhysicalEntityID>();

	ps_phys_entities_involved.setLong(1, eventID.getLong());
	ResultSet rs = ps_phys_entities_involved.executeQuery();

	while (rs.next()) 
	    entities.add(new PhysicalEntityID(rs.getString(ColumnNames.phys_entity_id)));

	return entities;
    }

    public synchronized EventSpecificFeatures getEventSpecificFeatures(EventID eventID) throws SQLException {

	EventSpecificFeatures esf = new EventSpecificFeatures();

	ps_assoc_string_data.setLong(1, eventID.getLong());
	ResultSet rs = ps_assoc_string_data.executeQuery();

	while (rs.next()) 
	    esf.addKeyValuePair(rs.getString(ColumnNames.feature_name),
				rs.getString(ColumnNames.feature_value));

	return esf;
    }

    protected synchronized  Vector<Event> readEventsPartial(ResultSet rs)
	throws SQLException, WKTParseException {

	return readEvents(rs, false);
    }

    protected synchronized  Vector<Event> readEventsComplete(ResultSet rs)
	throws SQLException, WKTParseException {

	return readEvents(rs, true);
    }

    public synchronized Vector<Event> getEventsByIDRange(long minID, long maxID) 
	throws SQLException, WKTParseException { 

	
	ps_events_by_id_range_with_data.setLong(1, minID);
	ps_events_by_id_range_with_data.setLong(2, maxID);
	ResultSet rs = ps_events_by_id_range_with_data.executeQuery();

// 	ps_events_by_id_range_without_data.setLong(1, minID);
// 	ps_events_by_id_range_without_data.setLong(2, maxID);
// 	ResultSet rs = ps_events_by_id_range_without_data.executeQuery();

	return readEventsComplete(rs);
	// return readEventsPartial(rs);
    }

    protected String composeRangeQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {

	String elmIDRange;

	if (template.minEventID == null && template.maxEventID != null)
	    elmIDRange = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + ColumnNames.event_id + 
		" <= " + template.maxEventID.getLong() + " ) ";
	else if (template.minEventID != null && template.maxEventID == null)
	    elmIDRange = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + template.minEventID.getLong() + 
		" <= " + ColumnNames.event_id  + " ) ";
	else if (template.minEventID != null && template.maxEventID != null)
	    elmIDRange = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + template.minEventID.getLong() + 
		" <= " + ColumnNames.event_id + " AND " + 
		ColumnNames.event_id + 
		" <= " + template.maxEventID.getLong() + " ) ";
	else
	    throw new ELMDatabaseReaderException("violated assumption for composeRangeQuery");
		
	return elmIDRange;
    }

    protected String composeApexQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {

	if (template.matchApexEvent)
	    throw new ELMDatabaseReaderException("violated assumption for composeApexQuery");

	String elmApex = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + ColumnNames.apex_event + " = " +
		( template.apexEvent ? " true " : " false ") +
		") ";	    
	return elmApex;
    }

    protected String composeDegreeQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {
	

	String elmDegree;
	    
	if (template.minDegree != elm.event.Event.degreeUndefined &&
	    template.maxDegree == elm.event.Event.degreeUndefined)
		
	    elmDegree = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + template.minDegree + 
		" <= " + ColumnNames.event_degree + ") ";
	else if (template.minDegree == elm.event.Event.degreeUndefined &&
		 template.maxDegree != elm.event.Event.degreeUndefined)

	    elmDegree = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + ColumnNames.event_degree + 
		" <= " + template.maxDegree + ") ";

	else if (template.minDegree != elm.event.Event.degreeUndefined &&
		 template.maxDegree != elm.event.Event.degreeUndefined)
		
	    elmDegree = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + template.minDegree + 
		" <= " + ColumnNames.event_degree + 
		" AND " + ColumnNames.event_degree + 
		" <= " + template.maxDegree + ") ";
	else
	    throw new ELMDatabaseReaderException("violated assumption for composeDegreeQuery");

	return elmDegree;
    }

    protected String composeTypeQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {
	
	if (template.eventType == null)
	    throw new ELMDatabaseReaderException("violated assumption for composeTypeQuery: " + 
						 "eventType may not be null");

	String elmType;

	if (template.exactTypeMatch)
	    elmType = 
		" ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + ColumnNames.event_type + " = " +
		"'" + template.eventType.getName() + "'" +
		") ";	    
	else
	    elmType = 
		" ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + ColumnNames.event_type + 
		" = '" + template.eventType.getName() + "'" +
		" OR " + ColumnNames.event_type + 
		" IN " +
		   " (SELECT " + ColumnNames.sub_type + 
		   " FROM " + TableNames.eth_trans_closure + 
		   " WHERE " + ColumnNames.event_type + " = " + 
		  	 "'" + template.eventType.getName() + "') " +
		") ";

	return elmType;
    }

    protected String composeTimeQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {
	
	if (template.time == null || template.timeMatchMode == template.noMatch)
	    throw new ELMDatabaseReaderException("violated assumption for composeTimeQuery");

	String elmTime;
	if (template.timeMatchMode == template.matchIntersectionNotEmpty) {
	    elmTime = 
		" ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + 
		/* 
		   using t.start <= t.end for all t, we get:
		   
		   NOT (    t1.end < t2.start 
		   OR  t2.end < t1.start)

		   iff

		   t1.end >= t2.start
		   AND t2.end >= t1.start
		*/
		" (to_timestamp(" + template.time.getBeginInSeconds() + ")" +
		" <= " + ColumnNames.end_time +
		" AND " + ColumnNames.start_time +
		" <= " + " to_timestamp(" + template.time.getEndInSeconds() + ")) )";

	}
	else {
	    String rel;

	    if (template.timeMatchMode == template.matchExact) 
		rel = " = "; 
	    else if (template.timeMatchMode == template.matchSubset) 
		rel = " <= "; 
	    else if (template.timeMatchMode == template.matchSuperset)
		rel = " >= "; 
	    else
		throw new ELMDatabaseReaderException("unknown matching mode for time");
	  
	    
	    elmTime = " ( SELECT " + ColumnNames.event_id + 
		" FROM " + TableNames.elm + 
		" WHERE " + " to_timestamp(" + template.time.getBeginInSeconds() + ")" +
		rel + ColumnNames.start_time + 
		" AND " + ColumnNames.end_time + rel + 
		" to_timestamp(" + template.time.getEndInSeconds() + ")) ";
	}


	return elmTime;
    }


    protected String composeLocationQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {
	
	if (template.location == null || template.locationMatchMode == template.noMatch)
	    throw new ELMDatabaseReaderException("violated assumption for composeLocationQuery");

	String elmLocation;
	String rel;

	if (template.locationMatchMode == template.matchExact) 
	    rel = "Equals"; 
	else if (template.locationMatchMode == template.matchSubset) 
	    rel = "Within";
	else if (template.locationMatchMode == template.matchSuperset)
	    rel = "Contains";
	else if (template.locationMatchMode == template.matchIntersectionNotEmpty)
	    rel = "Intersects";
	else
	    throw new ELMDatabaseReaderException("unknown matching mode for location");

	elmLocation = 
	    " ( SELECT DISTINCT " + ColumnNames.event_id + 
	    " FROM " + TableNames.elm + 
	    " WHERE " +
	    " " + rel + "(" + ColumnNames.location + ", " +  
	    template.location.getPostGISString() + ")) ";;

	return elmLocation;
    }



    // ------------ WARNING: code is relying on proprietary sun code ----------
    //                       change to something free occasionally?
    protected String composeDataQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {

	if (template.binaryEventData == null)
	    throw new ELMDatabaseReaderException("violated assumption for composeDataQuery: " + 
						 "binaryEventData may not be null");

	sun.misc.BASE64Encoder encoder = new sun.misc.BASE64Encoder();
	String encodedString = encoder.encode(template.binaryEventData);

	String dataQuery = 
	    " ( SELECT " + ColumnNames.event_id + 
	    " FROM " + TableNames.event_specific_binary_data + 
	    " WHERE " + ColumnNames.binary_data + 
	    " = decode('" + encodedString + "', 'base64') ) ";
	

	return dataQuery;
    }
    // ---------------------------------------------------------------------------


    protected <IDType extends LongID> String[] generateAssociatedLongItemsList(Vector<IDType> longIDs) 
	throws ELMDatabaseReaderException {

	if (longIDs.size() < 1)
	    throw new ELMDatabaseReaderException("for matching longID list may not be empty!");

	String[] itemsList = new String[longIDs.size()];

	for (int i = 0; i < longIDs.size(); i++) 
	    itemsList[i] = Long.toString(longIDs.get(i).getLong());
	
	return itemsList;
    }


    protected <IDType extends StringID> String[] generateAssociatedStringItemsList(Vector<IDType> stringIDs) 
	throws ELMDatabaseReaderException {

	if (stringIDs.size() < 1)
	    throw new ELMDatabaseReaderException("for matching StringID list may not be empty!");

	String[] itemsList = new String[stringIDs.size()];

	for (int i = 0; i < stringIDs.size(); i++) 
	    itemsList[i] = "'" + stringIDs.get(i).getString() + "'";
	
	return itemsList;
    }


    protected String composeSubEventQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {
	
	String[] associatedItemsList = generateAssociatedLongItemsList(template.subEventIDs);
	    
	return composeAssociatedDataQuery(TableNames.sub_events,
					  ColumnNames.sub_event_id,
					  addBrackets(associatedItemsList), 
					  template.subEventMatchMode);
    }

    protected String composeSuperEventQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {
	
	String[] associatedItemsList = generateAssociatedLongItemsList(template.superEventIDs);
	    
	return composeAssociatedDataQuery(TableNames.super_events,
					  ColumnNames.super_event_id,
					  addBrackets(associatedItemsList), 
					  template.superEventMatchMode);
    }


    protected String composePhysicalEntityQuery(EventTemplate template) 
	throws ELMDatabaseReaderException {
	
	String[] associatedItemsList = generateAssociatedStringItemsList(template.physicalEntityIDs);
	    
	return composeAssociatedDataQuery(TableNames.phys_entities_involved,
					  ColumnNames.phys_entity_id,
					  associatedItemsList, 
					  template.physicalEntityMatchMode);
    }


    // -------------- UNDER CONSTRUCTION ---------------------------------
    protected String composeESFQuery(EventTemplate template) 
	    throws ELMDatabaseReaderException {
	
	
	if (template.esfRestrictMatchToKeys) {
	    String[] associatedItemsList = new String[template.esf.size()];
	    associatedItemsList = addBrackets(addQuotes(template.esf.keySet().toArray(associatedItemsList)));
	    
	    return composeAssociatedDataQuery(TableNames.event_specific_features,
					      ColumnNames.feature_name,
					      associatedItemsList, 
					      template.esfMatchMode);
	}
	else {
	    String[] esfColumnNames = new String[2];
	    esfColumnNames[0] = ColumnNames.feature_name;
	    esfColumnNames[1] = ColumnNames.feature_value;
	    
	    Vector<String> associatedItemsList = new Vector<String>();
	    
	    java.util.Set<String> keySet = template.esf.keySet();
	    java.util.Iterator<String> keyIterator = keySet.iterator();

	    while (keyIterator.hasNext()) {
		String key = keyIterator.next();
		
		java.util.Set<String> values = template.esf.get(key);
		java.util.Iterator<String> valuesIterator = values.iterator();
		int valuesFound = 0;
		while (valuesIterator.hasNext()) {
		    valuesFound++;
		    associatedItemsList.add("('" + key + "', '" + valuesIterator.next() + "')");
		}
		if (valuesFound == 0)
		    throw new ELMDatabaseReaderException("If ESFs are matched for both feature name and feature values there must be at least one value per feature name (key). ESF object was: " + template.esf);
	    }
	    
	    return composeAssociatedDataQuery(TableNames.event_specific_features,
					      esfColumnNames,
					      associatedItemsList.toArray(new String[0]), 
					      template.esfMatchMode);
	}
    }
    
    protected String[] addQuotes(String[] a) {
	
	for (int i = 0; i < a.length; i++)
	   a[i] = "'" + a[i] + "'";
	return a;
    }
    
    protected String[] addBrackets(String[] a) {
	
	for (int i = 0; i < a.length; i++)
	   a[i] = "(" + a[i] + ")";
	return a;
    }
    
    
    // ------------------ THIS NEEDS AT LEAST postgresql 8.2 to work!!! ----------------
    // When passing values of (database) type string remember to addQuotes()!
    protected String valuesExpression(String[] assocItemsList) 
				      throws ELMDatabaseReaderException {
	
	return valuesExpression(assocItemsList, null, null);
    }

    // ------------------ THIS NEEDS AT LEAST postgresql 8.2 to work!!! ----------------
    // When passing values of (database) type string remember to addQuotes()!
    protected String valuesExpression(String[] assocItemsList,
				      String   tableName,
				      String[] columnNames) 
				      throws ELMDatabaseReaderException {

	StringBuffer buf = new StringBuffer(" ( VALUES ");

	for (int i = 0; i < assocItemsList.length; i++) {
	    if (i > 0)
		buf.append(", ");
	    // buf.append("(" + assocItemsList[i] + ")");
	    buf.append(assocItemsList[i]);
	}
	buf.append(") ");

	if (tableName != null && columnNames != null &&
	    columnNames.length > 0 && !tableName.equals(""))
	    
	    buf.append(" AS " + tableName + 
		       getColumnNamesList(null, columnNames, true) + " "); 
	
	return buf.toString();
    }

    // helper for composeAssociatedDataQuery() and valuesExpression()
    protected String getColumnNamesList(String table, String[] columns, boolean includeBrackets) 
					throws ELMDatabaseReaderException {
	
	if (columns.length < 1)
	    throw new ELMDatabaseReaderException("columns may not have length < 1");
	
	String t = (table == null ? "" : table + ".");
	StringBuffer res = new StringBuffer(includeBrackets ? "(" : ""); 

	for (int i = 0; i < columns.length - 1; i++) 
	    res.append(t + columns[i] + ", ");
	res.append(t + columns[columns.length - 1] + (includeBrackets ? ")" : ""));
	
	return res.toString();
    }
    
    protected String composeSingleValueQuery(String tableName,
					     String columnNameToSelect,
					     String columnNameForMatch,
					     String value) 
					     throws ELMDatabaseReaderException {
    
	return "( SELECT " + columnNameToSelect + " FROM " + tableName +
		" WHERE " + columnNameForMatch + " = " + value + " ) ";
    }
    protected String composeSingleValueQueries(String   tableName,
					       String   columnNameToSelect,
					       String   columnNameForMatch,
					       String[] values,
					       String   setOperation) 
					       throws ELMDatabaseReaderException {
    
	if (values.length < 1)
	    throw new ELMDatabaseReaderException("values[] array may not be empty!");
	
	StringBuffer res = new StringBuffer(" ( ");
	for (int i = 0; i < values.length - 1; i++) {
	    res.append(composeSingleValueQuery(tableName,
					       columnNameToSelect,
					       columnNameForMatch,
					       values[i]));
	    res.append(" " + setOperation + " ");
	}
	res.append(composeSingleValueQuery(tableName,
					   columnNameToSelect,
					   columnNameForMatch,
					   values[values.length-1]));
	
	res.append(" ) ");
	return res.toString();
    }
  
    protected String composeAssociatedDataQuery(String tableName,
						String assocDataColumnName,
						String[] associatedItemsList,
						int matchMode) 
						throws ELMDatabaseReaderException {
    
	String[] adcn = new String[1];
	adcn[0] = assocDataColumnName;
	return composeAssociatedDataQuery(tableName, adcn, associatedItemsList, matchMode);
    }
    
    
    protected String composeAssociatedDataQuery(String tableName,
						String[] assocDataColumnNames,
						String[] associatedItemsList,
						int matchMode) 
						throws ELMDatabaseReaderException {

	if (associatedItemsList == null || matchMode == EventTemplate.noMatch)
	    throw new ELMDatabaseReaderException("violated assumption for " +
						 "composeAssocDataQuery (" + 
						 tableName + ")");
	
	if (associatedItemsList.length == 0)
	    throw new ELMDatabaseReaderException("Sorry, matching against empty sets " +
						 "has not been implemented yet.");

	String eventIDColumnName = ColumnNames.event_id;
	
	String assocDataQuery;
	
	if (matchMode == EventTemplate.matchExact)	    
	    assocDataQuery = // no queryHead here
		    " ( " + 
		    composeAssociatedDataQuery(tableName,
					       assocDataColumnNames,
					       associatedItemsList, 
					       EventTemplate.matchSubset) +
		    " INTERSECT " + 
		    composeAssociatedDataQuery(tableName,
					       assocDataColumnNames,
					       associatedItemsList, 
					       EventTemplate.matchSuperset) +
		    " ) ";
		    
	else if (matchMode == EventTemplate.matchIntersectionNotEmpty)
	    assocDataQuery =
		    " ( SELECT DISTINCT " + eventIDColumnName +
		    " FROM " + 
		     composeSingleValueQueries(tableName, 
					       eventIDColumnName,
					       getColumnNamesList(null, assocDataColumnNames, true),
					       associatedItemsList,
					       "UNION") + 
		    " AS ad1 ) ";
		
	else if (matchMode == EventTemplate.matchSubset)
	    assocDataQuery =
		    " ( SELECT DISTINCT " + eventIDColumnName +
		    " FROM " + 
		     composeSingleValueQueries(tableName, 
					       eventIDColumnName,
					       getColumnNamesList(null, assocDataColumnNames, true),
					       associatedItemsList,
					       "INTERSECT") + 
		    " AS ad1 ) ";

	else if (matchMode == EventTemplate.matchSuperset) {
	    // Prevent a sequential table scan by working only on the 
	    // subset of events which match at least partially
	    String intersection = 
		    " ( SELECT " + eventIDColumnName + ", " +
		                   getColumnNamesList(null, assocDataColumnNames, false) +
		    " FROM " + 
		     composeSingleValueQueries(tableName, 
					       eventIDColumnName + ", " +
						       getColumnNamesList(null, assocDataColumnNames, false) ,
					       getColumnNamesList(null, assocDataColumnNames, true),
					       associatedItemsList, 
					       "UNION") + 
		    " AS sq_intersection ) ";
	    assocDataQuery =
		    " ( SELECT DISTINCT " + eventIDColumnName +
		    " FROM " + intersection + " AS ad1 " + 
		    " WHERE NOT EXISTS ( SELECT " + 
		    getColumnNamesList(null, assocDataColumnNames, false) +
		    " FROM " + valuesExpression(associatedItemsList, 
						"value_list", 
						assocDataColumnNames) + 
		    " WHERE " + getColumnNamesList("value_list", assocDataColumnNames, true) +
		    " NOT IN (SELECT " + getColumnNamesList(null, assocDataColumnNames, false) +
		    " FROM " + tableName + " ad2 " +
		    " WHERE ad1." + eventIDColumnName +
		    " = ad2." + eventIDColumnName +
		    " ))) ";
	}
	else
	    throw new ELMDatabaseReaderException("unknown matching mode for " +
		    "composeAssocDataQuery (" + 
		    tableName + ")");
	return assocDataQuery;
    }
    
    
    
    // OLD VERSION: 
    // POSTGRESQL DID NOT SEEM TO BE ABLE TO USE INDICES EFFICIENTLY WITH THESE QUERIES.
    //
    /*
    protected String composeAssociatedDataQuery(String tableName,
						String[] assocDataColumnNames,
						String[] associatedItemsList,
						int matchMode) 
						throws ELMDatabaseReaderException {

	if (associatedItemsList == null || matchMode == EventTemplate.noMatch)
	    throw new ELMDatabaseReaderException("violated assumption for " +
						 "composeAssocDataQuery (" + 
						 tableName + ")");
	
	if (associatedItemsList.length == 0)
	    throw new ELMDatabaseReaderException("Sorry, matching against empty sets " +
						 "has not been implemented yet.");

	String eventIDColumnName = ColumnNames.event_id;
	String queryHead = 
		" ( SELECT DISTINCT " + eventIDColumnName +
		" FROM " + tableName + " ad1 ";

	String assocDataQuery;
	
	if (matchMode == EventTemplate.matchExact)	    
	    assocDataQuery = // no queryHead here
		    " ( " + 
		    composeAssociatedDataQuery(tableName,
					       assocDataColumnNames,
					       associatedItemsList, 
					       EventTemplate.matchSubset) +
		    " INTERSECT " + 
		    composeAssociatedDataQuery(tableName,
					       assocDataColumnNames,
					       associatedItemsList, 
					       EventTemplate.matchSuperset) +
		    " ) ";
		    
	else if (matchMode == EventTemplate.matchIntersectionNotEmpty)
	    assocDataQuery =
		    queryHead + 
		    " WHERE " + getColumnNamesList(null, assocDataColumnNames, true) + 
		    " IN " + valuesExpression(associatedItemsList) +
		    " ) ";

	else if (matchMode == EventTemplate.matchSubset)
	    assocDataQuery =
		    queryHead + 
		    " WHERE NOT EXISTS ( SELECT " + eventIDColumnName +
		    " FROM " + tableName + " ad2 " +
		    " WHERE ad1." + eventIDColumnName +
		    " = ad2." + eventIDColumnName +
		    " AND " + getColumnNamesList("ad2", assocDataColumnNames, true) +
		    " NOT IN " + valuesExpression(associatedItemsList) + 
		    " )) ";

	else if (matchMode == EventTemplate.matchSuperset)
	    assocDataQuery =
		    queryHead + 
		    " WHERE NOT EXISTS ( SELECT " + 
		    getColumnNamesList(null, assocDataColumnNames, false) +
		    " FROM " + valuesExpression(associatedItemsList, 
						"value_list", 
						assocDataColumnNames) + 
		    " WHERE " + getColumnNamesList("value_list", assocDataColumnNames, true) +
		    " NOT IN (SELECT " + getColumnNamesList(null, assocDataColumnNames, false) +
		    " FROM " + tableName + " ad2 " +
		    " WHERE ad1." + eventIDColumnName +
		    " = ad2." + eventIDColumnName +
		    " ))) ";

	else
	    throw new ELMDatabaseReaderException("unknown matching mode for " +
		    "composeAssocDataQuery (" + 
		    tableName + ")");
	return assocDataQuery;
    }
    */

    // uses orderDefault as orderMode
    public synchronized Vector<Event> getMatchingEvents(EventTemplate template,
							int limit,
							int offset)

 	throws SQLException, WKTParseException, ELMDatabaseReaderException {

	return getMatchingEvents(template, limit, offset, orderModeDefault);
    }

    public synchronized Vector<Event> getMatchingEvents(EventTemplate template,
							int limit,
							int offset,
							int orderMode)

 	throws SQLException, WKTParseException, ELMDatabaseReaderException {
	
	String query = composeQuery(template, limit, offset, orderMode);
	return getMatchingEvents(query);
    }
    
    protected synchronized String composeQuery(EventTemplate template,
				 	       int limit,
					       int offset,
					       int orderMode) 

 	// throws WKTParseException, ELMDatabaseReaderException {
	throws ELMDatabaseReaderException {

	if (template == null)
	    throw new ELMDatabaseReaderException("template may not be null");
	if (limit < 0)
	    throw new ELMDatabaseReaderException("limit may not be negative");
	if (offset < 0)
	    throw new ELMDatabaseReaderException("offset may not be negative");

	boolean rangeQuery = template.minEventID != null 
	                  || template.maxEventID != null;

	boolean apexQuery  = template.matchApexEvent;

	boolean degreeQuery = template.minDegree != elm.event.Event.degreeUndefined 
	                   || template.maxDegree != elm.event.Event.degreeUndefined;

	boolean typeQuery   = template.eventType != null;

	boolean timeQuery   = template.time != null 
	                   && template.timeMatchMode != template.noMatch;

	boolean locationQuery = template.location != null 
	                     && template.locationMatchMode != template.noMatch;

	boolean dataQuery =  template.binaryEventData != null;
	
	boolean subEventQuery = template.subEventIDs != null 
	                     && template.subEventMatchMode != template.noMatch;

	boolean superEventQuery = template.superEventIDs != null 
	                       && template.superEventMatchMode != template.noMatch;

	boolean physEntityQuery = template.physicalEntityIDs != null 
	                       && template.physicalEntityMatchMode != template.noMatch;

	boolean esfQuery = template.esf != null 
			&& template.esfMatchMode != template.noMatch;
	
	if (!(rangeQuery || apexQuery || degreeQuery || typeQuery ||
	      timeQuery  || locationQuery || dataQuery || 
	      subEventQuery || superEventQuery || physEntityQuery || esfQuery))
	    throw new ELMDatabaseReaderException("template may not be empty!");

	String subQueriesName   = "sub_queries";
	StringBuffer subQueries = new StringBuffer("( ");
	sqNumber                = 0;  // number of subqueries included

	if (rangeQuery) {
	    subQueries.append(composeRangeQuery(template));
	    sqNumber++;
	}


	if (apexQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeApexQuery(template));
	    sqNumber++;	    
	}


	if (degreeQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeDegreeQuery(template));
	    sqNumber++;	    
	}


	if (typeQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeTypeQuery(template));
	    sqNumber++;	    
	}


	if (timeQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeTimeQuery(template));
	    sqNumber++;	    
	}

	if (locationQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeLocationQuery(template));
	    sqNumber++;	    
	}

	if (dataQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeDataQuery(template));
	    sqNumber++;	    
	}

	if (subEventQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeSubEventQuery(template));
	    sqNumber++;	    
	}

	if (superEventQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeSuperEventQuery(template));
	    sqNumber++;	    
	}

	if (physEntityQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composePhysicalEntityQuery(template));
	    sqNumber++;	    
	}

	if (esfQuery) {

	    if (sqNumber > 0) 
		subQueries.append(" INTERSECT ");

	    subQueries.append(composeESFQuery(template));
	    sqNumber++;	    
	}

	
	subQueries.append(" ) AS " + subQueriesName);


	String elmQuery = "SELECT DISTINCT " + TableNames.elm + "." +
	    ColumnNames.event_id + " AS " + 
	    ColumnNames.event_id + ", " +
	    ColumnNames.apex_event + ", " +
	    ColumnNames.event_degree + ", " +
	    " AsText(" + ColumnNames.location + 
	    ") AS " + ColumnNames.location + 
	    ", " + ColumnNames.start_time + 
	    ", " + ColumnNames.end_time + 
	    ", " + ColumnNames.event_type + 
	    ", " + ColumnNames.binary_data + 
	    " FROM " + TableNames.elm + 
	    " LEFT OUTER JOIN " + TableNames.event_specific_binary_data + 
	    " ON " + TableNames.elm + "." +
	    ColumnNames.event_id + " = " + 
	    TableNames.event_specific_binary_data + "." + 
	    ColumnNames.event_id + 
	    " INNER JOIN " + subQueries.toString() + 
	    " ON " + TableNames.elm + "." +
	    ColumnNames.event_id + " = " + 
	    subQueriesName + "." + ColumnNames.event_id + " ";

	String orderClause;
	
	if (orderMode == orderModeApexDegreeTime)
	    orderClause = " ORDER BY " + 
		ColumnNames.apex_event + " DESC, " +
		ColumnNames.event_degree + " DESC, " +
		ColumnNames.start_time + " ASC, " +
		ColumnNames.end_time + " ASC ";
	else if (orderMode == orderModeTimeOnly)
	    orderClause = " ORDER BY " + 
		ColumnNames.start_time + " ASC, " +
		ColumnNames.end_time + " ASC ";
	else if (orderMode == orderModeNone)
	    orderClause = "";
	else
	    throw new ELMDatabaseReaderException("invalid order mode"); 
	   

	StringBuffer completeQuery = new StringBuffer(elmQuery + orderClause);
	if (limit > 0)
	    if (offset > 0)
		completeQuery.append(" LIMIT " + limit + " OFFSET " + offset);
	    else
		completeQuery.append(" LIMIT " + limit);
	
	return completeQuery.toString();
    }

    protected synchronized Vector<Event> getMatchingEvents(String completeQuery)
 	throws SQLException, WKTParseException, ELMDatabaseReaderException {
	
	if (debugMode)
	    System.out.println("about to send:\n" + completeQuery.toString() + "\n\n");
	
	Statement statement = connection.createStatement();
	ResultSet rs = statement.executeQuery(completeQuery.toString());

	// return readEventsComplete(rs);

	return readEventsComplete(rs);
    }
    
    
}






