/** 
 *  \file CELM.idl
 *  This file defines structs in two modules: celm::autogen and locationConversion::autogen. 
 *  celm::autogen is the module for (almost) all interactions with the C-ELM system 
 *  whereas locationConversion::autogen contains interfaces to some other utilities 
 *  not considered core parts of C-ELM.
 *  <br>
 *  In case you wonder why there is this CELM prefix everywhere: 
 *  CORBA (3?) seems to have EventType as a keyword. 
 *  So this prefix is a workaround to avoid confusion and cryptic errors
 *  with Event* typenames. It might be dropped later on.
 *
 *  @author Dennis Stachowicz
 *
 * Converted to Slice by Nick Hawes, 10th June 2009
 */


#ifndef CELM_SLICE
#define CELM_SLICE


#include <cast/slice/CDL.ice>

module celm {

  /**
   *  celm::autogen defines the CAST interfaces for interactions with the C-ELM system.
   *  Other SAs may report events by writing CELMEventToStore or CELMPartialEventToStore 
   *  to ElmWriter's WM and query the memory store by writing a CELMEventQuery to Recollector's 
   *  WM.
   *  <br> 
   *  In case you wonder why there is this CELM prefix everywhere: 
   *  CORBA (3?) seems to have EventType as a keyword. 
   *  So this prefix is a workaround to avoid confusion and cryptic errors
   *  with Event* typenames. It might be dropped later on.
   *  @see locationConversion::autogen
   */  
  module autogen {

    
    /**
     *  Unspecific type for binary event data. Can be a serialised object, for example.
     *  Binary event data are event-specific and optional.
     */
     ["java:array"] sequence<byte> CELMEventSpecificBinaryData; 

    
    /**
     *  EventTimestamp - represents the onset or offset of an event.
     *  (Conversion utility from/to BALTTime (in Java): celm.conversion.BALTTimeConverter)
     */
    class CELMEventTimestamp {
      /**
       *  Milliseconds since January 1, 1970 00:00:00 GMT ("Epoch").
       *  In java you can get it from System.currentTimeMillis().
       */
      long milliseconds;
    };
    

    /**
     *  EventTime represents *both* start and end of an event.
     *  (Conversion utility in Java: celm.conversion.EventTimeConverter)
     */
    class CELMEventTime {
      CELMEventTimestamp begin;
      CELMEventTimestamp end;
    };
   
 


    /**
     *  Location of an event. 
     */
    class CELMEventLocation {
      /**
       *  A string representing a polygone in the WKT format.
       */
      string wktString;
    };


    sequence<long> LongList;    
    sequence<string> StringList;


    
    class CELMEventSpecificFeaturesEntry {
      string                     featureName;
      StringList                 featureValues;
    };

    /**
     *  Optional event-specific features. Representated in a string multi-map. 
     *  Can be used a generic means to store searchable and easily readable 
     *  data with an event.
     */
    sequence<CELMEventSpecificFeaturesEntry>  CELMEventSpecificFeatures;


    /**
     *  Definition of the core contents of an event. Used by 
     *  the CELMStoredEvent and CELMEventToStore classures.
     *  NOT to be written to any working memory directly.
     */
    class CELMCoreEvent {
     
      string                eventType;      

      CELMEventTime                eventTime;      
      CELMEventLocation            location;

      bool                       apexEvent;

      int                          eventDegree;

      LongList              subevents;

      CELMEventSpecificFeatures    eventSpecificFeatures;
      CELMEventSpecificBinaryData  eventSpecificBinaryData;
    };

    /**
     *  Representation of an event already stored in the database. 
     *  After storage ELMWriter writes it to the EventRecognizers WM.
     *  Can also be part of the answer to a CELMEventQuery. 
     */
    class CELMStoredEvent {

      long                eventID;
          
      CELMCoreEvent              event;
    };

    
    sequence<CELMStoredEvent> CELMStoredEventList;


    /**
     *  A fully specified (atomic or complex) event which
     *  shall be written to the database by ELMWriter.
     *  <br>
     *  Other subarchitectures / monitors may write this to 
     *  ELMWriter's WM. <br>
     *  Cave: If you do not know about the location you must use 
     *  a CELMPartialEventToStore class instead!!!
     *  @see CELMPartialEventToStore
     */
    class CELMEventToStore {
      
      CELMCoreEvent              event;
    };

    /**
     *  An incompletely specified (atomic!) event which
     *  shall be written to the database by ELMWriter
     *  after fusion with location information by LocationMonitor.
     *  <br>
     *  Other subarchitectures / monitors may write this to 
     *  ELMWriter's WM. <br>
     *  If you do already know about the location you should use 
     *  a CELMEventToStore class instead.
     *  @see CELMEventToStore
     */
    class CELMPartialEventToStore {

      string                eventType;      
      CELMEventTime                eventTime; 

      // below: optional information
      // if not needed, create 0-sized arrays 
      // or CORBA will make trouble (when passing null's)
      CELMEventSpecificFeatures    eventSpecificFeatures;
      CELMEventSpecificBinaryData  eventSpecificBinaryData;
    };

    /**
     *  CELMCueMatchMode defines modes for matching against 
     *  fields in a CELMEventCue used in a CELMEventQuery.
     *  @see CELMEventQuery
     *  @see CELMEventCue
     */
    enum CELMCueMatchMode { 

      /**
       *  No match, do NOT use given property as cue.
       */
      noMatch, 

      /**
       *  Match the given property EXACTLY as specified.
       */ 
      matchExact, 

      /**
       *  MATCHING EVENTS have happened WITHIN the given bounds,
       *  e.g. matching events' time interval is a subset of the 
       *  specified interval.
       */
      matchSubset, 
      
      /**
       *  MATCHING EVENTS' property ENCOMPASSES the given bounds,
       *  e.g. matching events' location is at least the 
       *  specified one (or a larger one).
       */
      matchSuperset, 

      /**
       *  There is at least one COMMON ELEMENT.
       */
      matchIntersectionNotEmpty 
    };


    /**
     *  A CELMEventCue class is similar to a CELMCoreEvent
     *  but can be underspecified. To leave a field unspecified 
     *  you can set the appropriate match flag to false or noMatch 
     *  and assign members some arbitrary values (except null references).
     *  <br>
     *  As part of a CELMEventQuery such a cue is matched with 
     *  previously stored events.
     *  @see CELMEventQuery
     *  @see CELMCueMatchMode
     */ 
    class CELMEventCue {

      bool                       matchEventID;
      long                  minEventID;
      long                  maxEventID;
     
      bool                       matchEventType;
      string                        eventType;
      bool                       eventTypeIncludeSubtypes;
      
      CELMCueMatchMode             timeMatchMode;
      CELMEventTime                eventTime;      
      
      CELMCueMatchMode             locationMatchMode;
      CELMEventLocation            location;
      
      bool                       matchApexEvent;
      bool                       apexEvent;

      bool                       matchEventDegree;
      int                          minEventDegree;
      int                          maxEventDegree;

      CELMCueMatchMode             subeventMatchMode;
      LongList              subevents;

      CELMCueMatchMode             supereventMatchMode;
      LongList              superevents;
       
      
      CELMCueMatchMode             esfMatchMode;
      bool                       esfRestrictMatchToKeys;
      CELMEventSpecificFeatures    eventSpecificFeatures;
      
      bool                       matchEventSpecificBinaryData;
      CELMEventSpecificBinaryData  eventSpecificBinaryData;
    };
    

    /**
     *  CELMEventQuery objects can be written to the
     *  working memory of the Recollector process which
     *  tries to find events matching the cue and return 
     *  them through the "events" member. For this to work
     *  the process issuing the query should register a 
     *  WM change receiver on this particular item before
     *  actually issuing it!
     *  @see CELMEventCue
     */
    class CELMEventQuery {
     
      /**
       *  How many events should be retrieved at most?
       *  Set this to -1 for the default behaviour.
       *  Set this to 0  for NO LIMIT.
       */
      int                  limit;

      // order???

      /**
       *  The cue against which events are matched. 
       */
      CELMEventCue         cue;

      
      /**
       *  This list of stored events should be empty 
       *  initially (a 0-sized array). The Recollector process 
       *  will fill it with matching events (if there are any).
       */
      CELMStoredEventList  events;
    };
  

        
  };
  
};


/**
  * The locationConversion module and the associated processes are work in progress.
  * NEVER TESTED! Use at your own risk or better: do not use it!
  * @see locationConversion::autogen
  */
module locationConversion {

  /** 
    *  The module locationConversion::autogen contains data classures describing the
    *  interface to the LocationConverter process which converts 'here', 
    *  'at (x, y)' or areaIDs from nav.sa to the CELMEventLocation format 
    *  (well-known text (WKT)) 
    *  
    *  @see    celm::autogen
    *  @author Dennis Stachowicz
    */
  module autogen {

    /**
     *  Returns a polygon approximating a circle with radius 
     *  bufferDistance around the current position.
     */
    class ConvertHere {

      double                            bufferDistance;

      /**
       *  Leave this empty (filled with an empty string)!
       *  It will be filled and the whole classure overwritten.
       */
      celm::autogen::CELMEventLocation location; 
    };

    /**
     *  Returns a polygon approximating a circle with radius 
     *  bufferDistance around the specified position.
     */
    class ConvertPosition {

      double                            x;
      double                            y;
      double                            bufferDistance;

      /**
       *  Leave this empty (filled with an empty string)!
       *  It will be filled and the whole classure overwritten.
       */
      celm::autogen::CELMEventLocation location; 
    };

    /**
     *  Returns a polygon approximating (!) the specified area
     *  based on FNodes from Nav.sa
     */
    class ConvertArea {

      long                              areaID;

      /**
       *  Leave this empty (filled with an empty string)!
       *  It will be filled and the whole classure overwritten.
       */
      celm::autogen::CELMEventLocation location; 
    };

  };

};


#endif
