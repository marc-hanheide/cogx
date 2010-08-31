/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import java.util.Vector;

public class EventTemplate {

    // no match, do not use given property
    public static final int noMatch        = -1;
    
    // given property exactly as specified    
    public static final int matchExact     = 0;

    // matching events have happened WITHIN the given bounds
    // e.g. matching events' time interval is a subset of the 
    // specified interval
    public static final int matchSubset    = 1;
    
    // matching events' property ENCOMPASSES the given bounds
    // e.g. matching events' location is at least the 
    // specified one (or a larger one)
    public static final int matchSuperset  = 2;

    // time, space or sub-/super-events intersect...
    public static final int matchIntersectionNotEmpty = 3;

   
    public EventID minEventID        = null;
    public EventID maxEventID        = null;

    public boolean matchApexEvent    = false;
    public boolean apexEvent         = false; 

    public int minDegree             = Event.degreeUndefined;
    public int maxDegree             = Event.degreeUndefined;

    public EventType eventType       = null;
    public boolean   exactTypeMatch  = true;  // false: subtypes match, too.

    public EventTime time            = null;
    public int       timeMatchMode   = noMatch;

    public EventLocation   location            = null;
    public int             locationMatchMode   = noMatch;

    // must match exactly, if given
    public byte[]          binaryEventData     = null;


    public Vector<EventID> subEventIDs         = null;
    public int             subEventMatchMode   = noMatch;
 
    public Vector<EventID> superEventIDs       = null;
    public int             superEventMatchMode = noMatch;
    

    public Vector<PhysicalEntityID> physicalEntityIDs       = null;
    public int                      physicalEntityMatchMode = noMatch;

    
    public EventSpecificFeatures    esf                     = null;
    /* 
	Set this to true to search for events for which a given feature 
	is defined irrespective of its actual value. Set to false to 
	match against both feature name and values.
    */
    public boolean                  esfRestrictMatchToKeys  = false;
    public int                      esfMatchMode            = noMatch; 
    

}


