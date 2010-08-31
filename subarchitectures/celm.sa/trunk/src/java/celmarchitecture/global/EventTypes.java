/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celmarchitecture.global;

import elm.event.EventType;

/**
 *  EventType equivalents of EventTypeNames constants. 
 */
public class EventTypes {
	
    public static final EventType beWelcomedUponEntry    = new EventType(EventTypeNames.beWelcomedUponEntry);
    public static final EventType beIgnoredUponEntry     = new EventType(EventTypeNames.beIgnoredUponEntry);
    
    public static final EventType beApproached           = new EventType(EventTypeNames.beApproached);
    public static final EventType beGreeted              = new EventType(EventTypeNames.beGreeted);
    public static final EventType enterRoom              = new EventType(EventTypeNames.enterRoom);
    public static final EventType leaveRoom              = new EventType(EventTypeNames.leaveRoom);
    
 
}