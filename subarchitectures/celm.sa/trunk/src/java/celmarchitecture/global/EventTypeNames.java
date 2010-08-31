/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celmarchitecture.global;

/**
 *  A wrapper for globals constants holding the names of event types.
 */
public class EventTypeNames {

    public static final String beWelcomedUponEntry      = "be welcomed upon entry";
    public static final String beIgnoredUponEntry       = "be ignored upon entry";
    
    public static final String beApproached             = "be approached";
    public static final String beGreeted                = "be greeted";
    public static final String enterRoom                = "enter room";
    public static final String leaveRoom                = "leave room";
    
    public static final String phonString               = "PhonString";
    
    // define more subtypes for dialogue move?
    public static final String dialogueMove             = "DialogueMove";
    public static final String dialogueMoveOPENING      = "DialogueMoveOPENING";
    
    public static final String spokenOutputItem         = "SpokenOutputItem";


}