/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.util;

public class PositionNotFoundException extends Exception {

    public static final int TOO_LATE  = -1;
    public static final int TOO_EARLY = 1;

    private int direction = 0; // not set

    public PositionNotFoundException(String msg) {
	super(msg);
    }

    public PositionNotFoundException(int direction) {

	// The call to super must be the first one.
	// So need to do it with a trick.
	super("The requested TimedPositions " + 
	      (direction == TOO_LATE ? "have already expired from the buffer!" :
	       (direction == TOO_EARLY ? "are not yet in the buffer!" : 
		"... wait a second ... I don't know this direction value!")));

	/* // This is what we really meant: 
	if (direction == TOO_LATE)
	    super("The requested TimedPositions have " + 
		  "already expired from the buffer!");
	else if (direction == TOO_EARLY)
	    super("The requested TimedPositions are " + 
		  "not yet in the buffer!");
	else
	    super("exception squared: unknown direction " +
		  "(wrong use of this exception!)");
	*/

	this.direction = direction;
    }

    public int getDirection() {
	return direction;
    }
    

}
