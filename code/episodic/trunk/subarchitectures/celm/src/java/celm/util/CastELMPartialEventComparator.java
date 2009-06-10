/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.util;

import java.util.Comparator;
import celm.autogen.CELMPartialEventToStore;

public class CastELMPartialEventComparator 
    implements Comparator<CELMPartialEventToStore> {

    public int compare(CELMPartialEventToStore e1, 
		       CELMPartialEventToStore e2) {

	if (e1.eventTime.end.milliseconds < 
	    e2.eventTime.end.milliseconds)
	    return -1;
	else if (e1.eventTime.end.milliseconds > 
		 e2.eventTime.end.milliseconds)
	    return 1;
	
	else if (e1.eventTime.begin.milliseconds < 
		 e2.eventTime.begin.milliseconds)
	    return -1;
	else if (e1.eventTime.begin.milliseconds > 
		 e2.eventTime.begin.milliseconds)
	    return -1;

	else
	    return 0;
    }

    public boolean equals(CELMPartialEventToStore e1, 
			  CELMPartialEventToStore e2) {

	return e1.eventTime.end.milliseconds == 
	       e2.eventTime.end.milliseconds 
	    && e1.eventTime.begin.milliseconds == 
	       e2.eventTime.begin.milliseconds;
    
    }
}
