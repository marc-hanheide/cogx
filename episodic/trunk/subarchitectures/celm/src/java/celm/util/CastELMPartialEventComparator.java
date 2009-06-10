/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.util;

import java.util.Comparator;
import celm.autogen.CELM_PartialEventToStore;

public class CastELMPartialEventComparator 
    implements Comparator<CELM_PartialEventToStore> {

    public int compare(CELM_PartialEventToStore e1, 
		       CELM_PartialEventToStore e2) {

	if (e1.m_eventTime.m_end.m_milliseconds < 
	    e2.m_eventTime.m_end.m_milliseconds)
	    return -1;
	else if (e1.m_eventTime.m_end.m_milliseconds > 
		 e2.m_eventTime.m_end.m_milliseconds)
	    return 1;
	
	else if (e1.m_eventTime.m_begin.m_milliseconds < 
		 e2.m_eventTime.m_begin.m_milliseconds)
	    return -1;
	else if (e1.m_eventTime.m_begin.m_milliseconds > 
		 e2.m_eventTime.m_begin.m_milliseconds)
	    return -1;

	else
	    return 0;
    }

    public boolean equals(CELM_PartialEventToStore e1, 
			  CELM_PartialEventToStore e2) {

	return e1.m_eventTime.m_end.m_milliseconds == 
	       e2.m_eventTime.m_end.m_milliseconds 
	    && e1.m_eventTime.m_begin.m_milliseconds == 
	       e2.m_eventTime.m_begin.m_milliseconds;
    
    }
}
