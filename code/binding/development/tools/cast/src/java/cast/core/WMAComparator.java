package cast.core;

import java.util.Comparator;

import cast.cdl.WorkingMemoryAddress;

/**
 * Comparator for working memory addresses.
 * @author nah
 *
 */
public class WMAComparator implements Comparator<WorkingMemoryAddress> {

	public int compare(WorkingMemoryAddress _o1, WorkingMemoryAddress _o2) {		
		int res = _o1.m_id.compareTo(_o2.m_id); 
		if(res == 0) {
			return _o1.m_subarchitecture.compareTo(_o2.m_subarchitecture); 
		}
		else {
			return res;
		}
	}

}
