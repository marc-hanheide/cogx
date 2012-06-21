package de.dfki.lt.tr.cast.dialogue;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.ListIterator;
import java.util.Map;
import java.util.Map.Entry;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import castutils.CASTTimeUtil;
import eu.cogx.beliefs.slice.GroundedBelief;

public class GBeliefMemory implements Serializable{
	
	private static final long serialVersionUID = 1L;
	private Map<WorkingMemoryAddress,Map<CASTTime,GroundedBelief>> gbhistory = new HashMap<WorkingMemoryAddress,Map<CASTTime,GroundedBelief>>();
	
	public void addGBelief(WorkingMemoryAddress _wma, CASTTime ct, GroundedBelief _newBelief) {
	
		if (gbhistory.containsKey(_wma)) {
			gbhistory.get(_wma).put(ct, _newBelief);
		} else {
			gbhistory.put(_wma, new LinkedHashMap<CASTTime,GroundedBelief>());
			gbhistory.get(_wma).put(ct, _newBelief);
		}
	}
	
	public GroundedBelief getGBelief (WorkingMemoryAddress _wma, CASTTime ct) {
		
		ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
				new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
		
		while (iter.hasPrevious()) {
			Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
			if (CASTTimeUtil.diff(ct, entry.getKey()) >= 0) {
				return entry.getValue();
			}
		}
		return null;
	}
	
    public GroundedBelief getLastGBelief (WorkingMemoryAddress _wma) {
    	
    	ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
				new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
		
		while (iter.hasPrevious()) {
			Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
			return entry.getValue();
		}
		return null;
	}
    
    public GroundedBelief getLastValidGBelief (WorkingMemoryAddress _wma) {
		
		ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
				new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
		
		while (iter.hasPrevious()) {
			Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
			if (entry.getValue() != null) {
				  return entry.getValue();
			}
		}
		return null;
	}
	
	public GroundedBelief getValidGBelief (WorkingMemoryAddress _wma, CASTTime ct) {
		
		ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
				new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
			
		while (iter.hasPrevious()) {
			Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
			if (CASTTimeUtil.diff(ct, entry.getKey()) >= 0) {
				if (entry.getValue() != null) {
					  return entry.getValue();
				}
			}
		}
		return null;
	}
	
    public Map<CASTTime,GroundedBelief> getAllGBelief (WorkingMemoryAddress _wma) {
		
		if (gbhistory.containsKey(_wma)) {
			return gbhistory.get(_wma);
		} else {
			return null;
		}
	}
    
    public Map<CASTTime,GroundedBelief> getAllGBeliefBefore (WorkingMemoryAddress _wma, CASTTime ct) {
    	
    	Map<CASTTime,GroundedBelief> returnMap = new LinkedHashMap<CASTTime,GroundedBelief>();
			
	    for (Map.Entry<CASTTime,GroundedBelief> entry : gbhistory.get(_wma).entrySet()) {
				
			if (CASTTimeUtil.diff(ct, entry.getKey()) >= 0) {
				returnMap.put(entry.getKey(), entry.getValue());
			}
			else {
				break;
			}
		}
		return returnMap;
	}
    
    public Boolean ceasedToExist (WorkingMemoryAddress _wma) {
    	
    	if (getLastGBelief(_wma) == null) {
    		return true;
    	}
    	return false;
    }
}
