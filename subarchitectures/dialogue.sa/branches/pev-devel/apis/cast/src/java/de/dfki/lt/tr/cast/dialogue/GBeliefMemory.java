package de.dfki.lt.tr.cast.dialogue;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.ListIterator;
import java.util.Map;

import cast.DoesNotExistOnWMException;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import castutils.CASTTimeUtil;
import eu.cogx.beliefs.slice.GroundedBelief;

public class GBeliefMemory implements Serializable{
	
	private static final long serialVersionUID = 1L;
	private Map<WorkingMemoryAddress,Map<CASTTime,GroundedBelief>> gbhistory = new HashMap<WorkingMemoryAddress,Map<CASTTime,GroundedBelief>>();
	private Map<Integer, Map<Integer, CASTTime>> timeStampMap = new HashMap<Integer,Map<Integer,CASTTime>>();
	
	public void addTimeStamp(int taskID, int poplanID, CASTTime ctt) {
	  if (poplanID == 0){
		timeStampMap.put(taskID, new HashMap<Integer, CASTTime>());
		timeStampMap.get(taskID).put(poplanID, ctt);
	  } else {
		  timeStampMap.get(taskID).put(poplanID, ctt);
	  }
		
	}
	
	public CASTTime getTimeStamp(int taskID, int poplanID) {
		return timeStampMap.get(taskID).get(poplanID);
			
	}
	
	public void addGBelief(WorkingMemoryAddress _wma, CASTTime ct, GroundedBelief _newBelief) {
	
		if (gbhistory.containsKey(_wma)) {
			gbhistory.get(_wma).put(ct, _newBelief);
		} else {
			gbhistory.put(_wma, new LinkedHashMap<CASTTime,GroundedBelief>());
			gbhistory.get(_wma).put(ct, _newBelief);
		}
	}
	
	public GroundedBelief getGBelief (WorkingMemoryAddress _wma, CASTTime ct) throws DoesNotExistOnWMException {
		
		if (gbhistory.containsKey(_wma)) {
			ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
					new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
			
			while (iter.hasPrevious()) {
				Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
				if (CASTTimeUtil.diff(ct, entry.getKey()) >= 0) {
					return entry.getValue();
				}
			}
		}
		throw new DoesNotExistOnWMException();
	}
	
    public GroundedBelief getGBelief (WorkingMemoryAddress _wma, int taskID, int poplanID) throws DoesNotExistOnWMException {
		
    	if (gbhistory.containsKey(_wma)) {
	    	ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
					new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
			
			while (iter.hasPrevious()) {
				Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
				if (CASTTimeUtil.diff(getTimeStamp(taskID, poplanID), entry.getKey()) >= 0) {
					return entry.getValue();
				}
			}
    	}
    	throw new DoesNotExistOnWMException();
	}
	
    public GroundedBelief getLastGBelief (WorkingMemoryAddress _wma) throws DoesNotExistOnWMException {
    	
    	if (gbhistory.containsKey(_wma)) {
	    	ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
					new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
			
			while (iter.hasPrevious()) {
				Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
				return entry.getValue();
			}
    	}
    	
    	throw new DoesNotExistOnWMException();
	}
    
    public GroundedBelief getLastValidGBelief (WorkingMemoryAddress _wma) throws DoesNotExistOnWMException {
		
		if (gbhistory.containsKey(_wma)) {
    	
			ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
				new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
		
			while (iter.hasPrevious()) {
				Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
				if (entry.getValue() != null) {
					return entry.getValue();
				}
			}
		}
		throw new DoesNotExistOnWMException();
	}
	
	public GroundedBelief getValidGBelief (WorkingMemoryAddress _wma, CASTTime ct) throws DoesNotExistOnWMException {
		
		if (gbhistory.containsKey(_wma)) {
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
		}
		throw new DoesNotExistOnWMException();
	}
	
	public GroundedBelief getValidGBelief (WorkingMemoryAddress _wma, int taskID, int poplanID) throws DoesNotExistOnWMException {
		
		if (gbhistory.containsKey(_wma)) {
			ListIterator<Map.Entry<CASTTime,GroundedBelief>> iter =
					new ArrayList(gbhistory.get(_wma).entrySet()).listIterator(gbhistory.get(_wma).size());
				
			while (iter.hasPrevious()) {
				Map.Entry<CASTTime, GroundedBelief> entry = iter.previous();
				if (CASTTimeUtil.diff(getTimeStamp(taskID, poplanID), entry.getKey()) >= 0) {
					if (entry.getValue() != null) {
						  return entry.getValue();
					}
				}
			}
		}
		throw new DoesNotExistOnWMException();
	}
	
    public Map<CASTTime,GroundedBelief> getAllGBelief (WorkingMemoryAddress _wma) throws DoesNotExistOnWMException {
		
		if (gbhistory.containsKey(_wma)) {
			return gbhistory.get(_wma);
		} 
		throw new DoesNotExistOnWMException();
	}
    
    public Map<CASTTime,GroundedBelief> getAllGBeliefBefore (WorkingMemoryAddress _wma, CASTTime ct) throws DoesNotExistOnWMException {
    	
    	if (gbhistory.containsKey(_wma)) {
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
    	throw new DoesNotExistOnWMException();
	}
    
    public Map<CASTTime,GroundedBelief> getAllGBeliefBefore (WorkingMemoryAddress _wma, int taskID, int poplanID) throws DoesNotExistOnWMException {
	   
	   if (gbhistory.containsKey(_wma)) {
    	
		    Map<CASTTime,GroundedBelief> returnMap = new LinkedHashMap<CASTTime,GroundedBelief>();
				
		    for (Map.Entry<CASTTime,GroundedBelief> entry : gbhistory.get(_wma).entrySet()) {
					
				if (CASTTimeUtil.diff(getTimeStamp(taskID, poplanID), entry.getKey()) >= 0) {
					returnMap.put(entry.getKey(), entry.getValue());
				}
				else {
					break;
				}
			}
			return returnMap;
	   }
	   throw new DoesNotExistOnWMException();
	}
    
    public Boolean ceasedToExist (WorkingMemoryAddress _wma) throws DoesNotExistOnWMException {
    	
    	if (getLastGBelief(_wma) == null) {
    		return true;
    	}
    	return false;
    }
}
