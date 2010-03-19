package wmrecorder;

import java.io.Serializable;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryOperation;

public class WMRecorderEvent implements Serializable {
	private static final long serialVersionUID = 6056331073021120795L;
	
	public WorkingMemoryOperation type;
	public WorkingMemoryAddress address;
	public Ice.ObjectImpl object = null;
	public CASTTime time;
	public String component; 
	
	public WMRecorderEvent(WorkingMemoryOperation _type, WorkingMemoryAddress _addr, 
			CASTTime _time, Ice.ObjectImpl _obj, String _component) {
		this.type = _type;
		this.address = _addr;
		this.time = _time;
		if (this.type == WorkingMemoryOperation.ADD || this.type == WorkingMemoryOperation.OVERWRITE) {
			this.object = _obj;
		}
		this.component = _component;
	}
	
	public String toString() {
		String name;
		if (object != null) {
			name = object.getClass().getName(); 
		}
		else {
			name = "unknown object";
		}
		
		switch (type) {
		case ADD:
			return String.format("add %s at address %s by %s", 
					name, address.id, component);
		case OVERWRITE:
			return String.format("overwrite %s at address %s by %s", 
					name, address.id, component);
		case DELETE:
			return String.format("delete object at address %s by %s",
					address.id, component);
		default:
			assert false;
		}
		return "";
	}
}
