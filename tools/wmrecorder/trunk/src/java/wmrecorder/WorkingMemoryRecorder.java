package wmrecorder;

import java.util.Map;

import cast.CASTException;
import cast.ConsistencyException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryOperation;

public class WorkingMemoryRecorder extends ManagedComponent implements WorkingMemoryChangeReceiver{
	private String filename;
	private EventQueue m_events = new EventQueue(); 
	
	@Override
	protected void configure(Map<String, String> _config) {
		super.configure(_config);
		String fname = _config.get("--file");
		if (fname != null) {
			filename = fname;
		}
		else {
			filename = "wm.log";
		}
	}

	@Override
	protected void start() {
		super.start();
		addChangeFilter(ChangeFilterFactory.createChangeFilter("", 
				WorkingMemoryOperation.WILDCARD, "", "", "", FilterRestriction.LOCALSA),
				this);
	}
	
	public void workingMemoryChanged(WorkingMemoryChange _wmc) throws CASTException {
		if (_wmc.operation == WorkingMemoryOperation.DELETE) {
			m_events.push(_wmc.operation, _wmc.address, _wmc.timestamp, null, _wmc.src);
		}
		else {
			System.out.println(_wmc.operation);
			try {
				Ice.ObjectImpl obj = getMemoryEntry(_wmc.address, Ice.ObjectImpl.class);
				m_events.push(_wmc.operation, _wmc.address, _wmc.timestamp, obj, _wmc.src);
			}
			catch (Exception e) {
				m_events.push(_wmc.operation, _wmc.address, _wmc.timestamp, null, _wmc.src);
			}
		}
	}
	
	@Override
	protected void runComponent() {
		while (isRunning()) {
			sleepComponent(1000);
		}
		log("saving history to " + filename);
		m_events.saveHistory(filename);
	}

}
