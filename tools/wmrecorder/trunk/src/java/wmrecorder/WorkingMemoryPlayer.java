package wmrecorder;

import java.util.Map;

import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.CASTTime;

public class WorkingMemoryPlayer extends ManagedComponent {
	private String filename;
	private EventQueue m_events = null; 
	
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
		m_events = new EventQueue(filename);
	}
	
	@Override
	protected void runComponent() {

		WMRecorderEvent next = m_events.removeFirst();
		while (isRunning() && next != null) {
			CASTTime now = getCASTTime();
			long dt = 1000000 * (next.time.s - now.s) + (next.time.us - now.us);
			System.out.printf("Next event (%s) in %d ms.\n", next, dt/1000);
			if (dt > 1000) {
				sleepComponent(dt/1000);
			}
			applyEvent(next);
			next = m_events.removeFirst();
		}
	}
	
	private void applyEvent(WMRecorderEvent event) {
		try {
			switch (event.type) {
			case ADD:
				addToWorkingMemory(event.address, event.object);
				break;
			case OVERWRITE:
				getMemoryEntry(event.address, Ice.ObjectImpl.class);
				overwriteWorkingMemory(event.address, event.object);
				break;
			case DELETE:
				deleteFromWorkingMemory(event.address);
			default:
				assert false;
			}
			
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (PermissionException e) {
			e.printStackTrace();
		} catch (ConsistencyException e) {
			e.printStackTrace();
		}
		
	}

	public static void main(String[] args) {
		String filename = args[0];
		EventQueue events = new EventQueue(filename);
		for (WMRecorderEvent event : events) {
			System.out.printf("%d.%03d: %s\n", event.time.s, event.time.us/1000, event);
			
		}

	}
	

}
