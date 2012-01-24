package castutils.components;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import cast.AlreadyExistsOnWMException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeFilterComparator;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChangeFilter;
import castutils.castextensions.FilterConfiguration;
import castutils.castextensions.WMEntryQueue;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;

public class WorkingMemoryPlayer extends ManagedComponent {
	private String m_filename;
	private WMEntryQueue<Ice.Object> m_events; 
	private FilterConfiguration m_filterConfig = null;
	private boolean m_restoreState = false;
	
	@Override
	protected void configure(Map<String, String> _config) {
		super.configure(_config);
		
		if (_config.containsKey("--state"))
			m_restoreState = true;
		
		String fname = _config.get("--file");
		if (fname != null) {
			m_filename = fname;
		}
		else {
			m_filename = getSubarchitectureID() + ".log";
		}
		m_events = new WMEntryQueue<Ice.Object>(this, m_filename, Ice.Object.class);

		String cname = _config.get("--config");
		if (cname != null) {
			m_filterConfig = new FilterConfiguration(cname);
		}
	}
	
	@Override
	protected void runComponent() {
		if (m_restoreState)
			restoreState();
		else
			replayEvents();
	}
	
	private void restoreState() {
		Map<WorkingMemoryAddress, Ice.Object> map = new HashMap<WorkingMemoryAddress, Ice.Object>();
		for (WMEntryQueueElement<Ice.Object> elem : m_events) {
			if (elem.getEntry() != null) {
				map.put(elem.getEvent().address, elem.getEntry());
			}
			else {
				map.remove(elem.getEvent().address);
			}
		}
		for (Entry<WorkingMemoryAddress, Ice.Object> e: map.entrySet()) {
			try {
				log("Add " +e.getValue().getClass().getName() + " to address " + e.getKey().id);
				addToWorkingMemory(e.getKey(), e.getValue());
			} catch (AlreadyExistsOnWMException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (DoesNotExistOnWMException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (UnknownSubarchitectureException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
		}
	}
	
	private void replayEvents() {
		WMEntryQueueElement<Ice.Object> next = m_events.poll();
		while (isRunning() && next != null) {
			CASTTime now = getCASTTime();
			CASTTime eventTime = next.getEvent().timestamp;
			long dt = 1000000 * (eventTime.s - now.s) + (eventTime.us - now.us);
			log(String.format("Next event (%s) in %d ms.", next, dt/1000));
			if (dt > 1000) {
				sleepComponent(dt/1000);
			}
			applyEvent(next);
			next = m_events.poll();
		}
	}
	
	private synchronized void applyEvent(WMEntryQueueElement<Ice.Object> event) {
		if (m_filterConfig != null && !m_filterConfig.isEmpty()) {
			boolean found = false;
			for (WorkingMemoryChangeFilter f : m_filterConfig) {
				if (WorkingMemoryChangeFilterComparator.allowsChange(f, event.getEvent())) {
					found = true;
					break;
				}
			}
			if (!found) {
				log("event has been filtered");
				return;
			}
		}
		
		WorkingMemoryAddress wma = event.getEvent().address; 
		try {
			switch (event.getEvent().operation) {
			case ADD:
				addToWorkingMemory(wma, event.getEntry());
				break;
			case OVERWRITE:
				getMemoryEntry(wma, Ice.ObjectImpl.class);
				overwriteWorkingMemory(wma, event.getEntry());
				break;
			case DELETE:
				deleteFromWorkingMemory(wma);
				break;
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
		WMEntryQueue<Ice.Object> events = new WMEntryQueue<Ice.Object>(null, filename,Ice.Object.class);
		for (WMEntryQueueElement<Ice.Object> event : events) {
			CASTTime t = event.getEvent().timestamp;
			System.out.printf("%d.%03d: %s\n", t.s, t.us/1000, event);
			
		}

	}
	

}
