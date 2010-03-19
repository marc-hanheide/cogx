package wmrecorder;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryOperation;

public class EventQueue implements Iterable<WMRecorderEvent> {
	private List<WMRecorderEvent> m_events = new LinkedList<WMRecorderEvent>();
	private Map<WorkingMemoryAddress, Ice.ObjectImpl> m_state = new HashMap<WorkingMemoryAddress, Ice.ObjectImpl>();
	
	public EventQueue() {}
	
	public EventQueue(String fname) {
		try
		{
			FileInputStream fis = new FileInputStream(fname);
			ObjectInputStream in = new ObjectInputStream(fis);
			int count = in.readInt();
			for (int i=0; i < count; i++) {
				WMRecorderEvent e = (WMRecorderEvent) in.readObject();
				this.push(e);
			}
			in.close();
		}
		catch(IOException e)
		{
			e.printStackTrace();
		}
		catch(ClassNotFoundException e)
		{
			e.printStackTrace();
		}
		
	}
	
	public WMRecorderEvent removeFirst() {
		if (m_events.isEmpty())
			return null;
		return m_events.remove(0);
	}

	public Iterator<WMRecorderEvent> iterator() {
		return m_events.iterator();
	}
	
	public void push(WorkingMemoryOperation _type, WorkingMemoryAddress _addr, 
			CASTTime _time, Ice.ObjectImpl _obj, String _component) {
		push(new WMRecorderEvent(_type, _addr, _time, _obj, _component));
	}
	
	public void push(WMRecorderEvent event) {
		if (!m_events.isEmpty()) {
			CASTTime prev_t = m_events.get(m_events.size()-1).time;
			assert event.time.s > prev_t.s || (event.time.s == prev_t.s && event.time.us >= prev_t.us);
		}
		m_events.add(event);
		if (event.type == WorkingMemoryOperation.DELETE) {
			assert m_state.containsKey(event.address);
			m_state.remove(event.address);
		}
		else {
			m_state.put(event.address, event.object);
		}
	}
	
	public void saveHistory(String fname) {
		 try {
			 FileOutputStream fos = new FileOutputStream(fname);
			 ObjectOutputStream out = new ObjectOutputStream(fos);
			 out.writeInt(m_events.size());
			 for (WMRecorderEvent e : m_events) {
				 out.writeObject(e);
			 }
			 out.close();
		 }
		 catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void saveState(String fname) {
		 try {
			 FileOutputStream fos = new FileOutputStream(fname);
			 ObjectOutputStream out = new ObjectOutputStream(fos);
			 out.writeInt(m_state.size());
			 for (Entry<WorkingMemoryAddress, Ice.ObjectImpl> e : m_state.entrySet()) {
				 WMRecorderEvent event = new WMRecorderEvent(WorkingMemoryOperation.ADD, e.getKey(), new CASTTime(0,0), e.getValue(), "");
				 out.writeObject(event);
			 }
			 out.close();
		 }
		 catch (IOException e) {
			e.printStackTrace();
		}
	}
	
}
