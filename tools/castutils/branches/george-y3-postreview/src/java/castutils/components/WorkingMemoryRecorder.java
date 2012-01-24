package castutils.components;

import java.util.Map;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.FilterConfiguration;
import castutils.castextensions.WMEntryQueue;

public class WorkingMemoryRecorder extends ManagedComponent {
	private String filename;
	private WMEntryQueue<Ice.Object> m_events;
	private FilterConfiguration m_filterConfig = null;

	@Override
	protected void configure(Map<String, String> _config) {
		super.configure(_config);
		String fname = _config.get("--file");
		if (fname != null) {
			filename = fname;
		} else {
			filename = getSubarchitectureID() + ".log";
		}
		String cname = _config.get("--config");
		if (cname != null) {
			m_filterConfig = new FilterConfiguration(cname);
		}
	}

	@Override
	protected void start() {
		super.start();
		m_events = new WMEntryQueue<Ice.Object>(this, Ice.Object.class);
		if (m_filterConfig == null || m_filterConfig.isEmpty()) {
			addChangeFilter(ChangeFilterFactory.createChangeFilter("",
					WorkingMemoryOperation.WILDCARD, "", "", "",
					FilterRestriction.LOCALSA), m_events);
		} else {
			for (WorkingMemoryChangeFilter f : m_filterConfig) {
				addChangeFilter(f, m_events);
			}
		}
	}

	/*
	 * public void workingMemoryChanged(WorkingMemoryChange _wmc) throws
	 * CASTException { if (_wmc.operation == WorkingMemoryOperation.DELETE) {
	 * m_events.push(_wmc.operation, _wmc.address, _wmc.timestamp, null,
	 * _wmc.src); } else { System.out.println(_wmc.operation); try {
	 * Ice.ObjectImpl obj = getMemoryEntry(_wmc.address, Ice.ObjectImpl.class);
	 * m_events.push(_wmc.operation, _wmc.address, _wmc.timestamp, obj,
	 * _wmc.src); } catch (Exception e) { m_events.push(_wmc.operation,
	 * _wmc.address, _wmc.timestamp, null, _wmc.src); } } }
	 */

	@Override
	protected void runComponent() {
		while (isRunning()) {
			sleepComponent(1000);
		}
		log("saving history to " + filename);
		m_events.save(filename);
	}

}
