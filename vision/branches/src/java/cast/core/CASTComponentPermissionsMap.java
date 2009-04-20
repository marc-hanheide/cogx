package cast.core;

import java.util.Hashtable;

import cast.cdl.WorkingMemoryPermissions;


public class CASTComponentPermissionsMap {

	
	private static class PermAndCheck extends Pair<WorkingMemoryPermissions, Boolean> {
		PermAndCheck(WorkingMemoryPermissions _a, boolean _b) {
			super(_a, _b);
		}
	}
	
	private final Hashtable<String, PermAndCheck> m_localPermissions;
	private final Hashtable<String, Hashtable<String, PermAndCheck>> m_globalPermissions;
	
	public CASTComponentPermissionsMap(String _subarch) {
		//keep local permissions separate for ease
		m_localPermissions = new Hashtable<String, PermAndCheck>();
		m_globalPermissions = new Hashtable<String, Hashtable<String,PermAndCheck>>();
		//store local permissions in global map for generality
		m_globalPermissions.put(_subarch, m_localPermissions);
	}	
	
	public WorkingMemoryPermissions setPermissions(String _id, WorkingMemoryPermissions _permissions) {
		PermAndCheck check =  m_localPermissions.put(_id, new PermAndCheck(_permissions,true));
		if(check == null) {
			return null;
		}
		else {
			return check.m_first;
		}
	}
	
	public WorkingMemoryPermissions setPermissions(String _id, String _subarch, WorkingMemoryPermissions _permissions) {
		if(!m_globalPermissions.containsKey(_subarch)) {
			m_globalPermissions.put(_subarch, new Hashtable<String, PermAndCheck>());
		}
		PermAndCheck check = m_globalPermissions.get(_subarch).put(_id, new PermAndCheck(_permissions,true));
		
		if(check == null) {
			return null;
		}
		else {
			return check.m_first;
		}
	}

	public WorkingMemoryPermissions getPermissions(String _id) {
		return m_localPermissions.get(_id).m_first;
	}
	
	public WorkingMemoryPermissions getPermissions(String _id, String _subarch) {
		if(!m_globalPermissions.containsKey(_subarch)) {
			return null;
		}
		PermAndCheck check = m_globalPermissions.get(_subarch).get(_id);
		if(check == null) {
			return null;
		}
		else {
			return check.m_first;
		}
		
	}

	public boolean hasPermissions(String _id) {
		return getPermissions(_id) != null;
	}
	
	public boolean hasPermissions(String _id, String _subarch) {
		return getPermissions(_id, _subarch) != null;
	}

	public WorkingMemoryPermissions removePermissions(String _id) {
		return m_localPermissions.remove(_id).m_first;
	}
	
	public WorkingMemoryPermissions removePermissions(String _id, String _subarch) {
		if(!m_globalPermissions.containsKey(_subarch)) {
			return null;
		}
		return m_globalPermissions.get(_subarch).remove(_id).m_first;
	}
	
	
	
	
	public boolean needsConsistencyCheck(String _id) {
		return m_localPermissions.get(_id).m_second;
	}
	
	public boolean needsConsistencyCheck(String _id, String _subarch) {
		if(!m_globalPermissions.containsKey(_subarch)) {
			return false;
		}
		return m_globalPermissions.get(_subarch).get(_id).m_second;
	}
	
	public void consistencyChecked(String _id) {
		 m_localPermissions.get(_id).m_second = false;
	}
	
	public void consistencyChecked(String _id, String _subarch) {
		if(!m_globalPermissions.containsKey(_subarch)) {
			return ;
		}
		 m_globalPermissions.get(_subarch).get(_id).m_second = false;
	}
	
	
	
}
