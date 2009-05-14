/**
 * 
 */
package cast.core.data;

import java.util.HashMap;

import cast.architecture.abstr.WorkingMemoryReaderProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;

/**
 * @author nah
 * 
 */
public class CASTDataCache<T> {

	private final HashMap<String, CachedCASTData<T>> m_cache;

	private final WorkingMemoryReaderProcess m_component;

	private String m_subarchitecture;

	private final Class<T> m_cls;

	/**
	 * 
	 */
	public CASTDataCache(WorkingMemoryReaderProcess _component,
			String _subarchitecture, Class<T> _cls) {
		m_cache = new HashMap<String, CachedCASTData<T>>();
		m_component = _component;
		m_subarchitecture = _subarchitecture;
		m_cls = _cls;
	}

	/**
	 * 
	 */
	public CASTDataCache(WorkingMemoryReaderProcess _component, Class<T> _cls) {
		this(_component, null, _cls);
	}

	public CachedCASTData<T> get(String _id) {
		if (m_cache.containsKey(_id)) {
			return m_cache.get(_id);
		}
		else {
			// create new cached data
			CachedCASTData<T> ccd = new CachedCASTData<T>(m_component, _id,
					m_subarchitecture, m_cls);
			m_cache.put(_id, ccd);
			return ccd;
		}
	}

	public T getData(String _id) throws SubarchitectureProcessException {
		return get(_id).getData();
	}

	/**
	 * @param _subarchitecture
	 *            the subarchitecture to set
	 */
	public void setSubarchitectureID(String _subarchitecture) {
		m_subarchitecture = _subarchitecture;
	}

	/**
	 * @return
	 */
	public final String subarchitectureID() {
		if (m_subarchitecture == null) {
			return m_component.getSubarchitectureID();
		}
		else {
			return m_subarchitecture;
		}
	}

}
