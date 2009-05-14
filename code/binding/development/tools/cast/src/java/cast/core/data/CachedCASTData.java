/**
 * 
 */
package cast.core.data;

import cast.architecture.abstr.WorkingMemoryReaderProcess;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;

/**
 * @author nah
 * 
 */
public class CachedCASTData<T> {

	private int m_version;

	private T m_data;

	private final String m_id;

	private String m_subarchitecture;

	private final Class<T> m_cls;

	private final WorkingMemoryReaderProcess m_component;

	/**
	 * @param _component
	 * @param _id
	 * @param _subarchitecture
	 * @param _cls
	 *            The stored class. Allows safer type-checking.
	 * 
	 */
	public CachedCASTData(WorkingMemoryReaderProcess _component, String _id,
			String _subarchitecture, Class<T> _cls) {
		m_version = -1;
		m_component = _component;
		m_id = _id;
		m_subarchitecture = _subarchitecture;
		m_cls = _cls;
		m_data = null;
	}

	public CachedCASTData(WorkingMemoryReaderProcess _component, String _id,
			Class<T> _cls) {
		this(_component, _id, null, _cls);
	}

	// / updates the local data if necessary
	private void _updateData() throws SubarchitectureProcessException {
		String subarch = subarchitectureID();
		if (!m_component.existsOnWorkingMemory(m_id, subarch)) {
			throw new DoesNotExistOnWMException(new WorkingMemoryAddress(m_id,
					subarch),
					"CachedCASTData::_updateData: Entry does not exist on wm. Was looking for id: "
							+ m_id + "  in subarchitecture "
							+ m_subarchitecture);
		}
		int new_version = m_component.getVersionNumber(m_id, subarch);
		// m_component.log("CACHE: new_version: " +
		// lexical_cast<std::string>(new_version) + " m_version: " +
		// lexical_cast<std::string>(m_version));
		if (m_version < new_version) {
			// System.out.println("updating cache for class: " + m_cls);
			CASTData<?> data_ptr = m_component.getWorkingMemoryEntry(m_id,
					subarch);
			m_data = m_cls.cast(data_ptr.getData());
			m_version = new_version;
		}
		// else {
		// System.out.println("wwwwwwwwwwwwwwwoooooooooooooooooooooo not
		// updating... saviing effort");
		// }
	}

	/**
	 * @return the data
	 * @throws SubarchitectureProcessException
	 */
	public T getData() throws SubarchitectureProcessException {
		_updateData();
		return m_data;
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

	/**
	 * @return
	 */
	public String getID() {
		return m_id;
	}

}
