/**
 * 
 */
package binding.abstr;

import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

import BindingData.*;
import binding.common.BindingComponentException;

import cast.architecture.WorkingMemoryReaderComponent;
import cast.SubarchitectureComponentException;
import cast.core.CASTData;

/**
 * "Convenience" class for reading things from binding working memory.
 * 
 * @author nah
 * 
 */
public abstract class AbstractBindingReader extends WorkingMemoryReaderComponent {

	/**
	 * defines the ID of binding SA, must be set via configure
	 */
	private String m_bindingSA;

	private HashMap<String, CASTData<BindingProxy>> m_proxyCache;

	private HashMap<String, CASTData<BindingUnion>> m_unionCache;

	/**
	 * @param _id
	 */
	public AbstractBindingReader() {
		super();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderWriterProcess#configure(java.util.Properties)
	 */
	@Override
	public void configure(Map<String, String> _config) {
		super.configure(_config);
		if (_config.containsKey("-bsa")) {
		    setBindingSA(_config.get(BINDINGSUBARCHCONFIGKEY.value));
		    log("setting binding subarch to: " + m_bindingSA);
		} else if (_config.containsKey("--bsa")) {
		    setBindingSA(_config.get("--bsa"));
		    log("setting binding subarch to: " + m_bindingSA);
		}
		else {
		    log("binding subarch not specified, assuming it\'s local to monitor");
		    setBindingSA(getSubarchitectureID());
		}
	}

	/**
	 * @return
	 * @throws BindingComponentException
	 */
	public final String getBindingSA() throws BindingComponentException {
		if (m_bindingSA.equals("")) {
			// throw a runtime exception to
			throw new BindingComponentException(
					"binding sa must be set before use");
		}
		return m_bindingSA;
	}

	/**
	 * @param _bindingSA
	 *            the bindingSA to set
	 */
	protected void setBindingSA(String _bindingSA) {
		m_bindingSA = _bindingSA;

		if (m_proxyCache == null) {
			// once we have the bindingSA initialise caches
			log("initialiising caches with binding sa: " + _bindingSA);
			initCaches(_bindingSA);
		}
	}

	/**
	 * @param _bindingSA
	 */
	private void initCaches(String _bindingSA) {
		m_proxyCache = new HashMap<String, CASTData<BindingProxy>>();
		m_unionCache = new HashMap<String, CASTData<BindingUnion>>();
	}

	/**
	 * @param _id
	 * @return
	 * @throws SubarchitectureComponentException
	 */
	public BindingProxy getProxy(String _id)
			throws SubarchitectureComponentException {
		return m_proxyCache.get(_id).getData();
	}

	/**
	 * @param _id
	 * @return
	 * @throws SubarchitectureComponentException
	 */
	public BindingUnion getUnion(String _id)
			throws SubarchitectureComponentException {
		return m_unionCache.get(_id).getData();
	}

}
