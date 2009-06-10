/**
 * 
 */
package binding.abstr;

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

	private CASTData<BindingProxy> m_proxyCache;

	private CASTData<BindingUnion> m_unionCache;

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
		    setBindingSA(m_subarchitectureID);
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
		m_proxyCache = new CASTDataCache<BindingProxy>(this, _bindingSA,
				BindingProxy.class);
		m_unionCache = new CASTDataCache<BindingUnion>(this, _bindingSA,
				BindingUnion.class);
	}

	/**
	 * @param _id
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	public BindingProxy getProxy(String _id)
			throws SubarchitectureProcessException {
		return m_proxyCache.get(_id).getData();
	}

	/**
	 * @param _id
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	public BindingUnion getUnion(String _id)
			throws SubarchitectureProcessException {
		return m_unionCache.get(_id).getData();
	}

}
