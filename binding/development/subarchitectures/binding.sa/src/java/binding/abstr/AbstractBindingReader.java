/**
 * 
 */
package binding.abstr;

import java.util.Properties;

import BindingData.*;
import binding.common.BindingComponentException;

import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.core.data.CASTDataCache;

/**
 * "Convenience" class for reading things from binding working memory.
 * 
 * @author nah
 * 
 */
public abstract class AbstractBindingReader extends PrivilegedManagedProcess {

	/**
	 * defines the ID of binding SA, must be set via configure
	 */
	private String m_bindingSA;

	private CASTDataCache<BindingProxy> m_proxyCache;

	private CASTDataCache<BindingUnion> m_unionCache;

	/**
	 * @param _id
	 */
	public AbstractBindingReader(String _id) {
		super(_id);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderWriterProcess#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		super.configure(_config);
		if (_config.containsKey("-bsa")) {
		    setBindingSA(_config.getProperty(BINDING_SUBARCH_CONFIG_KEY.value));
		    log("setting binding subarch to: " + m_bindingSA);
		} else if (_config.containsKey("--bsa")) {
		    setBindingSA(_config.getProperty("--bsa"));
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
