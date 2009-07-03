/**
 * 
 */
package cast.server;

import java.util.Hashtable;

import Ice.Current;
import cast.cdl.ComponentDescription;
import cast.interfaces._ComponentManagerDisp;

/**
 * @author nah
 * 
 */
public class CASTComponentManager extends _ComponentManagerDisp {

	private final Hashtable<String, ComponentDescription> m_descriptions;

	/**
	 * Serial version.
	 */
	private static final long serialVersionUID = 1L;

	public CASTComponentManager() {
		m_descriptions = new Hashtable<String, ComponentDescription>();
	}

	public void addComponentDescription(ComponentDescription _description,
			Current __current) {
		m_descriptions.put(_description.componentName, _description);
	}

	public ComponentDescription getComponentDescription(String _componentID,
			Current __current) {
		ComponentDescription desc = m_descriptions.get(_componentID);
		return desc;
	}

}
