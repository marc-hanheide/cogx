package coma.components;

import java.util.Map;

import coma.reasoning.CrowlWrapper;
import coma.reasoning.ReasonerException;

import Ice.Current;
import Ice.Identity;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;

public class ComaReasoner extends ManagedComponent {
	
	Identity id;
	
	/**
	 * Dunno whether this is still necessary in CAST2... 
	 */
	public ComaReasoner() {
		super();
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	}
	
	protected void configure(Map<String, String> args) {
		// establish ICE identity of the ComaReasonerInterface
		id = new Identity();
		id.name=this.getComponentID(); // use my component name
		id.category="ComaReasoner";
		
		if (args.containsKey("--crowl-cfg")) {
			getObjectAdapter().add(new ComaReasonerInterfaceI(args.get("--crowl-cfg"), m_bLogOutput), id);
	       	// done registering ComaReasonerInterface	
		} else {
			try {
				throw new CASTException("You need to specify a crowl config file using the --crowl-cfg flag! Exiting...");
			} catch (CASTException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				System.exit(0);
			}
		}
		
       	
	}

	
	
	/**
	 * ComaReasonerInterfaceI implements the functions specified
	 * in the SLICE definition.
	 * 
	 * This is the ICE object that handles requests to the reasoner.
	 * 
	 * @author zender
	 *
	 */
	public class ComaReasonerInterfaceI extends comadata._ComaReasonerInterfaceDisp {

		CrowlWrapper m_reasoner;
		
		public ComaReasonerInterfaceI(String _crowl_cfg_file, boolean _logging) {
			try {
				m_reasoner = new CrowlWrapper(_crowl_cfg_file.toLowerCase(), _logging);
			} catch (ReasonerException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		public String testReverseString(String s, Current __current) {
			int i, len = s.length();
		    StringBuffer dest = new StringBuffer(len);

		    for (i = (len - 1); i >= 0; i--)
		      dest.append(s.charAt(i));
			return dest.toString();
		}

		public String[] getAllInstances(String concept, Current __current) {
			return m_reasoner.getInstances(concept).toArray(new String[0]);
		}

		public boolean addInstance(String instance, String concept, Current __current) {
			try {
				m_reasoner.addInstance(instance, concept);
			} catch (ReasonerException e) {
				e.printStackTrace();
				return false;
			}
			return true;
		}

		public boolean addRelation(String instance1, String relation, String instance2, Current __current) {
			m_reasoner.assertRelation(instance1, relation, instance2);
			return true;
		}

		
		
		
	}
	
}
