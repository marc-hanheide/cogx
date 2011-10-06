package coma.components;

import java.io.ByteArrayOutputStream;
import java.io.OutputStream;
import java.util.Map;

import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.ResultSetFormatter;

import coma.reasoning.CrowlWrapper;
import coma.reasoning.ReasonerException;
import comadata.ComaReasonerInterface;

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
		this.registerIceServer(ComaReasonerInterface.class, new ComaReasonerInterfaceI(args.get("--crowl-cfg"), m_bLogOutput));
		
//		// establish ICE identity of the ComaReasonerInterface
//		id = new Identity();
//		id.name=this.getComponentID(); // use my component name
//		id.category="ComaReasoner";
//		
//		if (args.containsKey("--crowl-cfg")) {
//			getObjectAdapter().add(new ComaReasonerInterfaceI(args.get("--crowl-cfg"), m_bLogOutput), id);
//	       	// done registering ComaReasonerInterface	
//		} else {
//			try {
//				throw new CASTException("You need to specify a crowl config file using the --crowl-cfg flag! Exiting...");
//			} catch (CASTException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//				System.exit(0);
//			}
//		}
		
       	
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

		/**
		 * 
		 */
		private static final long serialVersionUID = -8225078913601911718L;
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

		public String[] getRelatedInstances(String instance, Current __current) {
			return m_reasoner.getRelatedInstances(instance).toArray(new String[0]);
		}
		public String[] getRelatedInstancesByRelation(String instance, String relation, Current __current) {
			return m_reasoner.getRelatedInstances(instance, relation).toArray(new String[0]);
		}
		
		public String[] getImmediateRelatedInstancesByRelation(String instance,
				String relation, Current current) {
			return m_reasoner.getImmediateRelatedInstances(instance, relation).toArray(new String[0]);
		}

		public String[] getInverseRelatedInstancesByRelation(String instance,
				String relation, Current current) {
			return m_reasoner.getInverseRelatedInstances(instance, relation).toArray(new String[0]);
		}

		public String[] getInstancesByPropertyValue(String property,
				String value, Current current) {
			return m_reasoner.getInstancesByPropertyValue(property, value).toArray(new String[0]);
		}

		public String[] getAllConcepts(String instance, Current __current) {
			return m_reasoner.getAllConcepts(instance).toArray(new String[0]);
		}
		public String[] getBasicLevelConcepts(String instance, Current current) {
			return m_reasoner.getBasicLevelConcepts(instance).toArray(new String[0]);
		}

		public String[] getMostSpecificConcepts(String instance, Current current) {
			return m_reasoner.getMostSpecificConcepts(instance).toArray(new String[0]);
		}

		public String[] getAllSubconcepts(String concept, Current current) {
			return m_reasoner.getAllSubConcepts(concept).toArray(new String[0]);
		}
		
		public String[] getPropertyValues(String instance, String property,
				Current current) {
			return m_reasoner.getPropertyValues(instance, property).toArray(new String[0]);
		}

		public String[] getRelationsBetweenInstances(String instance1,
				String instance2, Current current) {
			return m_reasoner.getRelations(instance1, instance2).toArray(new String[0]);
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

		public boolean deleteInstance(String instance, Current current) {
			return m_reasoner.deleteInstance(instance);
		}

		public boolean deleteRelation(String instance1, String relation, String instance2, Current current) {
			return m_reasoner.deleteRelation(instance1, relation, instance2);
		}

		public boolean isInstanceOf(String instance, String concept, Current __current) {
			return m_reasoner.isInstanceOf(instance, concept);
		}

		
		public boolean areInstancesRelated(String instance1, String relation,
				String instance2, Current current) {
			return m_reasoner.areInstancesRelated(instance1, instance2, relation);
		}

		
		public boolean isSubRelation(String relation1, String relation2,
				Current current) {
			return m_reasoner.isSubRelation(relation1, relation2);
		}

		public String executeSPARQL(String sparqlQuery, Current current) {
			ResultSet _results = m_reasoner.executeSPARQLQuery(sparqlQuery);
			OutputStream stream = new ByteArrayOutputStream();
			String returnString = "";
			if (_results==null) {
				returnString = "SPARQL Syntax Error!";
			}
			else { 
				ResultSetFormatter.out(stream, _results);
				returnString = stream.toString();
			}
			return returnString;
		}

		
		
	}
	
}
