package coma.components;

import java.util.Map;

import comadata.ComaReasonerInterfacePrx;

import Ice.Identity;
import Ice.ObjectPrx;
import cast.architecture.ManagedComponent;


public class TestWriter extends ManagedComponent {

		Identity m_reasoner_id;
		ComaReasonerInterfacePrx m_comareasoner;
	
	public void configure(Map<String, String> args) {
		log("configure() called");
		m_reasoner_id = new Identity();
		m_reasoner_id.name="";
		m_reasoner_id.category="ComaReasoner";

		if (args.containsKey("--reasoner-name")) {
			m_reasoner_id.name=args.get("--reasoner-name");
		}
	}

	public void start() {
		log("Initiating connection to server " + m_reasoner_id.category + "::" + m_reasoner_id.name + "...");
		ObjectPrx base = getObjectAdapter().createProxy(m_reasoner_id);
		m_comareasoner = comadata.ComaReasonerInterfacePrxHelper.checkedCast(base);
	}
	
	protected void runComponent() {
		println(m_comareasoner.testReverseString("12345"));
		for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
			println(_currIns);
		}
		m_comareasoner.addInstance("<http://www.dfki.de/cosy/officeenv.owl#room2>", "<http://www.dfki.de/cosy/officeenv.owl#Room>");
		for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
			println(_currIns);
		}
	}
	 

}
