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
		m_reasoner_id.category="coma.components.ComaReasoner";

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
		println("simple ICE connection test: reverse 12345");
		println(m_comareasoner.testReverseString("12345"));
		
		println("get all instances of owl:Thing");
		for (String _currIns : m_comareasoner.getAllInstances("owl:Thing")) {
			_currIns="test" + _currIns;
			println(_currIns);
//			_currIns=_currIns.replace("#", ":");
			for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
				println(_currRelIns);
			}
		}

		println("get all instances of test:PhysicalRoom and their related instances");
		for (String _currIns : m_comareasoner.getAllInstances("test:PhysicalRoom")) {
			_currIns="test" + _currIns;
			println("related instances of " + _currIns);
//			_currIns=_currIns.replace("#", ":");
			for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
				println(_currRelIns);
			}
		}
		
		println("get all instances of test:Place and their related instances");
		for (String _currIns : m_comareasoner.getAllInstances("test:Place")) {
			_currIns="test" + _currIns;
			println("related instances of " + _currIns);
//			_currIns=_currIns.replace("#", ":");
			for (String _currRelIns : m_comareasoner.getRelatedInstances(_currIns)) {
				println(_currRelIns);
			}
		}

	}
	 

}
