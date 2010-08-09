package coma.components;


import java.util.Map;

import comadata.HFCInterfacePrx;
import comadata.QueryResults;

import Ice.Identity;
import cast.CASTException;
import cast.architecture.ManagedComponent;

public class HFCTester extends ManagedComponent {

		Identity m_hfcserver_id;
		HFCInterfacePrx m_hfcserver;
	
	public void configure(Map<String, String> args) {
		log("configure() called");
		m_hfcserver_id = new Identity();
		m_hfcserver_id.name="";
		m_hfcserver_id.category="coma.components.HFCserver";

		if (args.containsKey("--hfcserver-name")) {
			m_hfcserver_id.name=args.get("--hfcserver-name");
		}
	}

	public void start() {
		log("Initiating connection to server " + m_hfcserver_id.category + "::" + m_hfcserver_id.name + "...");
		
		
		try {
			m_hfcserver = getIceServer(m_hfcserver_id.name, comadata.HFCInterface.class, comadata.HFCInterfacePrx.class);
		} catch (CASTException e) {
			e.printStackTrace();
			log("Connection to the HFCServer Ice server at "+ m_hfcserver_id.toString() + " failed! Exiting. (Specify the correct component name using --hfcserver-name)");
			System.exit(-1);
		}	
		
		//ObjectPrx base = getObjectAdapter().createProxy(m_hfcserver_id);
		//m_hfcserver = HFCInterfacePrxHelper.checkedCast(base);
	}
	
	protected void runComponent() {
		println("simple ICE connection test: ping");
		println(m_hfcserver.ping());

		String query = "SELECT ?x ?y ?z where ?x ?y ?z";
		QueryResults _results = m_hfcserver.querySelect(query);
		println("query results:");

		println("Query: " + _results.query);
		println("Variable to array position mapping: " + _results.varPosMap.toString());
		println("Variable result bindings: ");

		for (int i = 0; i < _results.bt.length; i++) {
			String[] _currLine = _results.bt[i];
			StringBuffer _currLineResult = new StringBuffer();
			for (int j = 0; j < _currLine.length; j++) {
				String _res  = _currLine[j];
				_currLineResult.append(_res + " ");
			}
			println(_currLineResult);
		}
	}
	 

}
