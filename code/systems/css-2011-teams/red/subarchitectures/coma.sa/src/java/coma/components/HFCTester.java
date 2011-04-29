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
		super.configure(args);
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
		log("simple ICE connection test: ping");
		log(m_hfcserver.ping());

		
		String query = "SELECT ?x ?y ?z where ?x ?y ?z";
//		String query = "SELECT ?x ?y ?z ?num where ?x ?y ?z ?num";
		
		QueryResults _results = m_hfcserver.querySelect(query);
		log("query results:");

		log("Query: " + _results.query);
		log("Variable to array position mapping: " + _results.varPosMap.toString());
		log("Variable result bindings: ");

		for (int i = 0; i < _results.bt.length; i++) {
			String[] _currLine = _results.bt[i];
			StringBuffer _currLineResult = new StringBuffer();
			for (int j = 0; j < _currLine.length; j++) {
				String _res  = _currLine[j];
				_currLineResult.append(_res + " ");
			}
			log(_currLineResult);
		}
		
		
		query = "SELECT ?x ?y where <dora:corridor> ?x ?y ";
		
		_results = m_hfcserver.querySelect(query);
		log("query results:");

		log("Query: " + _results.query);
		log("Variable to array position mapping: " + _results.varPosMap.toString());
		log("Variable result bindings: ");

		for (int i = 0; i < _results.bt.length; i++) {
			String[] _currLine = _results.bt[i];
			StringBuffer _currLineResult = new StringBuffer();
			for (int j = 0; j < _currLine.length; j++) {
				String _res  = _currLine[j];
				_currLineResult.append(_res + " ");
			}
			log(_currLineResult);
		}
		
		
		
		query = "SELECT ?x ?y ?num where <dora:corridor> ?x ?y ?num";
		
		_results = m_hfcserver.querySelect(query);
		log("query results:");

		log("Query: " + _results.query);
		log("Variable to array position mapping: " + _results.varPosMap.toString());
		log("Variable result bindings: ");

		for (int i = 0; i < _results.bt.length; i++) {
			String[] _currLine = _results.bt[i];
			StringBuffer _currLineResult = new StringBuffer();
			for (int j = 0; j < _currLine.length; j++) {
				String _res  = _currLine[j];
				_currLineResult.append(_res + " ");
			}
			log(_currLineResult);
		}
	
	}
	 

}
