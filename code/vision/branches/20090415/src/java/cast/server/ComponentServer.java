/**
 * 
 */
package cast.server;

import Ice.Application;
import Ice.Communicator;
import Ice.Identity;
import Ice.ObjectAdapter;
import Ice.SignalPolicy;
import cast.cdl.JAVASERVERPORT;

/**
 * @author nah
 * 
 */
public class ComponentServer extends Application {

	/**
	 * 
	 */
	public ComponentServer() {
	}

	/**
	 * @param _signalPolicy
	 */
	public ComponentServer(SignalPolicy _signalPolicy) {
		super(_signalPolicy);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see Ice.Application#run(java.lang.String[])
	 */
	@Override
	public int run(String[] arg0) {
		Communicator ic = communicator();
		ObjectAdapter adapter = ic.createObjectAdapterWithEndpoints(
				"ComponentServer1", "default -p " + JAVASERVERPORT.value);

		Identity id = new Identity("ComponentFactory", "ComponentFactory");
		adapter.add(new CASTComponentFactory(), id);
		
		
		Identity manid = new Identity("comp.man", CASTComponentManager.class
				.getCanonicalName());
		adapter.add(new CASTComponentManager(), manid);

		
		//		adapter.addServantLocator(new cast.server.ComponentLocator(), "");
		adapter.activate();
		adapter.waitForDeactivate();
		return 0;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		ComponentServer app = new ComponentServer();
		int status = app.main("cast.server.ComponentServer", args);
		System.exit(status);
	}
}
