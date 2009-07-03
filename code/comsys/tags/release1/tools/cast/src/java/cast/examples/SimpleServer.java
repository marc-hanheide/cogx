/**
 * 
 */
package cast.examples;

import Ice.Current;
import cast.core.CASTComponent;
import cast.examples.autogen.WordServer;
import cast.examples.autogen._WordServerOperations;
import cast.examples.autogen._WordServerTie;

/**
 * Demonstrates how to use simple direct Ice connections in CAST.
 * 
 * @author nah
 * 
 */
public class SimpleServer extends CASTComponent implements
		_WordServerOperations {


	/**
	 * 
	 */
	public SimpleServer() {
		// Should not do connection things in the constructor as we can't be
		// sure anyone else is ready yet. Also the variables the connections
		// depend on (component id etc.) will not be set yet.
	}


	@Override
	protected void start() {
		registerIceServer(WordServer.class, new _WordServerTie(this));
	}

	public String getNewWord(Current __current) {
		return "blah";
	}


}
