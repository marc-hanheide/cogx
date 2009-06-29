/**
 * 
 */
package cast.examples;

import java.util.Vector;

import Ice.Current;
import Ice.Identity;
import cast.core.CASTComponent;
import cast.examples.autogen.WordServer;
import cast.examples.autogen._WordServerDisp;

/**
 * Demonstrates how to use simple direct Ice connections in CAST. This server
 * may supply multiple interfaces via aggregation.
 * 
 * @author nah
 * 
 */
public class SimpleAggregateServer extends CASTComponent {

	private Vector<Identity> m_serverIdentities = new Vector<Identity>(0);

	private class WordServerI extends _WordServerDisp {
		// keep warnings away
		private static final long serialVersionUID = 1L;

		public String getNewWord(Current __current) {
			return "buerhgh";
		}

	}

	/**
	 * 
	 */
	public SimpleAggregateServer() {
		// Should not do connection things in the constructor as we can't be
		// sure anyone else is ready yet. Also the variables the connections
		// depend on (component id etc.) will not be set yet.
	}

	@Override
	protected void start() {
		WordServerI ws = new WordServerI();
		registerIceServer(WordServer.class, ws);
	}

	public String getNewWord(Current __current) {
		return "blah";
	}

	@Override
	public void destroy() {
		for (Identity id : m_serverIdentities) {
			getObjectAdapter().remove(id);
		}
	}

}
